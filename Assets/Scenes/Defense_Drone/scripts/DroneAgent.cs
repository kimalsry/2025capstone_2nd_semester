using DefenseDrone;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO; // ✅ CSV 저장용
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Globalization;

// =============================
// TriggerRelay: 트리거 이벤트 릴레이
// =============================
public class TriggerRelay : MonoBehaviour
{
    public System.Action<Collider> OnEnter;

    private void OnTriggerEnter(Collider other)
    {
        Debug.Log($"[TriggerRelay] {gameObject.name}.OnTriggerEnter: {other.gameObject.name}, Tag: {other.tag}");
        OnEnter?.Invoke(other);
    }
}

// =============================
// DroneAgent (Single-Primary-Ray Reward)
// =============================
public class DroneAgent : Agent
{
    [Header("드론 제어 연결")]
    public DroneController controller;

    [Header("추격 대상(침입 드론)")]
    public List<GameObject> invaders = new List<GameObject>();
    public bool autoDiscoverInvaders = false;              // 인스펙터에 안 넣으면 true로 두고 태그로 한 번만 자동 찾기
    public string invaderTag = "Invasion_Drone";

    [Header("에피소드당 침입 드론 수 (보통 3)")]
    public int maxInvadersPerEpisode = 3;
    
    public Vector3 invaderSpawnCenter = new Vector3(0, 25, 0);
    public float invaderSpawnRadius = 30f;
    public float invaderSpeed = 8f;
    [Header("방어 드론 스폰")]
    public Vector3 defenseSpawnCenter = new Vector3(0, 15, 0);
    public float defenseSpawnRadius = 5f;

    [Header("침입/요격 판정")]
    public float centerProximityRadius = 3f;
    public float interceptProximityRadius = 3f;

    [Header("관측(Perception Ray 11방향)")]
    public int numRays = 11;
    public float rayMaxDist = 50f;
    public float rayAngleSpan = 180f;
    public bool debugRaycast = false;

    [Header("Primary Ray Lock-On")]
    public bool usePrimaryRayLockOn = true;
    public float lockOnStickyTime = 1.0f;

    [Header("리워드 가중치")]
    public float rewardAlignWeight = 0.5f;
    public float rewardCloseShapingWeight = 1.0f;
    public float rewardCloseStickyWeight = 2.0f;
    public float rewardCloseSpeedWeight = 0.3f;
    public float rewardOriginStayWeight = 0.1f;
    public float rewardRayShrinkWeight = 1.5f;
    public float penaltyRetreatWeight = 0.2f;
    public float penaltyMultiRayWeight = 0.05f;
    public float penaltyNoDetectWeight = 0.05f;

    [Header("근접 거리 임계값")]
    public float closeDistThreshold = 8f;
    public float closeSpeedDistThreshold = 10f;

    [Header("🔋 전력소비 및 이동거리 추적")]
    public bool trackPowerConsumption = true;
    public bool debugPowerConsumption = false;

    // =============================
    // IEEE 논문 기반 물리 파라미터
    // =============================
    private const float P0 = 79.86f;
    private const float Pind = 88.63f;
    private const float Utip = 120f;
    private const float v0 = 4.03f;
    private const float d0 = 0.6f;
    private const float rho = 1.225f;
    private const float s = 0.05f;
    private const float A = 0.503f;
    private const float g = 9.8f;

    // 에피소드별 누적값
    private float episodeTotalEnergy = 0f;
    private float episodeTotalDistance = 0f;
    private Vector3 lastPosition;
    private float episodeStartTime;

    // =============================
    // 내부 변수
    // =============================

    private int spawnedCount = 0;
    private int interceptedCount = 0;
    private int invadedCount = 0;
    private bool episodeEnded = false;
    private bool episodeMetricsSent = false;

    private GameObject currentTarget;
    
    private float primaryRayLockTimer = 0f;

    private float[] rayHitDistances;
    private bool[] rayHitFlags;

    private List<GameObject> activeInvaders = new List<GameObject>();
    private HashSet<GameObject> alreadyIntercepted = new HashSet<GameObject>();

    // Reward 관련
    private float lastDistToTarget = float.MaxValue;
    private Vector3 lastTargetPos;
    private float lastPrimaryRayDist = float.MaxValue;
    private float cumulativeRayDistDecreaseReward = 0f;

    [Header("맵 경계 (로컬 좌표 기준)")]
    public float boundMinX = -50f;
    public float boundMaxX = 50f;
    public float boundMinZ = -50f;
    public float boundMaxZ = 50f;

    [Header("Defense Drone Y 경계")]
    public float defenseMaxY = 20f;

    [Header("학습 파라미터")]
    public int maxSteps = 40000;
    public float interceptDistance = 1.5f;

    [Header("행동 제어 매개변수")]
    [Range(0f, 1f)] public float actionSmoothing = 0.2f;

    // ===== 리워드 파라미터 =====
    [Header("R_closeSpeed: 폐쇄속도 보상 (선택)")]
    public bool enableCloseSpeed = true;
    public float closeSpeedGain = 2.0f; // kv
    public float seenFactorWhenSeen = 0.9f;
    public float seenFactorWhenMiss = 0.6f;

    [Header("R_align × R_closeShaping (곱, 최우선 리워드)")]
    public bool enableAlignClose = true;
    public float alignBonusCoeff = 0.08f;
    public float alignmentPower = 0.8f;
    public float positiveCloseGain = 0.25f;
    public float positiveCloseMaxDist = 5f;

    [Header("R_closeSticky: 근접 스틱키 보너스 (seen 2순위)")]
    public bool enableStickyBonus = true;
    public bool enableStickyBonusOverride = false;
    public float stickyCapDist = 1.8f;
    public float stickyBonusPerStep = 0.06f;

    [Header("R_nonPerception: 미감지 패널티")]
    public float nonPerceptionPenaltyPerSec = 0.2f;

    [Header("R_closeBoundary: 경계 근접 증폭(조건부)")]
    public float closeBoundaryThreshold = 5f;
    public float closeBoundaryLambda = 1.2f;
    public bool boundaryAmplifyConditional = true;

    [Header("이벤트 리워드")]
    public float interceptSuccessReward = 10.0f;
    public float invaderSuccessPenalty = -4.0f;
    public float floorCollisionPenalty = -2.0f;

    // ===== Perception Ray 설정 =====
    [Header("Perception Ray 설정 (수평 스팬 레이들)")]
    public float perceptionRayLength = 90f;
    public LayerMask perceptionMask = ~0;
    public float[] perceptionAnglesDeg = new float[]
    { -75f, -60f, -45f, -30f, -15f, 0f, 15f, 30f, 45f, 60f, 75f };
    [Range(-45f, 45f)] public float perceptionTiltDeg = -5f;
    public bool perceptionDebugRay = false;

    // ===== Primary Ray 선택/락온 =====
    [Header("Primary Ray 선택/락온")]
    public bool lockOnUntilLost = true;
    public float lostReleaseDelay = 0.15f;
    public bool preferClosestHit = true;

    // 내부 상태(Primary)
    private int primaryRayIndex = -1;
    private float primaryLastHitDist = -1f;
    private float primaryLastSeenTime = -1f;

    [Header("탐지/스폰 설정")]
    public float detectRange = 60f;
    public LayerMask detectMask = ~0;

    [Header("커리큘럼 단계")]
    public List<TrainingPhase> curriculumPhases = new List<TrainingPhase>();

    [Header("Spawn Rules (By Tag)")]
    public string targetTag = "Target";
    public string defenseTag = "Defense_Drone";
    public string invasionTagForRespawn = "Invasion_Drone";

    public float targetCircleRadius = 0.0f;
    public float defenseSpawnY = 10.0f;
    public float invasionMinRadius = 50.0f;
    public float invasionBoundsHalf = 50.0f;
    public float spawnMinY = 10.0f;
    public float spawnMaxY = 20.0f;
    public int sampleMaxTries = 100;

    [Header("트리거 기반 종료 설정 (선택)")]
    public Collider targetFailZone;

    // ===== 침입 성공 영역(XZ 원형) =====
    [Header("Invasion Zone Fail (원형 XZ)")]
    public bool useZoneFail = true;
    public Vector2 zoneCenterXZ = Vector2.zero;
    public float zoneFailRadius = 18f;

    // ===== 미감지 시 원점 머무르기(지배적) =====
    [Header("R_homeWhenUnseen: 미감지 시 원점 머무르기(지배적)")]
    public bool enableHomeWhenUnseen = true;
    public Vector2 homeXZ = Vector2.zero;
    public float homeRadius = 2.5f;
    public float homeDominantGain = 5f;
    public float homeHeadingGain = 0.2f;
    public float homeVelAwayPenaltyGain = 0.3f;

    // ===== 근접 로이터 억제 =====
    [Header("Anti-Loiter Near Target: 근접 로이터 억제")]
    public bool penalizeLoiterNearTarget = true;
    public float loiterNearDist = 3.0f;
    public float loiterPenaltyPerSec = 0.15f;

    // ===== 거리 증가 강한 페널티(후퇴 억제) =====
    [Header("Retreat Penalty: 거리 증가 강한 페널티")]
    public bool enableRetreatPenalty = true;
    public float retreatVclosePenaltyGain = 2.0f;
    public float retreatVcloseExponent = 1.0f;
    public float retreatRayExpandPenaltyGain = 0.9f;
    public float retreatClipMin = -1.0f;

    // ===== R_rayShrink: 레이 거리 감소(1초 창) =====
    [Header("R_rayShrink: 레이 거리 감소(1초 창)")]
    public float rayShrinkGain = 0.8f;
    public float rayStickAngleDeg = 10f;
    public float rayEvalPeriodSec = 1.0f;
    private float rayLastEvalTime = 0f;
    private float rayDistanceAtEval = -1f;
    private Vector3 rayDirAtEval = Vector3.zero;

    // ===== 감지 유지(시간) 보상 =====
    [Header("R_hold: 감지 유지 시간 보상")]
    public bool enablePerceptionHold = true;
    public float perceptionHoldGain = 0.02f;
    public float perceptionHoldScale = 0.2f;
    public float perceptionHoldMaxStreak = 5f;
    private float perceptionStreakSec = 0f;

    // ===== 레이 거리 가중치 =====
    [Header("Ray 거리 가중치 w(d)=exp(-alpha·d)")]
    public bool enableRayDistanceWeight = true;
    public float rayWeightAlpha = 0.05f;
    public float rayWeightMin = 0.2f;
    public float rayWeightMax = 1.0f;

    // ===== Multi-Ray Penalty =====
    [Header("Multi-Ray Seen Penalty: 동시 레이 탐지 억제")]
    public bool enableMultiRayPenalty = true;
    public int multiRayPenaltyStartCount = 4;
    public float multiRayPenaltyGain = 1.2f;

    // ===== Unseen Pitch/Roll Activity Reward =====
    [Header("Unseen Pitch/Roll Activity: 미감지 시 틸트 움직임 보상")]
    public bool enableUnseenTiltActivity = true;
    public float unseenTiltTargetRateDeg = 40f;
    public float unseenTiltActivityGain = 0.6f;
    public float unseenTiltMaxRewardPerStep = 0.03f;

    // ===== Invasion Spawn Timing =====
    [Header("Invasion Spawn Timing")]
    public float invaderSpawnInterval = 2.0f;   // ✅ 2초 간격
    private Coroutine invaderSpawnRoutine;

    // 내부 상태
    private Vector3 initPos;
    private Quaternion initRot;
    private GameObject targetInvader;

    private float currentMoveX, currentMoveZ, currentMoveY, currentPitchCmd, currentRollCmd, currentYaw;

    // 이동 거리 측정용
    private Vector3 prevPosition;
    private float cumulativeDistance = 0f;

    private Rigidbody rb;
    private float lastDistanceToTarget = -1f;

    // 현재 스텝에서 침입 드론을 감지한 레이 개수
    private int currentInvaderRayCount = 0;

    // 이번 에피소드에서 "해결된"(요격 또는 침입성공) 침입 드론 수
    private int resolvedInvaderCount = 0;
    private bool anyZoneFailThisEpisode = false;

    [System.Serializable]
    public class TrainingPhase
    {
        public int startStep = 0;
        public float invaderSpeedMultiplier = 1.0f;
        public float interceptDistance = 3.0f;
    }

    // ===== CSV 로깅 설정 =====
    [Header("CSV 로깅 (Logs/episode_metrics.csv)")]
    public bool enableCsvLogging = true;
    private static string runStamp;
    private string csvPath;
    private bool csvInitialized = false;
    private int episodeIndex = 0;

    private void InitializeCsvLogger()
{
    if (!enableCsvLogging || csvInitialized)
        return;

    // ⭐ 세션(Play)별로 한 번만 타임스탬프 생성
    if (string.IsNullOrEmpty(runStamp))
    {
        runStamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
    }

    // 프로젝트 루트/Logs/episode_metrics_타임스탬프.csv
    string folder = Path.Combine(Application.dataPath, "../Logs");
    if (!Directory.Exists(folder))
        Directory.CreateDirectory(folder);

    string fileName = $"episode_metrics_{runStamp}.csv";   // ⭐ 시간 포함 파일명
    csvPath = Path.Combine(folder, fileName);

    if (!File.Exists(csvPath))
    {
        // 헤더 작성 (⭐ AvgPower_W 추가 반영)
        string header =
            "Episode,Reason,Duration_s,TotalEnergy_J,TotalDistance_m,AvgSpeed_mps,EnergyPerMeter_J_per_m,Intercepted,MaxInvaders,Success,ZoneFail,AvgPower_W,Reward\n";
        File.WriteAllText(csvPath, header);
    }

    csvInitialized = true;
    Debug.Log($"[DroneAgent] CSV Logger initialized at: {csvPath}");
}


private void AppendEpisodeCsv(
    int ep,
    string reason,
    float duration,
    float totalEnergy,
    float totalDistance,
    int intercepted,
    int maxInvaders,
    float successFlag,
    float zoneFailFlag,
    float episodeReward        
)

{
    if (!enableCsvLogging || !csvInitialized)
        return;

    float avgSpeed      = totalDistance > 0f ? totalDistance / duration      : 0f;
    float energyPerMeter= totalDistance > 0f ? totalEnergy   / totalDistance : 0f;
    float avgPower      = totalEnergy / duration;   // ⭐ 평균 전력 (W)

    string line = string.Format(
    CultureInfo.InvariantCulture,
    "{0},{1},{2:F4},{3:F4},{4:F4},{5:F4},{6:F4},{7},{8},{9},{10},{11:F4},{12:F4}\n",
    ep,
    reason,
    duration,
    totalEnergy,
    totalDistance,
    avgSpeed,
    energyPerMeter,
    intercepted,
    maxInvaders,
    successFlag,
    zoneFailFlag,
    avgPower,
    episodeReward         
);

    File.AppendAllText(csvPath, line);
}



    // =============================
    // Unity/ML-Agents 라이프사이클
    // =============================
    public override void Initialize()
    {
        rb = controller.GetComponent<Rigidbody>();
        
        if (autoDiscoverInvaders && invaders.Count == 0)
        {
            GameObject[] found = GameObject.FindGameObjectsWithTag(invaderTag);
            invaders.AddRange(found);
        }
        
        MaxStep = maxSteps;
        if (controller == null) controller = GetComponent<DroneController>();
        initPos = transform.localPosition;
        initRot = transform.localRotation;
        rb = GetComponent<Rigidbody>();

        // 침입 드론 태그 기반 자동 발견 (초기 1회용)
        if (autoDiscoverInvaders && (invaders == null || invaders.Count == 0))
        {
            invaders = SafeFindByTag(invaderTag).Take(maxInvadersPerEpisode).ToList();
        }

        HookTrigger(targetFailZone, OnTargetFailZoneEnter);
        EnsureDefaultCurriculum();

        // ✅ CSV 초기화
        InitializeCsvLogger();
    }

    private void EnsureDefaultCurriculum()
    {
        if (curriculumPhases == null || curriculumPhases.Count == 0)
        {
            curriculumPhases = new List<TrainingPhase>
            {
                new TrainingPhase{ startStep = 0,     invaderSpeedMultiplier = 0.6f, interceptDistance = 2.5f },
                new TrainingPhase{ startStep = 20000, invaderSpeedMultiplier = 1.0f, interceptDistance = 2.0f },
                new TrainingPhase{ startStep = 60000, invaderSpeedMultiplier = 1.3f, interceptDistance = 1.5f },
            };
        }
    }

    public override void OnEpisodeBegin()
    {
        // 🔋 전력소비 및 이동거리 초기화
        episodeTotalEnergy = 0f;
        episodeTotalDistance = 0f;
        lastPosition = transform.position;
        episodeStartTime = Time.time;
        resolvedInvaderCount = 0;
        anyZoneFailThisEpisode = false;
        interceptedCount = 0; 

        episodeMetricsSent = false;  

        // 위치/상태 리셋
        RespawnByTagAll();

        if (CompareTag(defenseTag))
            transform.localPosition = new Vector3(0f, defenseSpawnY, 0f);
        else
            transform.localPosition = new Vector3(
                Random.Range(boundMinX * 0.5f, boundMaxX * 0.5f),
                initPos.y,
                Random.Range(boundMinZ * 0.5f, boundMaxZ * 0.5f)
            );

        transform.localRotation = initRot;

        currentMoveX = currentMoveZ = currentYaw = 0f;
        currentMoveY = currentPitchCmd = currentRollCmd = 0f;

        targetInvader = null;

        if (controller != null) controller.ResetMovement();

        InitializeInvaders();
        ApplyCurriculum(StepCount);

        lastDistanceToTarget = -1f;

        // 이동 거리 초기화
        cumulativeDistance = 0f;
        prevPosition = transform.position;

        // Primary 초기화
        primaryRayIndex = -1;
        primaryLastHitDist = -1f;
        primaryLastSeenTime = -1f;

        // R_rayShrink 초기화
        rayLastEvalTime = Time.time;
        rayDistanceAtEval = -1f;
        rayDirAtEval = Vector3.zero;

        // 감지 유지 초기화
        perceptionStreakSec = 0f;

        // Multi-Ray 카운트 초기화
        currentInvaderRayCount = 0;

        // 스폰 코루틴 정리 후 새로 시작
        if (invaderSpawnRoutine != null)
        {
            StopCoroutine(invaderSpawnRoutine);
            invaderSpawnRoutine = null;
        }
        invaderSpawnRoutine = StartCoroutine(SpawnInvadersSequence());
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition / 50f);

        Vector3 v = controller != null ? controller.GetVelocity() : Vector3.zero;
        sensor.AddObservation(v.magnitude > 1e-3f ? v / 20f : Vector3.zero);

        if (targetInvader != null && targetInvader.activeInHierarchy)
        {
            Vector3 toTarget = targetInvader.transform.position - transform.position;
            sensor.AddObservation(toTarget / 80f);

            Vector3 invVel = Vector3.zero;
            var rbInv = targetInvader.GetComponent<Rigidbody>();
            if (rbInv) invVel = rbInv.velocity;

            Vector3 relVel = invVel - v;
            sensor.AddObservation(relVel / 20f);

            sensor.AddObservation(Mathf.Clamp01(toTarget.magnitude / 30f));
            sensor.AddObservation(1f); // target 존재 플래그
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(1f);
            sensor.AddObservation(0f); // target 없음
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        AcquireTargetIfNeeded();
        if (controller == null)
        {
            Debug.LogWarning("[DroneAgent] controller is NULL. Attach DroneController.");
            return;
        }

        var ca = actions.ContinuousActions;
        float txRaw = ca[0];
        float tyRaw = ca[1];
        float tzRaw = ca[2];
        float pRaw = ca[3];
        float yRaw = ca[4];
        float rRaw = ca[5];

        float tx = Mathf.Lerp(currentMoveX, txRaw, actionSmoothing);
        float ty = Mathf.Lerp(currentMoveY, tyRaw, actionSmoothing);
        float tz = Mathf.Lerp(currentMoveZ, tzRaw, actionSmoothing);
        float pCmd = Mathf.Lerp(currentPitchCmd, pRaw, actionSmoothing);
        float yCmd = Mathf.Lerp(currentYaw, yRaw, actionSmoothing);
        float rCmd = Mathf.Lerp(currentRollCmd, rRaw, actionSmoothing);

        currentMoveX = tx; currentMoveY = ty; currentMoveZ = tz;
        currentPitchCmd = pCmd; currentYaw = yCmd; currentRollCmd = rCmd;

        controller.autoTiltFromPlanar = false;

        Vector3 trans = new Vector3(tx, ty, tz);
        Vector3 rotDeg = new Vector3(
            pCmd * controller.maxPitchRate,
            yCmd * controller.maxYawRate,
            rCmd * controller.maxRollRate
        );
        controller.Move6DoF(trans, rotDeg);

        if (IsOutOfBounds(transform.position))
        {
            AddReward(floorCollisionPenalty);
            Academy.Instance?.StatsRecorder.Add("Episode/Success", 0f);
            Debug.Log($"[DefenseDrone] ❌ 경계 이탈 - 에피소드 종료 (Penalty {floorCollisionPenalty})");
            EndEpisodeWithMetrics("Out of bounds"); 
            return;
        }

        // 침입 성공 존 체크 (한 대씩 처리, 모든 드론이 해결되면 에피소드 종료)
        // CheckZoneFail()은 더 이상 즉시 EndEpisode을 호출하지 않고 resolvedInvaderCount만 업데이트합니다.
        CheckZoneFail(); 

        // 모든 드론이 처리되었는지 확인하고, 처리되었으면 에피소드 종료
        TryEndEpisodeIfResolved();

        ApplyStepReward();
    }

    // =============================
    // 태그별 스폰 / 위치 리셋
    // =============================
    private void RespawnByTagAll()
    {
        // Target 재배치
        foreach (var t in SafeFindByTag(targetTag))
        {
            Vector3 p = RandomPointOnCircleXZ(targetCircleRadius);
            p.y = 15f;
            t.transform.position = p;
        }

        // Defense 드론 초기화
        foreach (var d in SafeFindByTag(defenseTag))
        {
            d.transform.position = new Vector3(0f, defenseSpawnY, 0f);
            var rbD = d.GetComponent<Rigidbody>();
            if (rbD)
            {
                rbD.velocity = Vector3.zero;
                rbD.angularVelocity = Vector3.zero;
            }
        }

        // Invasion 드론: invaders 리스트 기준으로 위치/속도 리셋
        foreach (var inv in invaders)
        {
            if (!inv) continue;

            inv.SetActive(true); // 에피소드 시작할 때는 다시 켠다

            Vector3 p = SampleOutsideCircleInSquare(invasionMinRadius, invasionBoundsHalf, sampleMaxTries);
            p.y = Random.Range(spawnMinY, spawnMaxY);
            inv.transform.position = p;

            var rbI = inv.GetComponent<Rigidbody>();
            if (rbI)
            {
                rbI.velocity = Vector3.zero;
                rbI.angularVelocity = Vector3.zero;
            }
        }
    }

    private static GameObject[] SafeFindByTag(string tag)
    {
        if (string.IsNullOrEmpty(tag)) return new GameObject[0];
        try { return GameObject.FindGameObjectsWithTag(tag); }
        catch (UnityException) { return new GameObject[0]; }
    }

    private static Vector3 RandomPointOnCircleXZ(float r)
    {
        float theta = Random.Range(0f, Mathf.PI * 2f);
        return new Vector3(Mathf.Cos(theta) * r, 0f, Mathf.Sin(theta) * r);
    }

    private static Vector3 SampleOutsideCircleInSquare(float r, float b, int maxTries)
    {
        float r2 = r * r;
        for (int i = 0; i < maxTries; i++)
        {
            float x = Random.Range(-b, b);
            float z = Random.Range(-b, b);
            if (x * x + z * z >= r2) return new Vector3(x, 0f, z);
        }
        float theta = Random.Range(0f, Mathf.PI * 2f);
        return new Vector3(Mathf.Cos(theta) * r, 0f, Mathf.Sin(theta) * r);
    }

    // =============================
    // 침입 드론 초기화 & 순차 스폰
    // =============================
    private void InitializeInvaders()
    {
        foreach (var inv in invaders)
        {
            if (!inv) continue;

            var rbInv = inv.GetComponent<Rigidbody>();
            if (rbInv)
            {
                rbInv.velocity = Vector3.zero;
                rbInv.angularVelocity = Vector3.zero;
            }

            var mover = inv.GetComponent<DroneFlightController>();
            if (mover) mover.enabled = false;   // 스폰까지 정지

            inv.SetActive(false);               // 처음에는 전부 숨겨둠
        }
    }

    private IEnumerator SpawnInvadersSequence()
    {
        if (invaders == null || invaders.Count == 0)
            yield break;

        float interval = Mathf.Max(0f, invaderSpawnInterval);

        // maxInvadersPerEpisode 개까지만 사용 (보통 3)
        int countToSpawn = Mathf.Min(maxInvadersPerEpisode, invaders.Count);

        for (int i = 0; i < countToSpawn; i++)
        {
            var inv = invaders[i];
            if (!inv) continue;

            inv.SetActive(true);  // 여기서 실제로 등장

            var mover = inv.GetComponent<DroneFlightController>();
            if (mover) mover.enabled = true;

            var rbInv = inv.GetComponent<Rigidbody>();
            if (rbInv)
            {
                rbInv.velocity = Vector3.zero;
                rbInv.angularVelocity = Vector3.zero;
            }

            // 첫 드론이면 타겟으로 설정
            if (i == 0 && targetInvader == null)
            {
                targetInvader = inv;
                lastDistanceToTarget = Vector3.Distance(transform.position, targetInvader.transform.position);
            }

            if (i < countToSpawn - 1 && interval > 0f)
                yield return new WaitForSeconds(interval);
        }

        invaderSpawnRoutine = null;
    }

    // =============================
    // 커리큘럼/타깃 획득
    // =============================
    private void ApplyCurriculum(int currentStep)
    {
        foreach (var phase in curriculumPhases.OrderByDescending(p => p.startStep))
        {
            if (currentStep >= phase.startStep)
            {
                interceptDistance = phase.interceptDistance;
                foreach (var inv in invaders)
                {
                    if (!inv) continue;
                    var mover = inv.GetComponent<DroneFlightController>();
                    if (mover) mover.moveSpeed = 3f * phase.invaderSpeedMultiplier;
                }
                break;
            }
        }
    }

    private void AcquireTargetIfNeeded()
    {
        if (targetInvader == null || !targetInvader.activeInHierarchy)
        {
            foreach (var inv in invaders)
            {
                if (!inv || !inv.activeInHierarchy) continue;

                Vector3 origin = transform.position + Vector3.up * 0.5f;
                Vector3 dir = (inv.transform.position - origin).normalized;
                if (Physics.Raycast(origin, dir, out RaycastHit hit, detectRange, detectMask, QueryTriggerInteraction.Ignore))
                {
                    if (hit.collider && hit.collider.gameObject == inv)
                    {
                        targetInvader = inv;
                        lastDistanceToTarget = Vector3.Distance(transform.position, targetInvader.transform.position);
                        return;
                    }
                }
            }

            targetInvader = GetNearestInvader();
            lastDistanceToTarget = targetInvader != null ? Vector3.Distance(transform.position, targetInvader.transform.position) : -1f;
        }
        else
        {
            float d = Vector3.Distance(transform.position, targetInvader.transform.position);
            if (d > detectRange * 1.2f || !targetInvader.activeInHierarchy)
            {
                targetInvader = GetNearestInvader();
                lastDistanceToTarget = targetInvader != null ? Vector3.Distance(transform.position, targetInvader.transform.position) : -1f;
            }
        }
    }

    // =============================
    // 리워드/판정/통계
    // =============================
    private float ComputeCloseBoundaryFactor(Vector3 pos, Vector3 vel, bool seen)
    {
        float factor = 1.0f;

        float dxMin = pos.x - boundMinX;
        float dxMax = boundMaxX - pos.x;
        float dzMin = pos.z - boundMinZ;
        float dzMax = boundMaxZ - pos.z;
        float dyTop = defenseMaxY - pos.y;

        float minDist = Mathf.Min(dxMin, dxMax, dzMin, dzMax, dyTop);
        minDist = Mathf.Max(0f, minDist);

        if (closeBoundaryThreshold <= 0f) return factor;

        bool shouldAmplify = !seen;
        if (boundaryAmplifyConditional && shouldAmplify)
        {
            Vector3 outward = OutwardDirectionToNearestBoundary(pos);
            float outwardSpeed = Vector3.Dot(vel, outward);
            shouldAmplify = outwardSpeed > 0f;
        }
        if (!shouldAmplify) return factor;

        float closeness = Mathf.Clamp01(1f - (minDist / closeBoundaryThreshold));
        factor = Mathf.Max(1f, 1f + closeBoundaryLambda * closeness);
        return factor;
    }

    private Vector3 OutwardDirectionToNearestBoundary(Vector3 pos)
    {
        float distLeft = Mathf.Abs(pos.x - boundMinX);
        float distRight = Mathf.Abs(boundMaxX - pos.x);
        float distDown = Mathf.Abs(pos.z - boundMinZ);
        float distUp = Mathf.Abs(boundMaxZ - pos.z);
        float distTop = Mathf.Abs(defenseMaxY - pos.y);

        float min = distLeft; Vector3 n = Vector3.left;
        if (distRight < min) { min = distRight; n = Vector3.right; }
        if (distDown < min) { min = distDown; n = Vector3.back; }
        if (distUp < min) { min = distUp; n = Vector3.forward; }
        if (distTop < min) { min = distTop; n = Vector3.up; }
        return n;
    }

    private bool IsOutOfBounds(Vector3 pos)
    {
        if (pos.x < boundMinX || pos.x > boundMaxX) return true;
        if (pos.z < boundMinZ || pos.z > boundMaxZ) return true;
        //if (pos.y < 0f || pos.y > defenseMaxY) return true;
        return false;
    }

    private float RayWeight(float d)
    {
        if (!enableRayDistanceWeight) return 1f;
        float w = 5f * Mathf.Exp(-rayWeightAlpha * Mathf.Max(0f, d));
        return Mathf.Clamp(w, rayWeightMin, rayWeightMax);
    }

    private void ApplyStepReward()
    {
        if (targetInvader == null || !targetInvader.activeInHierarchy) return;

        float dt = Time.fixedDeltaTime;

        bool primaryHit = TryUpdateAndGetPrimaryRay(out float rayDistNow, out Vector3 rayDirNow, out int rayIdxNow);
        float wRay = primaryHit ? RayWeight(rayDistNow) : 1f;

        Vector3 toTarget = targetInvader.transform.position - transform.position;
        float dist = toTarget.magnitude;

        Vector3 vAgent = controller != null ? controller.GetVelocity() : Vector3.zero;
        Vector3 moveDir = vAgent.sqrMagnitude > 1e-6f ? vAgent.normalized : Vector3.zero;
        Vector3 rhat = toTarget.sqrMagnitude > 1e-6f ? toTarget.normalized : Vector3.zero;

        float align = 0f;
        if (moveDir != Vector3.zero && rhat != Vector3.zero)
        {
            float dot = Mathf.Max(0f, Vector3.Dot(moveDir, rhat));
            align = Mathf.Pow(dot, alignmentPower);
        }

        // SEEN 1순위: 정렬 × 근접
        if (enableAlignClose && primaryHit)
        {
            float alignBase = alignBonusCoeff * align;

            float closeBase = 0f;
            if (positiveCloseGain > 0f && positiveCloseMaxDist > 0f && dist < positiveCloseMaxDist)
            {
                float s = 1f - (dist / positiveCloseMaxDist);
                closeBase = positiveCloseGain * s;
            }

            float r_alignClose = alignBase * closeBase * wRay * dt;
            AddClippedReward(r_alignClose, -0.5f, 0.5f);
        }

        // SEEN 2순위: Sticky 보너스
        if (!enableStickyBonusOverride && enableStickyBonus && dist < Mathf.Max(stickyCapDist, interceptDistance))
        {
            float stickyWeight = primaryHit ? wRay : 1f;
            float r_sticky = stickyBonusPerStep * stickyWeight;
            AddClippedReward(r_sticky, -0.5f, 0.5f);
        }

        // CloseSpeed 및 후퇴 페널티
        if (enableCloseSpeed && primaryHit)
        {
            Vector3 invVel = Vector3.zero;
            var rbInv = targetInvader.GetComponent<Rigidbody>();
            if (rbInv) invVel = rbInv.velocity;
            Vector3 relVel = invVel - vAgent;
            float vclose = -Vector3.Dot(relVel, rhat);   // >0 접근
            float seenFactor = seenFactorWhenSeen;
            float r_v = closeSpeedGain * vclose * dt * seenFactor * wRay;
            AddClippedReward(r_v, -0.5f, 0.5f);

            if (penalizeLoiterNearTarget && dist < loiterNearDist)
            {
                if (vclose <= 0.05f)
                    AddClippedReward(-loiterPenaltyPerSec * dt, -0.25f, 0f);
            }

            if (enableRetreatPenalty && vclose < 0f)
            {
                float mag = Mathf.Pow(-vclose, retreatVcloseExponent);
                float r_retreat = -retreatVclosePenaltyGain * mag * dt * wRay;
                AddClippedReward(r_retreat, retreatClipMin, 0f);
            }

            Academy.Instance?.StatsRecorder.Add("Approach/vclose", vclose);
        }

        // 감지 유지 보상
        if (enablePerceptionHold)
        {
            if (primaryHit) perceptionStreakSec += dt; else perceptionStreakSec = 0f;
            float streak = Mathf.Min(perceptionStreakSec, perceptionHoldMaxStreak);
            float r_hold = perceptionHoldGain * (1f + perceptionHoldScale * streak) * wRay * dt;
            AddClippedReward(r_hold, -0.5f, 0.5f);
        }

        // 미감지 처리
        if (!primaryHit)
        {
            float baseMiss = -nonPerceptionPenaltyPerSec * dt;
            float boundaryFactor = ComputeCloseBoundaryFactor(transform.position, vAgent, false);
            float miss = baseMiss * boundaryFactor;
            AddClippedReward(miss);
            Academy.Instance?.StatsRecorder.Add("Miss/BoundaryFactor", boundaryFactor);

            if (enableHomeWhenUnseen)
            {
                Vector2 xz = new Vector2(transform.position.x, transform.position.z);
                float r = Vector2.Distance(xz, homeXZ);
                float proximity = 1f - Mathf.Clamp01(r / Mathf.Max(1e-3f, homeRadius));
                float r_home = homeDominantGain * proximity * dt;
                AddClippedReward(r_home, 0f, 1.0f);

                Vector2 vXZ = new Vector2(vAgent.x, vAgent.z);
                if (vXZ.sqrMagnitude > 1e-6f)
                {
                    Vector2 toHome = (homeXZ - xz).normalized;
                    Vector2 fromHome = -toHome;

                    float towardHome = Vector2.Dot(vXZ.normalized, toHome);
                    float awayHome = Vector2.Dot(vXZ.normalized, fromHome);

                    if (towardHome > 0f)
                        AddClippedReward(homeHeadingGain * towardHome * dt, 0f, 0.25f);

                    if (awayHome > 0f)
                        AddClippedReward(-homeVelAwayPenaltyGain * awayHome * dt, -0.25f, 0f);
                }
            }
        }

        // 미감지 시 틸트 활동 보상
        if (!primaryHit && enableUnseenTiltActivity && rb != null)
        {
            Vector3 omegaWorld = rb.angularVelocity;
            Vector3 omegaBody = transform.InverseTransformDirection(omegaWorld);

            float pitchRateDeg = Mathf.Abs(omegaBody.x * Mathf.Rad2Deg);
            float rollRateDeg = Mathf.Abs(omegaBody.z * Mathf.Rad2Deg);
            float rateSumDeg = pitchRateDeg + rollRateDeg;

            float target = Mathf.Max(1e-3f, unseenTiltTargetRateDeg);
            float tNorm = Mathf.Clamp(rateSumDeg / target, 0f, 2f);
            float shape = tNorm <= 1f ? tNorm : (2f - tNorm);

            float r_tilt = unseenTiltActivityGain * shape * dt;
            AddClippedReward(r_tilt, 0f, unseenTiltMaxRewardPerStep);
        }

        // RayShrink
        if (rayShrinkGain > 0f && rayEvalPeriodSec > 0f)
        {
            if (Time.time - rayLastEvalTime >= rayEvalPeriodSec)
            {
                if (primaryLastHitDist > 0f && primaryHit)
                {
                    float delta = primaryLastHitDist - rayDistNow; // +면 접근
                    if (delta > 0f)
                    {
                        float r_ray = rayShrinkGain * delta * wRay;
                        AddClippedReward(r_ray, -0.5f, 0.5f);
                        Academy.Instance?.StatsRecorder.Add("RayShrink/Delta", delta);
                        Academy.Instance?.StatsRecorder.Add("RayShrink/Reward", r_ray);
                    }
                    else if (enableRetreatPenalty && delta < 0f)
                    {
                        float p = -retreatRayExpandPenaltyGain * (-delta) * wRay;
                        AddClippedReward(p, retreatClipMin, 0f);
                        Academy.Instance?.StatsRecorder.Add("RayShrink/Delta", delta);
                    }
                }

                rayLastEvalTime = Time.time;
                rayDistanceAtEval = primaryHit ? rayDistNow : -1f;
                rayDirAtEval = primaryHit ? rayDirNow : Vector3.zero;
            }
            else
            {
                if (rayDistanceAtEval < 0f && primaryHit)
                {
                    rayDistanceAtEval = rayDistNow;
                    rayDirAtEval = rayDirNow;
                }
            }
        }

        // 현재 스텝의 레이 카운트
        currentInvaderRayCount = CountInvaderHitRays();

        // Multi-Ray 패널티
        ApplyMultiRayPenalty(dt);

        // 통계
        Academy.Instance?.StatsRecorder.Add("Intercept/Distance", dist);
        Academy.Instance?.StatsRecorder.Add("Perception/Seen", primaryHit ? 1f : 0f);
        Academy.Instance?.StatsRecorder.Add("Ray/PrimaryIndex", primaryRayIndex);
        Academy.Instance?.StatsRecorder.Add("Ray/DistNow", primaryHit ? rayDistNow : -1f);
        Academy.Instance?.StatsRecorder.Add("Ray/Weight", wRay);
        Academy.Instance?.StatsRecorder.Add("Within/Cap", dist <= Mathf.Max(stickyCapDist, interceptDistance) ? 1f : 0f);
    }

    private GameObject GetNearestInvader()
    {
        GameObject best = null; float bestDist = float.MaxValue;
        foreach (var inv in invaders)
        {
            if (!inv || !inv.activeInHierarchy) continue;
            float d = Vector3.Distance(transform.position, inv.transform.position);
            if (d < bestDist) { bestDist = d; best = inv; }
        }
        return best;
    }

    private void AddClippedReward(float r, float min = -0.25f, float max = 0.25f)
    {
        float clipped = Mathf.Clamp(r, min, max);
        AddReward(clipped);
    }

    private void FixedUpdate()
    {
        if (trackPowerConsumption)
        {
            // 1. 현재 속도 계산 (월드 좌표계)
            float currentVelocity = rb.velocity.magnitude; // m/s
            
            // 2. 전력소비 계산 (W)
            float powerConsumption = CalculatePowerConsumption(currentVelocity);
            
            // 3. 에너지 누적 (J = W × s)
            float deltaTime = Time.fixedDeltaTime; // 초
            float energyConsumed = powerConsumption * deltaTime; // Joules
            episodeTotalEnergy += energyConsumed;
            
            // 4. 이동거리 계산
            Vector3 currentPosition = transform.position;
            float distanceMoved = Vector3.Distance(lastPosition, currentPosition);
            episodeTotalDistance += distanceMoved;
            lastPosition = currentPosition;
        }
        if (rb != null)
        {
            float stepDistance = Vector3.Distance(transform.position, prevPosition);
            cumulativeDistance += stepDistance;
            prevPosition = transform.position;
        }

        if (targetInvader != null && targetInvader.activeInHierarchy)
        {
            float d = Vector3.Distance(transform.position, targetInvader.transform.position);
            Academy.Instance?.StatsRecorder.Add("Intercept/Distance", d);
            Academy.Instance?.StatsRecorder.Add("Intercept/WithinRange", d <= interceptDistance ? 1f : 0f);
        }
    }

    // =============================
    // 전력소비 계산 함수 (IEEE 논문 기반)
    // =============================
    private float CalculatePowerConsumption(float velocity)
    {
        // 회전 반지름 추정: 실제 궤적 기반이 아닌 경우 큰 값으로 가정
        float turnRadius = 100f; // m
        
        // 1. 블레이드 프로파일 전력
        float v2 = velocity * velocity;
        float Pblade = P0 * (1f + (3f * v2) / (Utip * Utip));
        
        // 2. 유도 전력
        float v4 = v2 * v2;
        float rg2 = turnRadius * turnRadius * g * g;
        float v04 = v0 * v0 * v0 * v0;
        float v02 = v0 * v0;
        
        float term1 = Mathf.Sqrt(1f + v4 / rg2);
        float term2Inner = v4 / (4f * v04) - v2 / (2f * v02);
        float term2 = term2Inner > 0 ? Mathf.Sqrt(term2Inner) : 0f;
        
        float Pinduced = Pind * term1 - Pind * term2;
        
        // 3. 기체 항력 전력
        float Pparasitic = 0.5f * d0 * rho * s * A * velocity * v2;
        
        // 총 전력 (W)
        float totalPower = Pblade + Pinduced + Pparasitic;
        
        return Mathf.Max(totalPower, 0f); // 음수 방지
    }

    // =============================
    // 충돌 처리 (요격)
    // =============================
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject == null) return;

        if (collision.gameObject.CompareTag("Floor"))
        {
            AddReward(floorCollisionPenalty);
            Academy.Instance?.StatsRecorder.Add("Episode/Success", 0f);
            Debug.Log($"[DefenseDrone] ❌ 지면 충돌 - 에피소드 종료 (Penalty {floorCollisionPenalty})");
            EndEpisodeWithMetrics("Floor collision");
            return;
        }

        string a = gameObject.tag;
        string b = collision.gameObject.tag;
        bool defenseHitInvader =
            (a == defenseTag && b == invaderTag) ||
            (a == invaderTag && b == defenseTag);

        if (defenseHitInvader)
        {
            GameObject inv =
                (a == invaderTag) ? gameObject :
                (b == invaderTag) ? collision.gameObject :
                null;

            HandleInvaderIntercept(inv);
        }
    }

    // 침입 드론이 요격된 경우 처리
    private void HandleInvaderIntercept(GameObject inv)
    {
        if (inv == null) return;
        if (!inv.activeInHierarchy) return;  // 이미 처리된 놈이면 무시

        AddReward(interceptSuccessReward);

        inv.SetActive(false);
        resolvedInvaderCount++;

        interceptedCount++;
        // Target이 요격된 드론이면 Target을 해제합니다.
        if (inv == targetInvader) targetInvader = null; 

       Debug.Log($"[DefenseDrone] ✅ 요격 성공! (+{interceptSuccessReward}) : {inv.name}  (resolved={resolvedInvaderCount}, intercepted={interceptedCount})");

        
        // 요격 성공 시에도 에피소드 종료는 TryEndEpisodeIfResolved()가 담당합니다.
        TryEndEpisodeIfResolved();
    }

    // 트리거 존(예: TargetFailZone)에 들어간 경우 처리 → 침입 성공으로 간주
    private void OnTargetFailZoneEnter(Collider other)
    {
        if (other != null && other.CompareTag(invaderTag))
        {
            HandleInvaderSuccess(other.gameObject);
        }
    }

    // ✅ 침입 성공 존 체크 (원형 XZ) - 한 대씩 처리
    private bool CheckZoneFail()
    {
        if (!useZoneFail) return false;
        
        bool anyInvaderFailedThisFrame = false;

        foreach (var inv in invaders)
        {
            if (!inv || !inv.activeInHierarchy) continue;
            Vector3 p = inv.transform.position;
            Vector2 xz = new Vector2(p.x - zoneCenterXZ.x, p.z - zoneCenterXZ.y);
            if (xz.magnitude <= zoneFailRadius)
            {
                HandleInvaderSuccess(inv);
                anyInvaderFailedThisFrame = true;
            }
        }
        
        Academy.Instance?.StatsRecorder.Add("Episode/ZoneFail", anyZoneFailThisEpisode ? 1f : 0f);
        return anyInvaderFailedThisFrame; 
    }

    // 침입 드론 1대가 "성공적으로 침입"한 경우 처리
    private void HandleInvaderSuccess(GameObject inv)
    {
        if (inv == null) return;
        if (!inv.activeInHierarchy) return;

        AddReward(invaderSuccessPenalty);
        anyZoneFailThisEpisode = true;

        Debug.Log($"[DefenseDrone] ❌ 방어 실패(존 진입 또는 FailZone): {inv.name}, Penalty {invaderSuccessPenalty}");

        inv.SetActive(false);
        resolvedInvaderCount++;
        if (inv == targetInvader) targetInvader = null;

        Academy.Instance?.StatsRecorder.Add("Episode/ZoneFail", 1f);

        TryEndEpisodeIfResolved(); 
    }

    // 3대 모두 "해결"되었으면 에피소드 종료
    private void TryEndEpisodeIfResolved()
    {
        bool anyAlive = invaders.Any(x => x != null && x.activeInHierarchy);

        if (!anyAlive || resolvedInvaderCount >= maxInvadersPerEpisode)
        {
            float successValue = anyZoneFailThisEpisode ? 0f : 1f;
            Academy.Instance?.StatsRecorder.Add("Episode/Success", successValue);
            string reason = anyZoneFailThisEpisode ? "Mission failed - Zone breach" : "All invaders intercepted";
            Debug.Log($"[DefenseDrone] ▶ 에피소드 종료: resolved={resolvedInvaderCount}, anyZoneFail={anyZoneFailThisEpisode}");
            EndEpisodeWithMetrics(reason);
        }
    }

    private void EndEpisodeWithMetrics(string reason)
    {
        // ✅ 중복 호출 방지: 한 에피소드에서 한 번만 메트릭 기록
        if (episodeMetricsSent)
            return;
        episodeMetricsSent = true;

        // 에피소드 번호 증가 (1,2,3,...)
        episodeIndex++;

        float episodeDuration = Time.time - episodeStartTime; // 초
        episodeDuration = Mathf.Max(episodeDuration, 1e-5f);

        // -------- 전력/거리 메트릭 (TensorBoard) --------
        if (trackPowerConsumption)
        {
            Academy.Instance?.StatsRecorder.Add("Defense/Episode_Total_Energy_J", episodeTotalEnergy);
            Academy.Instance?.StatsRecorder.Add("Defense/Episode_Total_Energy_kJ", episodeTotalEnergy / 1000f);
            Academy.Instance?.StatsRecorder.Add("Defense/Episode_Avg_Power_W", episodeTotalEnergy / episodeDuration);
            
            Academy.Instance?.StatsRecorder.Add("Defense/Episode_Total_Distance_m", episodeTotalDistance);
            Academy.Instance?.StatsRecorder.Add("Defense/Episode_Avg_Speed_mps", episodeTotalDistance / episodeDuration);
            
            float energyPerMeter = episodeTotalDistance > 0 ? episodeTotalEnergy / episodeTotalDistance : 0f;
            Academy.Instance?.StatsRecorder.Add("Defense/Energy_Per_Meter_J_per_m", energyPerMeter);
        }

        // -------- 요격 대수 / 요격 비율 메트릭 (TensorBoard) --------
        Academy.Instance?.StatsRecorder.Add("Defense/Episode_Intercept_Count", interceptedCount);

        float interceptRate = maxInvadersPerEpisode > 0
            ? (float)interceptedCount / maxInvadersPerEpisode
            : 0f;
        Academy.Instance?.StatsRecorder.Add("Defense/Episode_Intercept_Rate", interceptRate);

        // -------- CSV 로깅 --------
        // 성공 플래그: "All invaders intercepted" 이고 zoneFail 없을 때 1, 아니면 0
        float successFlag = (!anyZoneFailThisEpisode && reason.StartsWith("All invaders intercepted")) ? 1f : 0f;
        float zoneFailFlag = anyZoneFailThisEpisode ? 1f : 0f;

        float episodeReward = GetCumulativeReward();   

        AppendEpisodeCsv(
            episodeIndex,
            reason,
            episodeDuration,
            episodeTotalEnergy,
            episodeTotalDistance,
            interceptedCount,
            maxInvadersPerEpisode,
            successFlag,
            zoneFailFlag,
            episodeReward 
        );

        // -------- Debug 로그 --------
        Debug.Log($"[DroneAgent] Episode End - {reason}");
        Debug.Log($"  Episode: {episodeIndex}");
        Debug.Log($"  Intercepted: {interceptedCount}/{maxInvadersPerEpisode} (rate={interceptRate:F2})");
        Debug.Log($"  Energy: {episodeTotalEnergy:F2} J ({episodeTotalEnergy / 1000f:F3} kJ)");
        Debug.Log($"  Distance: {episodeTotalDistance:F2} m");
        Debug.Log($"  Duration: {episodeDuration:F2} s");
        Debug.Log($"  Avg Power: {episodeTotalEnergy / episodeDuration:F2} W");

        EndEpisode();
    }

    // =============================
    // Primary Ray: 선택/락온 + 측정
    // =============================
    private bool TryUpdateAndGetPrimaryRay(out float hitDist, out Vector3 hitDir, out int rayIdx)
    {
        hitDist = 0f; hitDir = Vector3.zero; rayIdx = -1;
        if (targetInvader == null || !targetInvader.activeInHierarchy) return false;

        Vector3 origin = transform.position + Vector3.up * 0.5f;
        Vector3 baseDir = Quaternion.AngleAxis(perceptionTiltDeg, transform.right) * transform.forward;

        int bestIdx = -1; float bestDist = float.PositiveInfinity; Vector3 bestDir = Vector3.zero;
        for (int i = 0; i < perceptionAnglesDeg.Length; i++)
        {
            float ang = perceptionAnglesDeg[i];
            Vector3 dir = Quaternion.AngleAxis(ang, Vector3.up) * baseDir;
            if (Physics.Raycast(origin, dir, out RaycastHit hit, perceptionRayLength, perceptionMask, QueryTriggerInteraction.Ignore))
            {
                if (perceptionDebugRay) Debug.DrawRay(origin, dir * hit.distance, Color.green, Time.fixedDeltaTime);
                if (hit.collider != null && hit.collider.CompareTag(invaderTag))
                {
                    if (hit.distance < bestDist)
                    {
                        bestDist = hit.distance;
                        bestIdx = i;
                        bestDir = dir;
                    }
                }
            }
            else if (perceptionDebugRay)
            {
                Debug.DrawRay(origin, dir * perceptionRayLength, Color.yellow, Time.fixedDeltaTime);
            }
        }

        bool haveBest = (bestIdx >= 0);

        if (lockOnUntilLost)
        {
            if (primaryRayIndex < 0)
            {
                if (haveBest)
                {
                    primaryRayIndex = bestIdx;
                    primaryLastHitDist = bestDist;
                    primaryLastSeenTime = Time.time;
                }
                return haveBest ? ReturnPrimary(bestDist, bestDir, bestIdx, out hitDist, out hitDir, out rayIdx) : false;
            }
            else
            {
                float ang = perceptionAnglesDeg[Mathf.Clamp(primaryRayIndex, 0, perceptionAnglesDeg.Length - 1)];
                Vector3 pdir = Quaternion.AngleAxis(ang, Vector3.up) * baseDir;
                if (Physics.Raycast(origin, pdir, out RaycastHit h, perceptionRayLength, perceptionMask, QueryTriggerInteraction.Ignore)
                    && h.collider != null && h.collider.CompareTag(invaderTag))
                {
                    primaryLastSeenTime = Time.time;
                    primaryLastHitDist = h.distance;
                    return ReturnPrimary(h.distance, pdir, primaryRayIndex, out hitDist, out hitDir, out rayIdx);
                }
                else
                {
                    if (primaryLastSeenTime > 0f && Time.time - primaryLastSeenTime <= lostReleaseDelay)
                    {
                        return false;
                    }

                    primaryRayIndex = -1;
                    primaryLastHitDist = -1f;
                    primaryLastSeenTime = -1f;

                    if (haveBest)
                    {
                        primaryRayIndex = bestIdx;
                        primaryLastHitDist = bestDist;
                        primaryLastSeenTime = Time.time;
                        return ReturnPrimary(bestDist, bestDir, bestIdx, out hitDist, out hitDir, out rayIdx);
                    }
                    return false;
                }
            }
        }
        else
        {
            if (haveBest)
            {
                primaryRayIndex = bestIdx;
                return ReturnPrimary(bestDist, bestDir, bestIdx, out hitDist, out hitDir, out rayIdx);
            }
            return false;
        }
    }

    private bool ReturnPrimary(float d, Vector3 dir, int idx, out float hitDist, out Vector3 hitDir, out int rayIdx)
    {
        hitDist = d; hitDir = dir; rayIdx = idx; return true;
    }

    // =============================
    // Multi-Ray 카운트 & 패널티
    // =============================
    private int CountInvaderHitRays()
    {
        if (perceptionAnglesDeg == null || perceptionAnglesDeg.Length == 0)
            return 0;

        Vector3 origin = transform.position + Vector3.up * 0.5f;
        Vector3 baseDir = Quaternion.AngleAxis(perceptionTiltDeg, transform.right) * transform.forward;

        int count = 0;
        for (int i = 0; i < perceptionAnglesDeg.Length; i++)
        {
            float ang = perceptionAnglesDeg[i];
            Vector3 dir = Quaternion.AngleAxis(ang, Vector3.up) * baseDir;

            if (Physics.Raycast(origin, dir, out RaycastHit hit, perceptionRayLength, perceptionMask, QueryTriggerInteraction.Ignore))
            {
                if (hit.collider != null && hit.collider.CompareTag(invaderTag))
                    count++;
            }
        }
        return count;
    }

    private void ApplyMultiRayPenalty(float dt)
    {
        if (!enableMultiRayPenalty)
            return;

        int n = currentInvaderRayCount;
        if (n < multiRayPenaltyStartCount)
            return;

        int start = Mathf.Max(1, multiRayPenaltyStartCount);
        int maxRays = (perceptionAnglesDeg != null && perceptionAnglesDeg.Length > 0)
            ? perceptionAnglesDeg.Length
            : 12;

        int effectiveExcess = Mathf.Clamp(n - (start - 1), 0, maxRays - (start - 1));
        float factor = (float)effectiveExcess / Mathf.Max(1, maxRays - (start - 1)); // 0~1

        float r = -multiRayPenaltyGain * factor * dt;
        AddClippedReward(r, -1.0f, 0f);

        Academy.Instance?.StatsRecorder.Add("MultiRay/Count", n);
        Academy.Instance?.StatsRecorder.Add("MultiRay/Factor", factor);
        Academy.Instance?.StatsRecorder.Add("MultiRay/Penalty", r);
    }

    // =============================
    // 유틸 & Perception (보조)
    // =============================
    public void SetInvaderList(IEnumerable<GameObject> invaderObjects)
    {
        invaders = invaderObjects != null ? invaderObjects.Where(o => o != null).Take(maxInvadersPerEpisode).ToList() : new List<GameObject>();
    }

    private void HookTrigger(Collider col, System.Action<Collider> handler)
    {
        if (col == null)
        {
            Debug.LogWarning("[DroneAgent] HookTrigger: Collider is null!");
            return;
        }
        if (!col.isTrigger)
            Debug.LogWarning($"[DroneAgent] HookTrigger: '{col.gameObject.name}' isTrigger=false");

        var relay = col.GetComponent<TriggerRelay>() ?? col.gameObject.AddComponent<TriggerRelay>();
        relay.OnEnter -= handler;
        relay.OnEnter += handler;
    }

    // =============================
    // Heuristic
    // =============================
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ca = actionsOut.ContinuousActions;

        float x = 0f, y = 0f, z = 0f;
        if (Input.GetKey(KeyCode.W)) z = 1f;
        if (Input.GetKey(KeyCode.S)) z = -1f;
        if (Input.GetKey(KeyCode.A)) x = -1f;
        if (Input.GetKey(KeyCode.D)) x = 1f;
        if (Input.GetKey(KeyCode.Q)) y = 1f;
        if (Input.GetKey(KeyCode.E)) y = -1f;

        float pitch = 0f, roll = 0f, yaw = 0f;
        if (Input.GetKey(KeyCode.I)) pitch = 1f;
        if (Input.GetKey(KeyCode.K)) pitch = -1f;
        if (Input.GetKey(KeyCode.J)) roll = -1f;
        if (Input.GetKey(KeyCode.L)) roll = 1f;
        if (Input.GetKey(KeyCode.U)) yaw = -1f;
        if (Input.GetKey(KeyCode.O)) yaw = 1f;

        ca[0] = x;
        ca[1] = y;
        ca[2] = z;
        ca[3] = pitch;
        ca[4] = yaw;
        ca[5] = roll;
    }
}
