using System;
using UnityEngine;

/// <summary>
/// 6DoF 드론 컨트롤러 (Rigidbody 물리)
/// - 선형: 전/후/좌/우/상/하 (Body frame)
/// - 각:   Roll/Pitch/Yaw rate (Body frame)
/// - 속도/각속도 기반 PD + 모터 1차 지연 + (옵션) OU 노이즈 + (옵션) 바람 외란
/// - 언더댐프/요동을 줄이는 능동 브레이크 + 하강 제동 + 지면 안전추력 + 최대 하강속도 캡
/// - 이동 범위 제한: 월드 축 박스(하드 클램프) + (옵션) 소프트 벽 감쇠
/// - 스냅피(arcade) 튜닝 반영
/// - ✅ 요구 반영: 고도 7m 이상 유지, 최대 틸트 10°
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class DroneController : MonoBehaviour
{
    private Rigidbody rb;

    // =============================
    // Physics / Mass / Gravity
    // =============================
    [Header("물리/중력")]
    public float mass = 1.2f;
    public bool useGravity = true;

    // =============================
    // Linear Control (Body frame)
    // =============================
    [Header("선형 제어(Body frame 기준)")]
    [Tooltip("목표 바디 속도 상한 (x:좌우, y:상하, z:앞뒤), m/s")]
    public float maxBodyVelX = 3.8f;
    public float maxBodyVelY = 3.5f;
    public float maxBodyVelZ = 3.8f;
    [Tooltip("속도 오차→힘 변환 P 게인")]
    public float linKp = 28f;    // ↑ 민첩/급정지 강화
    [Tooltip("가속/속도 변화 감쇠(미분 성향)")]
    public float linKd = 9.0f;   // ↑ 미끄럼 억제

    // =============================
    // Attitude (Angular Rate) Control
    // =============================
    [Header("자세 제어(각속도 명령)")]
    public float maxRollRate  = 180f; // deg/s (Body z)
    public float maxPitchRate = 180f; // deg/s (Body x)
    public float maxYawRate   = 0f; // deg/s (Body y)
    [Tooltip("각속도 오차→토크 변환 P 게인")] public float angKp = 4.0f;
    [Tooltip("각속도 감쇠(미분 성향)")]      public float angKd = 0.10f;
    [Tooltip("Pitch/Roll 최대 기울기(각도)")]
    public float maxTiltDeg = 20f;    // ✅ 요구: 10°

    // =============================
    // Output Limits
    // =============================
    [Header("출력 제한")]
    public float maxForcePerAxis = 160f;   // N (Body x/y/z 별 클램프)
    public float maxTorquePerAxis = 12.0f; // N*m (Body x/y/z 별 클램프)

    // =============================
    // Extra Damping (World)
    // =============================
    [Header("감쇠/안정화(월드 감쇠)")]
    public float linearDamping  = 0.15f;  // 크게↑: 관성 체감↓
    public float angularDamping = 0.10f;

    // =============================
    // Actuator / Command Dynamics
    // =============================
    [Header("모터/액추에이터 동특성")]
    [Tooltip("명령 1차 지연 상수(초). ↓면 반응 빠름")]
    public float motorLagTau = 0.025f;
    [Tooltip("명령 Slew(추가 저역통과) 강도(0=없음,1=강함)")]
    [Range(0f,1f)] public float motorCmdSlew = 0f;

    // =============================
    // Exploration Noise (OU)
    // =============================
    [Header("탐색/노이즈 (학습 시 ON 권장)")]
    public bool useExplorationNoise = false; // 기본 OFF
    [Tooltip("속도 명령 OU 노이즈 표준편차(m/s)")]
    public float velNoiseStd = 0.8f;
    [Tooltip("각속도 명령 OU 노이즈 표준편차(deg/s)")]
    public float rateNoiseStdDeg = 25f;
    [Tooltip("OU 노이즈 시간상관(초)")]
    public float noiseTau = 0.4f;

    // =============================
    // Wind / Disturbance
    // =============================
    [Header("환경 외란(바람)")]
    public bool useWind = false; // 기본 OFF
    [Tooltip("평균 외란 강도(N) - 수평 위주")]
    public float windSigma = 4f;
    [Tooltip("외란 변화 시간상수(초)")]
    public float windTau = 0.8f;

    // =============================
    // Bounds (World Space)
    // =============================
    [Header("◆ 이동 범위 제한(World)")]
    public float boundMinX = -50f;
    public float boundMaxX =  50f;
    public float boundMinY =   10f;   // ✅ 요구: 최소 고도 10m 유지
    public float boundMaxY =  12f;
    public float boundMinZ = -50f;
    public float boundMaxZ =  50f;

    [Tooltip("경계 근처에서 입력 감쇠(소프트 벽). 0이면 미사용")]
    [Range(0f, 2f)] public float softWallWidth = 0f;   // m
    [Range(0f, 1f)] public float softWallStrength = 0.6f;

    // =============================
    // Auto Tilt (Pitch/Roll feedback)
    // =============================
    [Header("오토 틸트 (평면 이동 시 기울기)")]
    [Tooltip("평면 이동 입력에 따라 자동으로 드론이 기울어지게 함")]
    public bool autoTiltFromPlanar = true;
    [Tooltip("기울기 속도명령 크기 (deg/s)")]
    public float autoTiltRateDeg = 6.0f; // 완만하게

    // =============================
    // Active Brake & Deadzone
    // =============================
    [Header("능동 브레이크/데드존")]
    [Tooltip("입력이 거의 없을 때 속도에 반대 방향 힘을 추가")]
    public float brakeGain = 24f;       // N per (m/s)
    [Tooltip("입력 데드존 크기(-1..1)")]
    [Range(0f,0.3f)] public float inputDeadzone = 0.08f;

    // =============================
    // Vertical Safety (NEW)
    // =============================
    [Header("수직 안전/급정지 강화 (NEW)")]
    [Tooltip("하강 중 수직 속도 제동 게인 (N per (m/s))")]
    public float descentBrakeGain = 35f;   // v.y<0일 때 상향 힘
    [Tooltip("최대 하강 속도 (m/s)")]
    public float maxDescentSpeed = 2.0f;   // -2 m/s로 클램프
    [Tooltip("지면 안전 고도 (m). 이 이내면 상승 편향/추력 부스트")]
    public float groundSafeHeight = 2.5f;
    [Tooltip("지면 안전 고도 이내일 때 추가 상향 힘 (N)")]
    public float groundUpPush = 40f;

    // =============================
    // Internal States
    // =============================
    // 명령(타겟) - 비필터 원본
    private Vector3 velCmdBody = Vector3.zero;   // [m/s] Body (x: right, y: up, z: forward)
    private Vector3 rateCmdBody = Vector3.zero;  // [rad/s] Body (x: pitch, y: yaw, z: roll)

    // 명령(필터 후)
    private Vector3 velCmdBodyFilt = Vector3.zero;
    private Vector3 rateCmdBodyFilt = Vector3.zero;

    // OU 노이즈 상태
    private Vector3 ouVel = Vector3.zero;
    private Vector3 ouRate = Vector3.zero;

    // 바람 외란 상태(월드)
    private Vector3 windForce = Vector3.zero;

    // 가속 추정용
    private Vector3 prevVelBody = Vector3.zero;

    // 유틸
    private static float ClampSafe(float v, float lo, float hi) => Mathf.Clamp(v, Mathf.Min(lo, hi), Mathf.Max(lo, hi));
    private static bool IsFinite(float v) => !(float.IsNaN(v) || float.IsInfinity(v));
    private static float RandN()
    {
        // Box–Muller
        float u1 = Mathf.Max(1e-6f, UnityEngine.Random.value);
        float u2 = UnityEngine.Random.value;
        return Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
    }

    // =============================
    // Unity
    // =============================
    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.mass = mass;
        rb.useGravity = useGravity;
        rb.drag = 0f;             // 자체 감쇠는 코드로
        rb.angularDrag = 0f;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
    }

    void FixedUpdate()
    {
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-4f);

        // 바람 외란
        if (useWind)
        {
            float b = ClampSafe(dt / Mathf.Max(1e-3f, windTau), 0f, 1f);
            Vector3 wn = new Vector3(RandN(), 0.3f * RandN(), RandN()) * windSigma;
            windForce = (1f - b) * windForce + Mathf.Sqrt(2f * b) * wn;
            rb.AddForce(windForce, ForceMode.Force);
        }

        // 선형/자세 제어
        ApplyLinearControl(dt);

        // 중력 보상 (추력 모델 단순화: 항상 중력상쇄)
        rb.AddForce(-Physics.gravity * rb.mass, ForceMode.Force);

        ApplyAngularControl(dt);
        ApplyExtraDamping();

        // 최대 하강 속도 제한 (물리 적용 후 캡)
        if (rb.velocity.y < -maxDescentSpeed)
        {
            Vector3 v = rb.velocity; v.y = -maxDescentSpeed; rb.velocity = v;
        }

        // ◆ 물리 통합 이후 "마지막"에 경계 강제
        EnforceBoundsAfterPhysics();
    }

    // =============================
    // Public API
    // =============================

    /// <summary>
    /// [-1,1] 명령을 받아 속도/각속도 목표로 변환 (Body frame)
    /// cmdTrans: (x=좌우, y=상하, z=앞뒤) in [-1,1]
    /// cmdRotDegPerSec: (roll(X), yaw(Y), pitch(Z)) in deg/s
    /// </summary>
    public void Move6DoF(Vector3 cmdTrans, Vector3 cmdRotDegPerSec)
    {
        // ===== 입력 데드존 처리 =====
        if (Mathf.Abs(cmdTrans.x) < inputDeadzone) cmdTrans.x = 0f;
        if (Mathf.Abs(cmdTrans.y) < inputDeadzone) cmdTrans.y = 0f;
        if (Mathf.Abs(cmdTrans.z) < inputDeadzone) cmdTrans.z = 0f;

        // cmdTrans = (x:좌우, y:상하, z:앞뒤)
        // cmdRotDegPerSec = (roll(X), yaw(Y), pitch(Z))
        Vector3 velCmd = new Vector3(
            cmdTrans.x * maxBodyVelX,
            cmdTrans.y * maxBodyVelY,
            cmdTrans.z * maxBodyVelZ
        );

        Vector3 rateCmd = new Vector3(
            ClampSafe(cmdRotDegPerSec.x, -maxPitchRate, maxPitchRate),  // Pitch
            ClampSafe(cmdRotDegPerSec.y, -maxYawRate,   maxYawRate),    // Yaw
            ClampSafe(cmdRotDegPerSec.z, -maxRollRate,  maxRollRate)    // Roll
        ) * Mathf.Deg2Rad;

        // 1차 지연 + Slew 필터
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-4f);
        float a = ClampSafe(dt / Mathf.Max(1e-3f, motorLagTau), 0f, 1f);
        float s = motorCmdSlew;

        velCmdBodyFilt  = Vector3.Lerp(velCmdBodyFilt,  velCmd,  a);
        rateCmdBodyFilt = Vector3.Lerp(rateCmdBodyFilt, rateCmd, a);
        if (s > 0f)
        {
            velCmdBodyFilt  = Vector3.Lerp(velCmdBodyFilt,  velCmd,  s);
            rateCmdBodyFilt = Vector3.Lerp(rateCmdBodyFilt, rateCmd, s);
        }

        // OU 노이즈
        if (useExplorationNoise)
        {
            float b = ClampSafe(dt / Mathf.Max(1e-3f, noiseTau), 0f, 1f);
            ouVel  = (1f - b) * ouVel  + Mathf.Sqrt(2f * b) * new Vector3(RandN(), RandN(), RandN()) * velNoiseStd;
            ouRate = (1f - b) * ouRate + Mathf.Sqrt(2f * b) * new Vector3(RandN(), RandN(), RandN()) * (rateNoiseStdDeg * Mathf.Deg2Rad);

            velCmdBody  = velCmdBodyFilt  + ouVel;
            rateCmdBody = rateCmdBodyFilt + ouRate;
        }
        else
        {
            velCmdBody  = velCmdBodyFilt;
            rateCmdBody = rateCmdBodyFilt;
        }
    }

    /// <summary>
    /// 평면 이동 + Yaw 제어(기존 호환)
    /// direction=(x:좌우, y:앞뒤), rotation=yaw[-1,1]
    /// </summary>
    public void Move(Vector2 direction, float yawInput)
    {
        // 평면 이동 명령 (-1..1)
        Vector3 trans = new Vector3(direction.x, 0f, direction.y);

        float pitchCmdDeg = 0f; // X
        float yawCmdDeg   = Mathf.Clamp(yawInput, -1f, 1f) * maxYawRate; // Y
        float rollCmdDeg  = 0f; // Z

        if (autoTiltFromPlanar)
        {
            // 전진(+z)하려면 Pitch(-)로 숙여지고, 우측(+x) 가려면 Roll(+)로 기울게
            pitchCmdDeg = -Mathf.Clamp(trans.z, -1f, 1f) * autoTiltRateDeg; // X=Pitch
            rollCmdDeg  =  Mathf.Clamp(trans.x, -1f, 1f) * autoTiltRateDeg; // Z=Roll
        }

        // X=Pitch, Y=Yaw, Z=Roll 로 전달
        Vector3 rotDeg = new Vector3(pitchCmdDeg, yawCmdDeg, rollCmdDeg);
        Move6DoF(trans, rotDeg);
    }

    public Vector3 GetVelocity() => rb.velocity;
    public Vector3 GetVelocityWorld() => rb.velocity;
    public Vector3 GetAngularVelocityBody() => transform.InverseTransformDirection(rb.angularVelocity); // rad/s

    public void ResetMovement()
    {
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        // 필터/노이즈 상태 초기화
        velCmdBody = velCmdBodyFilt = Vector3.zero;
        rateCmdBody = rateCmdBodyFilt = Vector3.zero;
        ouVel = ouRate = Vector3.zero;
        windForce = Vector3.zero;
        prevVelBody = Vector3.zero;
    }

    // =============================
    // Internal Control
    // =============================
    private void ApplyLinearControl(float dt)
    {
        // 월드속도 -> 바디속도
        Vector3 velBody = WorldToBody(rb.velocity);

        // 속도 오차
        Vector3 vErr = velCmdBody - velBody;

        // 간단한 바디 가속 추정(차분)
        Vector3 aBody = BodyAccelApprox(velBody, dt);

        // PD → 바디힘
        Vector3 forceBody = linKp * vErr - linKd * aBody;

        // ===== 능동 브레이크: 입력이 거의 없을 때 속도 반대 방향 힘 추가 =====
        bool nearZeroCmd =
            Mathf.Approximately(velCmdBody.x, 0f) &&
            Mathf.Approximately(velCmdBody.y, 0f) &&
            Mathf.Approximately(velCmdBody.z, 0f);

        if (nearZeroCmd && brakeGain > 0f)
        {
            // 현재 바디 속도에 반대 방향 힘 = -k * v
            forceBody += -brakeGain * velBody;
        }

        // ===== 하강 제동(수직 속도 제동) =====
        Vector3 velWorld = rb.velocity;
        if (velWorld.y < 0f && descentBrakeGain > 0f)
        {
            float upBrake = -velWorld.y * descentBrakeGain; // (-)*gain → +
            float upBrakeBody = transform.InverseTransformDirection(Vector3.up * upBrake).y;
            forceBody.y += upBrakeBody;
        }

        // ===== 지면 안전추력(소프트 벽) =====
        float gDist = GroundDistance(rb.position, 50f);
        if (gDist >= 0f && gDist < groundSafeHeight)
        {
            float t = 1f - Mathf.Clamp01(gDist / Mathf.Max(0.1f, groundSafeHeight)); // 0~1
            float push = groundUpPush * t; // 가까울수록 큰 상향 힘
            float pushBody = transform.InverseTransformDirection(Vector3.up * push).y;
            forceBody.y += pushBody;

            // 지면 근처에서 하강 명령 완화(상향 편향)
            velCmdBody.y = Mathf.Max(velCmdBody.y, 0.5f * (1f - gDist / groundSafeHeight) * maxBodyVelY);
        }

        // 출력 클램프
        forceBody.x = ClampSafe(forceBody.x, -maxForcePerAxis, maxForcePerAxis);
        forceBody.y = ClampSafe(forceBody.y, -maxForcePerAxis, maxForcePerAxis);
        forceBody.z = ClampSafe(forceBody.z, -maxForcePerAxis, maxForcePerAxis);

        // 바디힘 → 월드힘
        Vector3 forceWorld = BodyToWorld(forceBody);
        rb.AddForce(forceWorld, ForceMode.Force);
    }

    private void ApplyAngularControl(float dt)
    {
        // 현재 회전 각도 읽기
        Vector3 euler = transform.rotation.eulerAngles;
        float rollDeg  = (euler.z > 180f) ? euler.z - 360f : euler.z;
        float pitchDeg = (euler.x > 180f) ? euler.x - 360f : euler.x;

        // Pitch/Roll 각도 제한 (안전)
        rollDeg  = Mathf.Clamp(rollDeg,  -maxTiltDeg, maxTiltDeg);
        pitchDeg = Mathf.Clamp(pitchDeg, -maxTiltDeg, maxTiltDeg);
        transform.rotation = Quaternion.Euler(pitchDeg, euler.y, rollDeg);

        // 월드 각속도를 바디축으로 변환
        Vector3 wBody = transform.InverseTransformDirection(rb.angularVelocity);

        // rate error (바디축 기준)
        Vector3 wErr = rateCmdBody - wBody;

        // PD
        Vector3 torqueBody = angKp * wErr - angKd * wBody;

        // 출력 제한
        torqueBody.x = ClampSafe(torqueBody.x, -maxTorquePerAxis, maxTorquePerAxis);
        torqueBody.y = ClampSafe(torqueBody.y, -maxTorquePerAxis, maxTorquePerAxis);
        torqueBody.z = ClampSafe(torqueBody.z, -maxTorquePerAxis, maxTorquePerAxis);

        // 각가속도 기반(질량/관성 독립) 토크 적용
        rb.AddRelativeTorque(torqueBody, ForceMode.Acceleration);
    }

    private void ApplyExtraDamping()
    {
        // 월드 감쇠(약하게) - 언더댐프 유지
        rb.AddForce(-linearDamping * rb.velocity, ForceMode.Force);
        rb.AddTorque(-angularDamping * rb.angularVelocity, ForceMode.Force);
    }

    private Vector3 BodyAccelApprox(Vector3 velBodyNow, float dt)
    {
        Vector3 a = (velBodyNow - prevVelBody) / Mathf.Max(1e-4f, dt);
        prevVelBody = velBodyNow;
        // NaN/Inf 가드
        if (!IsFinite(a.x) || !IsFinite(a.y) || !IsFinite(a.z)) return Vector3.zero;
        return a;
    }

    private Vector3 WorldToBody(Vector3 v) => transform.InverseTransformDirection(v);
    private Vector3 BodyToWorld(Vector3 v)  => transform.TransformDirection(v);

    // =============================
    // Bounds Enforcement
    // =============================
    /// <summary>
    /// 물리 통합 이후 "마지막"에 호출. 박스 내부로 하드 클램프 + (옵션) 소프트 벽 감쇠 + 바깥 방향 속도 제거
    /// </summary>
    private void EnforceBoundsAfterPhysics()
    {
        Vector3 pos = rb.position;

        // (옵션) 소프트 벽: 경계 근처에서 속도 명령 감쇠 → 자연스럽게 둔화
        if (softWallWidth > 0f)
        {
            float k = softWallStrength;

            float fx = 1f - Mathf.Clamp01(Mathf.Min(
                Mathf.Abs(pos.x - boundMinX),
                Mathf.Abs(pos.x - boundMaxX)) / softWallWidth) * k;

            float fy = 1f - Mathf.Clamp01(Mathf.Min(
                Mathf.Abs(pos.y - boundMinY),
                Mathf.Abs(pos.y - boundMaxY)) / softWallWidth) * k;

            float fz = 1f - Mathf.Clamp01(Mathf.Min(
                Mathf.Abs(pos.z - boundMinZ),
                Mathf.Abs(pos.z - boundMaxZ)) / softWallWidth) * k;

            velCmdBodyFilt = new Vector3(
                velCmdBodyFilt.x * fx,
                velCmdBodyFilt.y * fy,
                velCmdBodyFilt.z * fz
            );
        }

        // 하드 클램프(무조건 박스 안)
        float clampedX = Mathf.Clamp(pos.x, boundMinX, boundMaxX);
        float clampedY = Mathf.Clamp(pos.y, boundMinY, boundMaxY); // ✅ 7m 하한 적용
        float clampedZ = Mathf.Clamp(pos.z, boundMinZ, boundMaxZ);

        bool hit = (pos.x != clampedX) || (pos.y != clampedY) || (pos.z != clampedZ);

        if (hit)
        {
            // 위치 보정
            pos.x = clampedX; pos.y = clampedY; pos.z = clampedZ;
            rb.position = pos;

            // 바깥으로 나가려는 속도 성분 제거(떨림/관통 방지)
            Vector3 v = rb.velocity;
            if (Mathf.Approximately(pos.x, boundMinX) && v.x < 0f) v.x = 0f;
            if (Mathf.Approximately(pos.x, boundMaxX) && v.x > 0f) v.x = 0f;
            if (Mathf.Approximately(pos.y, boundMinY) && v.y < 0f) v.y = 0f;   // ✅ 하한면에서 하강속도 제거
            if (Mathf.Approximately(pos.y, boundMaxY) && v.y > 0f) v.y = 0f;
            if (Mathf.Approximately(pos.z, boundMinZ) && v.z < 0f) v.z = 0f;
            if (Mathf.Approximately(pos.z, boundMaxZ) && v.z > 0f) v.z = 0f;
            rb.velocity = v;
        }
    }

    // =============================
    // Ground Ray Utility
    // =============================
    private float GroundDistance(Vector3 pos, float maxRay = 50f, int mask = ~0)
    {
        if (Physics.Raycast(pos, Vector3.down, out var hit, maxRay, mask, QueryTriggerInteraction.Ignore))
            return hit.distance;
        return -1f;
    }
}
