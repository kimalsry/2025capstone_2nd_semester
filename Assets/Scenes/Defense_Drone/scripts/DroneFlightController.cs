using System;
using UnityEngine;

namespace DefenseDrone
{
    [RequireComponent(typeof(Rigidbody))]
    [AddComponentMenu("DefenseDrone/DroneFlightController")]
    public class DroneFlightController : MonoBehaviour
    {
        [Header("[translate:물리 파라미터]")]
        public float mass = 1.2f;
        public bool useGravity = true;
        public float maxThrust = 35f;
        public Vector3 inertiaTensor = new Vector3(1f, 1f, 1f);

        [Header("[translate:목표 접근/정지]")]
        public float stopRadius = 0.0f;

        [Header("[translate:수평/수직 속도 상한]")]
        public float maxVelXY = 6.0f;
        public float maxVelZ = 3.5f;

        [Header("[translate:XY(수평) 제어 게인]")]
        public float posKpXY = 3.0f;
        public float posKdXY = 3.5f; // (옵션) 필요 시 사용
        public float accKXY = 4.0f;

        [Header("[translate:자세/요(Att) 제어 게인]")]
        public float attKp = 8.0f;
        public float attKd = 1.0f;
        public float yawKp = 5.0f;
        public float yawKd = 0.8f;

        [Header("[translate:Yaw 옵션]")]
        public bool faceMovementYaw = true; // 이동 방향(=원점 방향)을 보도록 Yaw 정렬

        [Header("[translate:Z(고도) 전용 게인]")]
        public float posKpZ = 5.0f;
        public float posKdZ = 3.0f; // (옵션) 필요 시 사용

        [Header("[translate:제한/옵션]")]
        [Range(5f, 45f)] public float maxTiltDeg = 30f;
        public bool lockYawRotation = false;

        [Header("[translate:기본 이동 속도]")]
        public float moveSpeed = 3.0f; // DroneAgent에서 참조하는 필드 유지

        [Header("[translate:속도 배율(전체 수평 응답 x배)]")]
        public float speedScale = 3f;  

        [Header("[translate:경계(클램프) 설정]")]
        public Vector3 minBounds = new Vector3(-50f, 0f, -50f);
        public Vector3 maxBounds = new Vector3(50f, 35f, 50f);
        public bool clampY = false;
        
        // ==========================================
        // 💡 수정된 필드: 회피/간격 유지 파라미터
        // 대형 드론 스케일 (17x17x17) 반영
        // ==========================================
        [Header("[translate:회피/간격 유지]")]
        public float separationDistance = 30.0f; // 충돌을 피해야 하는 최소 간격 (30m 설정)
        public float separationForce = 80.0f;   // 반발력의 강도 (50~100 사이에서 테스트 필요)
        public string invaderTag = "Invasion_Drone"; // 공격 드론 태그
        // ==========================================

        private Rigidbody rb;
        private float g => Physics.gravity.magnitude;
        private float desiredYawRad; // 이동 방향(Yaw) 목표

        private void Awake()
        {
            rb = GetComponent<Rigidbody>();
            rb.mass = mass;
            rb.useGravity = useGravity;

            if (inertiaTensor.x > 0f && inertiaTensor.y > 0f && inertiaTensor.z > 0f)
                rb.inertiaTensor = inertiaTensor;

            rb.drag = 0.6f;
            rb.angularDrag = 2.5f;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            rb.interpolation = RigidbodyInterpolation.Interpolate;

            desiredYawRad = transform.eulerAngles.y * Mathf.Deg2Rad;
        }

        private void FixedUpdate()
        {
            // 💡 변경점: 충돌 방지 로직을 먼저 적용하여 밀어냅니다.
            ApplySeparationForce();
            
            MoveToOrigin();
            KeepInsideBounds();
        }

        // ==========================================
        // 💡 추가된 메서드: 공격 드론 간 간격 유지 및 회피
        // ==========================================
        private void ApplySeparationForce()
        {
            // 이 함수는 공격 드론(Invader Drone)이 다른 공격 드론과 충돌하는 것을 방지합니다.
            
            // 모든 Invader를 찾는 것은 다소 비효율적일 수 있으나, 현재 코드 구조상 FindGameObjectsWithTag 사용
            GameObject[] allInvaders = GameObject.FindGameObjectsWithTag(invaderTag);
            Vector3 repulsiveForce = Vector3.zero;
            
            float separationDistSq = separationDistance * separationDistance;

            foreach (var invader in allInvaders)
            {
                // 자기 자신은 제외
                if (invader == gameObject) continue;
                
                Vector3 toOther = invader.transform.position - transform.position;
                float distSq = toOther.sqrMagnitude;

                // 다른 드론이 separationDistance 이내로 너무 가깝다면
                if (distSq < separationDistSq && distSq > 0.001f) // 0으로 나누는 것 방지
                {
                    float dist = Mathf.Sqrt(distSq);
                    
                    // 거리가 가까울수록 반발력이 강해지도록 계산
                    // closenessFactor는 0 (분리 거리)에서 1 (접근 거리 0) 사이의 값
                    float closenessFactor = 1f - dist / separationDistance; 
                    float strength = separationForce * closenessFactor;
                    
                    // 반발력은 상대 드론으로부터 멀어지는 방향입니다.
                    Vector3 direction = -toOther.normalized; 
                    repulsiveForce += direction * strength;
                }
            }

            if (repulsiveForce.sqrMagnitude > 0.01f)
            {
                // 💡 수정 부분: 반발력의 Y축 성분을 0으로 강제 설정
                repulsiveForce.y = 0f; 
    
                // 계산된 반발력을 Rigidbody에 힘으로 적용합니다.
                rb.AddForce(repulsiveForce, ForceMode.Acceleration);
            }
        }
        // ==========================================


        private void MoveToOrigin()
        {
            Vector3 myPos = transform.position;
            // 공격 드론의 목표는 맵 중앙 (0, Y, 0)
            Vector3 targetPos = new Vector3(0f, myPos.y, 0f); // XZ 평면 기준 (y는 유지)
            Vector3 toTgt = targetPos - myPos;

            // ===== 1) 고도(Z) 제어 =====
            float altErr = 0f; // 고도 유지(원하면 target y를 따로 줄 것)
            float thrustHover = mass * g;
            float thrustCorrection = Mathf.Clamp(posKpZ * altErr, -maxThrust, maxThrust);
            float thrustCmd = Mathf.Clamp(thrustHover + thrustCorrection, 0f, maxThrust);
            rb.AddForce(Vector3.up * thrustCmd, ForceMode.Force);

            // ===== 2) 수평(XZ) 이동 =====
            Vector3 toTgtXZ = new Vector3(toTgt.x, 0f, toTgt.z);
            float distXZ = toTgtXZ.magnitude;

            Vector3 desiredVelXZ = Vector3.zero;
            Vector3 dirXZ = Vector3.zero;
            if (distXZ > stopRadius)
            {
                dirXZ = toTgtXZ / Mathf.Max(distXZ, 1e-6f);

                // ★ 변경점: 수평 속도 상한/기본 이동속도를 speedScale로 2배 적용
                float speedCapBase = Mathf.Max(0.01f, moveSpeed) * Mathf.Max(0.01f, speedScale);
                float effectiveMaxVelXY = Mathf.Max(0.01f, maxVelXY) * Mathf.Max(0.01f, speedScale);
                float speedCap = Mathf.Min(speedCapBase, effectiveMaxVelXY);

                desiredVelXZ = dirXZ * speedCap;
            }

            // 이동 방향을 바라보도록 Yaw 목표 갱신
            if (!lockYawRotation && faceMovementYaw && dirXZ.sqrMagnitude > 1e-8f)
            {
                // ★ 변경점: 반대로 보이던 문제 수정 (원점 바라보도록)
                desiredYawRad = Mathf.Atan2(-dirXZ.x, -dirXZ.z);
            }

            Vector3 vel = rb.velocity;
            Vector3 velXZ = new Vector3(vel.x, 0f, vel.z);

            // ★ 변경점: 수평 응답 가속도도 speedScale 적용 → 2배 더 민첩
            Vector3 accXZ = (desiredVelXZ - velXZ) * (accKXY * Mathf.Max(0.01f, speedScale));
            rb.AddForce(accXZ * mass, ForceMode.Force);

            // ===== 3) 안정화 (Yaw/Pitch/Roll) =====
            StabilizeAttitude();
        }

        private void StabilizeAttitude()
        {
            float maxTiltRad = Mathf.Deg2Rad * maxTiltDeg;

            // 속도 기반 간이 수평화 (Pitch/Roll)
            Vector3 velBody = transform.InverseTransformDirection(rb.velocity);
            float desiredPitch = Mathf.Clamp(velBody.z / g, -maxTiltRad, maxTiltRad);
            float desiredRoll  = Mathf.Clamp(-velBody.x / g, -maxTiltRad, maxTiltRad);

            Vector3 euler = transform.rotation.eulerAngles;
            float rollNow  = NormalizeDeg(euler.z) * Mathf.Deg2Rad; // Z
            float pitchNow = NormalizeDeg(euler.x) * Mathf.Deg2Rad; // X
            float yawNow   = NormalizeDeg(euler.y) * Mathf.Deg2Rad; // Y

            float eRoll  = ShortestAngleRad(rollNow,  desiredRoll);
            float ePitch = ShortestAngleRad(pitchNow, desiredPitch);
            float eYaw   = ShortestAngleRad(yawNow,   desiredYawRad);

            // Unity local angular velocity: x=Pitch, y=Yaw, z=Roll
            Vector3 wLocal = transform.InverseTransformDirection(rb.angularVelocity);
            float pitchRate = wLocal.x;
            float yawRate   = wLocal.y;
            float rollRate  = wLocal.z;

            float tauRoll  = attKp * eRoll  - attKd * rollRate;   // torque Z
            float tauPitch = attKp * ePitch - attKd * pitchRate;  // torque X
            float tauYaw   = yawKp * eYaw   - yawKd * yawRate;    // torque Y

            // Apply in local body axes: (X=Pitch, Y=Yaw, Z=Roll)
            rb.AddRelativeTorque(new Vector3(tauPitch, tauYaw, tauRoll), ForceMode.Acceleration);
        }

        private void KeepInsideBounds()
        {
            Vector3 p = transform.position;
            Vector3 inward = Vector3.zero;

            if (p.x < minBounds.x) inward.x += (minBounds.x - p.x);
            if (p.x > maxBounds.x) inward.x -= (p.x - maxBounds.x);
            if (p.z < minBounds.z) inward.z += (minBounds.z - p.z);
            if (p.z > maxBounds.z) inward.z -= (p.z - maxBounds.z);

            if (clampY)
            {
                if (p.y < minBounds.y) inward.y += (minBounds.y - p.y);
                if (p.y > maxBounds.y) inward.y -= (p.y - maxBounds.y);
            }

            if (inward.sqrMagnitude > 0f)
            {
                rb.AddForce(inward * 10f, ForceMode.Acceleration);
                Vector3 v = rb.velocity;
                Vector3 outwardN = -inward.normalized;
                float vOut = Vector3.Dot(v, outwardN);
                if (vOut > 0f)
                    rb.AddForce(-outwardN * (vOut * 4f), ForceMode.Acceleration);
            }
        }

        private static float NormalizeDeg(float deg)
        {
            deg %= 360f;
            if (deg > 180f) deg -= 360f;
            if (deg < -180f) deg += 360f;
            return deg;
        }

        private static float ShortestAngleRad(float current, float target)
        {
            float diff = target - current;
            while (diff > Mathf.PI)  diff -= 2f * Mathf.PI;
            while (diff < -Mathf.PI) diff += 2f * Mathf.PI;
            return diff;
        }
    }
}