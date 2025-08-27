using System.Collections;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class TankController : MonoBehaviour
{
    [Header("=== Movement (Physics) ===")]
    public float engineForce = 12000f;         // forward thrust
    public float reverseForce = 9000f;         // reverse thrust
    public float turnTorque = 6000f;           // yaw torque
    public float maxSpeed = 14f;               // m/s
    public float maxTurnSpeed = 14f;               // m/s
    public float traction = 6f;                // lateral friction to kill side slip
    public float downforce = 40f;              // pushes tank to ground for grip
    public float brakeStrength = 18000f;       // when no input, resist rolling
    public float slopeAssist = 0.75f;          // keeps speed on slopes

    [Header("Grounding")]
    public Transform[] wheelPoints;            // points near treads to check ground
    public float groundCheckDistance = 1.2f;
    public LayerMask groundMask = ~0;
    public float suspensionStrength = 20000f;  // spring constant
    public float suspensionDamping = 4500f;    // counteracts bounce

    [Header("=== Turret & Gun ===")]
    public Transform turret;                   // horizontal swivel
    public Transform barrel;                   // vertical pitch (child of turret)
    public Transform firePoint;                // muzzle
    public float lookDistance = 20f;      // how far ahead of the camera to aim
    public float turretTurnSpeed = 120f;       // deg/sec
    public float barrelTurnSpeed = 80f;        // deg/sec

    [Header("=== Shooting ===")]
    public GameObject shellPrefab;             // must have Rigidbody + Collider
    public float muzzleVelocity = 120f;        // initial m/s
    public float gravityScale = 1f;
    public float reloadTime = 2.5f;
    public float recoilImpulse = 4000f;        // pushes tank back slightly
    public bool ballisticDrop = false;         // simple “point at” vs arc (on = arc calc)
    public float gravity = 9.81f;              // used for simple ballistic solve

    [Header("=== Camera (3rd Person Orbit) ===")]
    public Camera cam;
    public Transform camPivot;                 // empty object above tank (target for orbit)
    public float camDistance = 12f;
    public float camMinDistance = 4f;
    public float camMaxDistance = 22f;
    public float camSensitivity = 2.5f;
    public float camPitchMin = 5f;
    public float camPitchMax = 65f;
    public float camCollisionRadius = 0.4f;
    public float camFollowLerp = 12f;

    [Header("Input")]
    public string moveAxis = "Vertical";
    public string turnAxis = "Horizontal";
    public KeyCode fireKey = KeyCode.Mouse0;

    Rigidbody rb;
    float yaw;    // camera yaw
    float pitch;  // camera pitch
    bool canShoot = true;

    // cache
    Vector3 desiredCamPos;
    float barrelInitialLocalPitch = 0f;

    void Awake()
    {
        Cursor.lockState = CursorLockMode.Locked;

        rb = GetComponent<Rigidbody>();
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        if (!cam && Camera.main) cam = Camera.main;
        if (!camPivot)
        {
            // auto-create a pivot if not set
            GameObject piv = new GameObject("CamPivot");
            camPivot = piv.transform;
            camPivot.SetParent(transform);
            camPivot.localPosition = new Vector3(0f, 2.0f, 0f);
        }

        // initialize camera angles from current camera
        if (cam)
        {
            Vector3 toCam = cam.transform.position - camPivot.position;
            Vector3 flat = new Vector3(toCam.x, 0, toCam.z);
            yaw = Mathf.Atan2(flat.x, flat.z) * Mathf.Rad2Deg;
            pitch = Mathf.Clamp(Vector3.SignedAngle(flat, toCam, Vector3.right), camPitchMin, camPitchMax);
        }

        if (barrel) barrelInitialLocalPitch = NormalizeAngle(barrel.localEulerAngles.x);
    }

    void Update()
    {
        HandleCameraOrbit();
        AimTurretAndBarrel();
        HandleShooting();
    }

    void FixedUpdate()
    {
        ApplyGroundForces();
        HandleMovementPhysics();
        LimitMaxSpeed();
        LimitMaxTurnSpeed();
        ApplyDownforce();
        KillLateralSlip();
    }

    void LateUpdate()
    {
        PositionCamera();
    }

    float NormalizeAngle(float a)
    {
        a = Mathf.Repeat(a + 180f, 360f) - 180f;
        return a;
    }

    // === CAMERA ===

    void HandleCameraOrbit()
    {
        if (!cam) return;

        yaw += Input.GetAxis("Mouse X") * camSensitivity;
        pitch -= Input.GetAxis("Mouse Y") * camSensitivity;
        pitch = Mathf.Clamp(pitch, camPitchMin, camPitchMax);

        // Zoom wheel
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.001f)
        {
            camDistance = Mathf.Clamp(camDistance - scroll * 5f, camMinDistance, camMaxDistance);
        }
    }

    void PositionCamera()
    {
        if (!cam) return;

        Quaternion rot = Quaternion.Euler(pitch, yaw, 0f);
        Vector3 behind = rot * Vector3.back;
        Vector3 targetPos = camPivot.position;
        Vector3 desired = targetPos + behind * camDistance;

        // camera collision (sphere cast)
        if (Physics.SphereCast(targetPos, camCollisionRadius, (desired - targetPos).normalized,
                               out RaycastHit hit, Vector3.Distance(targetPos, desired), groundMask, QueryTriggerInteraction.Ignore))
        {
            desired = hit.point + hit.normal * camCollisionRadius;
        }

        // smooth move
        cam.transform.position = Vector3.Lerp(cam.transform.position, desired, Time.deltaTime * camFollowLerp);
        cam.transform.rotation = Quaternion.Slerp(cam.transform.rotation, rot, Time.deltaTime * camFollowLerp);
    }

    // === AIMING ===
    void AimTurretAndBarrel()
    {
        if (!turret || !barrel || !cam) return;

        // pick a spot straight ahead of the camera
        Vector3 targetPoint = cam.transform.position + cam.transform.forward * lookDistance;

        // --- Yaw turret (LOCAL Y relative to parent up) ---
        Transform parent = turret.parent;
        Vector3 planeUp = parent ? parent.up : Vector3.up; // use parent up so turret follows tank tilt

        Vector3 toTarget = targetPoint - turret.position;
        Vector3 flat = Vector3.ProjectOnPlane(toTarget, planeUp); // remove component along planeUp

        if (flat.sqrMagnitude > 0.0001f)
        {
            Vector3 flatDir = flat.normalized;

            // direction in parent-local space (so yaw is measured relative to parent's axes)
            Vector3 flatDirLocal = parent ? parent.InverseTransformDirection(flatDir) : flatDir;

            // desired yaw in local degrees (atan2: x = left/right, z = forward)
            float desiredLocalYaw = Mathf.Atan2(flatDirLocal.x, flatDirLocal.z) * Mathf.Rad2Deg;
            desiredLocalYaw = NormalizeAngle(desiredLocalYaw);

            // current turret local yaw
            float currentLocalYaw = NormalizeAngle(turret.localEulerAngles.y);

            // delta and clamp by turretTurnSpeed (degrees/sec)
            float deltaYaw = Mathf.DeltaAngle(currentLocalYaw, desiredLocalYaw);
            float maxStep = turretTurnSpeed * Time.deltaTime;
            float yawStep = Mathf.Clamp(deltaYaw, -maxStep, maxStep);
            float newLocalYaw = currentLocalYaw + yawStep;

            // preserve local X and Z (so turret inherits parent's pitch/roll)
            float curLocalX = NormalizeAngle(turret.localEulerAngles.x);
            float curLocalZ = NormalizeAngle(turret.localEulerAngles.z);

            turret.localRotation = Quaternion.Euler(curLocalX, newLocalYaw, curLocalZ);
        }

        // --- Pitch barrel (local rotation) ---
        Quaternion desiredRot = Quaternion.LookRotation(targetPoint - barrel.position, turret.up);

        // Smoothly rotate towards it
        barrel.rotation = Quaternion.RotateTowards(
            barrel.rotation,
            desiredRot,
            barrelTurnSpeed * Time.deltaTime
        );
    }

    // === SHOOTING ===

    void HandleShooting()
    {
        if (!canShoot || shellPrefab == null || fireKey == KeyCode.None) return;
        if (Input.GetKeyDown(fireKey))
        {
            FireShell();
        }
    }

    void FireShell()
    {
        if (!firePoint || !shellPrefab) return;

        // Fire straight forward from the fire point
        Quaternion shotRot = firePoint.rotation;
        GameObject shell = Instantiate(shellPrefab, firePoint.position, shotRot);

        if (shell.TryGetComponent<Rigidbody>(out var srb))
        {
            srb.linearVelocity = firePoint.forward * muzzleVelocity;
            srb.useGravity = false; // disable Unity's default gravity

            var projectile = shell.AddComponent<ProjectileGravity>();
            projectile.gravityScale = gravityScale; // custom gravity scaling
        }

        // Apply recoil to tank
        if (rb)
            rb.AddForceAtPosition(-firePoint.forward * recoilImpulse, firePoint.position, ForceMode.Impulse);

        StartCoroutine(ReloadRoutine());
    }

    IEnumerator ReloadRoutine()
    {
        canShoot = false;
        yield return new WaitForSeconds(reloadTime);
        canShoot = true;
    }

    // Ballistic velocity solver (flat gravity). Returns initial velocity vector.
    bool SolveBallistic(Vector3 start, Vector3 end, float speed, float g, out Vector3 velocity)
    {
        Vector3 dir = end - start;
        Vector3 dirXZ = new Vector3(dir.x, 0f, dir.z);
        float y = dir.y;
        float xz = dirXZ.magnitude;
        float v2 = speed * speed;
        float g2 = g * g;

        float under = v2 * v2 - g * (g * xz * xz + 2f * y * v2);
        if (under < 0f)
        {
            velocity = Vector3.zero;
            return false; // no solution at this speed
        }

        float root = Mathf.Sqrt(under);
        // choose lower angle (−)
        float tanTheta = (v2 - root) / (g * xz);
        float angle = Mathf.Atan(tanTheta);

        Vector3 xzDir = dirXZ.normalized;
        Vector3 v = xzDir * Mathf.Cos(angle) * speed + Vector3.up * Mathf.Sin(angle) * speed;
        velocity = v;
        return true;
        // (You could also compute the higher arc with +root if you want.)
    }

    // === MOVEMENT (PHYSICS) ===

    void HandleMovementPhysics()
    {
        float fwdInput = Input.GetAxis(moveAxis);
        float turnInput = Input.GetAxis(turnAxis);

        Vector3 fwd = transform.forward;
        Vector3 vel = rb.linearVelocity;

        // Project velocity onto forward to estimate current speed along facing
        float speedAlongForward = Vector3.Dot(vel, fwd);

        // Engine force (separate forward/reverse to tune feel)
        float thrust = fwdInput >= 0f ? engineForce : reverseForce;
        Vector3 engine = fwd * (fwdInput * thrust);

        // Slope assist: add a component along the slope that offsets gravity
        if (IsGrounded(out Vector3 groundNormal))
        {
            Vector3 gravityOnSlope = Vector3.Project(rb.mass * Physics.gravity, Vector3.Cross(Vector3.Cross(groundNormal, Vector3.down), groundNormal));
            engine += -gravityOnSlope * slopeAssist;
        }

        rb.AddForce(engine * Time.fixedDeltaTime);

        // Turning (yaw torque)
        float turn = turnInput * turnTorque;
        rb.AddTorque(Vector3.up * turn * Time.fixedDeltaTime, ForceMode.Force);

        // Passive braking when no throttle
        if (Mathf.Abs(fwdInput) < 0.05f)
        {
            Vector3 along = fwd * Vector3.Dot(vel, fwd);
            rb.AddForce(-along.normalized * Mathf.Min(along.magnitude * rb.mass, brakeStrength) * Time.fixedDeltaTime, ForceMode.Force);
        }
    }

    void LimitMaxSpeed()
    {
        Vector3 v = rb.linearVelocity;
        float horizontalSpeed = new Vector3(v.x, 0f, v.z).magnitude;
        if (horizontalSpeed > maxSpeed)
        {
            Vector3 capped = new Vector3(v.x, 0f, v.z).normalized * maxSpeed;
            rb.linearVelocity = new Vector3(capped.x, v.y, capped.z);
        }
    }

    void LimitMaxTurnSpeed()
    {
        //if (!IsGrounded(out Vector3 val)) return;

        Vector3 angVel = rb.angularVelocity; // radians/sec
        float currentTurnSpeed = angVel.magnitude;

        if (currentTurnSpeed > maxTurnSpeed)
        {
            rb.angularVelocity = angVel.normalized * maxTurnSpeed;
        }
    }

    void ApplyDownforce()
    {
        rb.AddForce(Vector3.down * downforce, ForceMode.Force);
    }

    void KillLateralSlip()
    {
        // Remove sideways velocity to feel tracked/weighty
        Vector3 right = transform.right;
        Vector3 vel = rb.linearVelocity;
        Vector3 lateral = right * Vector3.Dot(vel, right);
        Vector3 correction = -lateral * traction;
        rb.AddForce(correction, ForceMode.Acceleration);
    }

    void ApplyGroundForces()
    {
        // optional: simple stick-to-ground by raycasts to keep stable on bumps
        if (wheelPoints == null || wheelPoints.Length == 0) return;

        foreach (var p in wheelPoints)
        {
            if (!p) continue;
            if (Physics.Raycast(p.position, Vector3.down, out RaycastHit hit, groundCheckDistance, groundMask, QueryTriggerInteraction.Ignore))
            {
                float compression = 1f - (hit.distance / groundCheckDistance);
                Vector3 springForce = Vector3.up * compression * suspensionStrength;

                // Add damping (resists velocity at that point)
                Vector3 velAtPoint = rb.GetPointVelocity(p.position);
                float verticalVel = Vector3.Dot(velAtPoint, Vector3.up);
                Vector3 damperForce = -Vector3.up * verticalVel * suspensionDamping;

                rb.AddForceAtPosition((springForce + damperForce) * Time.fixedDeltaTime, p.position, ForceMode.Force);
            }
        }
    }

    bool IsGrounded(out Vector3 groundNormal)
    {
        groundNormal = Vector3.up;
        if (wheelPoints == null || wheelPoints.Length == 0) return false;

        int hits = 0;
        Vector3 normalAccum = Vector3.zero;
        foreach (var p in wheelPoints)
        {
            if (!p) continue;
            if (Physics.Raycast(p.position, Vector3.down, out RaycastHit hit, groundCheckDistance, groundMask, QueryTriggerInteraction.Ignore))
            {
                hits++;
                normalAccum += hit.normal;
            }
        }
        if (hits > 0)
        {
            groundNormal = (normalAccum / hits).normalized;
            return true;
        }
        return false;
    }

    // --- Gizmos (optional visual helpers) ---
#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        if (firePoint)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawRay(firePoint.position, firePoint.forward * 3f);
        }
        if (wheelPoints != null)
        {
            Gizmos.color = Color.green;
            foreach (var p in wheelPoints)
            {
                if (!p) continue;
                Gizmos.DrawLine(p.position, p.position + Vector3.down * groundCheckDistance);
            }
        }
    }
#endif
}
