using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Color = UnityEngine.Color;

public enum TankState
{
    Idle,
    MovingToCapture,
    Capturing,
    Attacking,
    Defending,
    Patrol
}

public class TankAI : MonoBehaviour
{
    [SerializeField] private TankState currentState;
    public NavMeshPathStatus PathStatus;
    public LayerMask TanksLayer;
    public LayerMask LOSLayer;

    [Header("Target")]
    public Transform target;
    public float enemyDetectionRadius = 100f;

    [Header("Turret & Firing")]
    public TankCannon Cannon;

    [Header("Ballistics")]
    public float predictionMultiplier = 1f;

    [Header("Aiming Settings")]
    [Range(0, 1)]
    public float accuracy = 0.9f;
    [Range(0, 1)]
    public float aimThreshold = 0.75f; // Formerly hardcoded 0.75f
    public float maxAimDistance = 100f; // Distance limit for aiming

    [Header("Firing Settings")]
    public float inaccuracyUpdateInterval = 2f; // Update random offset every 2 seconds

    [Header("Navigation")]
    [SerializeField] private NavMeshAgent agent;

    public float yOffset = 0.01f;

    // Private variables
    private Vector3 currentRandomOffset;
    private float accuracyUpdateElapsedTime;

    private CaptureZone targetZone;
    private Team myTeam;

    float evaluateStateStep = 1f;
    float randomEvaluateStep = 0;
    float elapsedTimeSinceEvaluation = 0;

    Vector3 lastSetDestination;
    float repathThreshold = 2f; // tweak this
    float targetRepathTimer;
    float targetRepathInterval = 1f;
    float zoneMoveTimer;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        myTeam = GetComponentInParent<Team>();
        var health = GetComponentInParent<Health>();
        if (health != null)
        {
            health.OnDie.AddListener(() =>
            {
                SetTarget(null);
                SetState(TankState.Idle);

                // Remove self from any capture zones we were in
                var captureZones = FindObjectsByType<CaptureZone>();
                foreach (var zone in captureZones)
                {
                    zone.tanksInZone.Remove(myTeam);
                }
                targetZone = null;
                agent.ResetPath();
            });
        }

        UpdateRandomOffset(); // Initialize random offset
    }

    void Update()
    {
        if (GameManager.Instance != null && !GameManager.Instance.CanPlayerMove())
        {
            if (currentState != TankState.Idle)
                SetState(TankState.Idle);
            return;
        }
        if (!agent.enabled) return;
        PathStatus = agent.pathStatus;
        if (agent.pathPending)
        {
            //Debug.Log("Pending Path...", gameObject);
            return;
        }

        // Update our accuracy so its random
        accuracyUpdateElapsedTime += Time.deltaTime;
        if (accuracyUpdateElapsedTime >= inaccuracyUpdateInterval)
        {
            accuracyUpdateElapsedTime = 0;
            UpdateRandomOffset();
        }

        elapsedTimeSinceEvaluation += Time.deltaTime;
        if (elapsedTimeSinceEvaluation > randomEvaluateStep)
        {
            EvaluateState();
            elapsedTimeSinceEvaluation = 0;

            // Add reaction randomness to make AI feel a bit more real
            float reactionDelay = Random.Range(0.2f, 0.6f);
            randomEvaluateStep = evaluateStateStep + reactionDelay;
        }
        HandleState();
    }

    void EvaluateState()
    {
        // ALWAYS try get closest enemy FIRST
        Transform closestEnemy = GetClosestEnemy();

        if (closestEnemy != null)
        {
            SetTarget(closestEnemy);
            SetState(TankState.Attacking);
            return;
        }

        // If we lost enemy
        SetTarget(null);

        // Capture logic
        CaptureZone bestZone = FindBestZone();

        if (bestZone != null)
        {
            targetZone = bestZone;
            SetState(TankState.MovingToCapture);
            return;
        }

        SetState(TankState.Patrol);
    }

    void HandleState()
    {
        switch (currentState)
        {
            case TankState.MovingToCapture:
                MoveToZone();
                break;

            case TankState.Capturing:
                StayInZone();
                break;

            case TankState.Attacking:
                AttackTarget();
                break;

            case TankState.Patrol:
                Patrol();
                break;

            case TankState.Idle:
                Patrol();
                break;
        }
    }

    CaptureZone FindBestZone()
    {
        CaptureZone[] zones = FindObjectsByType<CaptureZone>();

        CaptureZone best = null;
        float bestScore = float.MinValue;

        foreach (var zone in zones)
        {
            if (zone.TeamOwner == myTeam.teamId)
                continue; // Skip already owned zones

            float dist = Vector3.Distance(transform.position, zone.centerPoint.position);

            float score = 0;

            // Prefer closer zones
            score -= dist;

            // Prefer enemy zones
            if (zone.TeamOwner != myTeam.teamId)
                score += 50f;

            // Avoid overcrowded zones
            score -= zone.tanksInZone.Count * 10f;

            if (score > bestScore)
            {
                bestScore = score;
                best = zone;
            }
        }

        return best;
    }

    public float preferredDistance = 40f;

    float strafeTimer;
    Vector3 strafeDirection;

    void AttackTarget()
    {
        if (target == null)
        {
            SetState(TankState.Idle);
            return;
        }

        float dist = Vector3.Distance(transform.position, target.position);

        // Update strafe direction occasionally
        strafeTimer -= Time.deltaTime;
        if (strafeTimer <= 0f)
        {
            Vector3 toTarget = (target.position - transform.position).normalized;

            // Perpendicular = strafe
            Vector3 perp = Vector3.Cross(toTarget, Vector3.up);

            strafeDirection = (Random.value > 0.5f ? perp : -perp);
            strafeDirection += Random.insideUnitSphere * 0.3f; // noise
            strafeDirection.y = 0;

            strafeTimer = Random.Range(3, 6f);
        }

        Vector3 movePos = transform.position;

        if (dist > preferredDistance)
        {
            movePos = target.position; // move closer
        }
        else if (dist < preferredDistance * .25f) // move away
        {
            movePos = transform.position - (target.position - transform.position).normalized * 10f;
        }
        else
        {
            // 🔥 THIS is the magic → strafe instead of stopping
            movePos = transform.position + strafeDirection * 2f;
        }

        targetRepathTimer -= Time.deltaTime;

        if (targetRepathTimer <= 0f)
        {
            targetRepathTimer = Random.Range(targetRepathInterval, targetRepathInterval * 4); // randomise to make movement less predictable

            if (NavMesh.SamplePosition(movePos, out NavMeshHit hit, 10f, NavMesh.AllAreas))
            {
                SetDestinationIfNeeded(hit.position);
            }
        }

        AimTurret();
        TryFire();
    }

    Transform GetClosestEnemy()
    {
        Collider[] hits = Physics.OverlapSphere(transform.position, enemyDetectionRadius, TanksLayer);

        Transform bestTarget = null;
        float closestDist = float.MaxValue;

        foreach (var hit in hits)
        {
            Team other = hit.GetComponentInParent<Team>();

            if (other == null || other.teamId == myTeam.teamId)
                continue;

            var enemyTank = hit.gameObject;

            float dist = Vector3.Distance(transform.position, enemyTank.transform.position);

            bool hasLOS = HasLineOfSight(hit);

            // Allow close enemies even without LOS
            if (!hasLOS)
                continue;

            if (dist < closestDist)
            {
                closestDist = dist;
                bestTarget = enemyTank.transform;
            }
        }

        return bestTarget;
    }

    float moveToPointElapsedTime = 0;
    float moveToZoneInterval = 5f;

    void MoveToZone()
    {
        if (targetZone == null)
        {
            SetState(TankState.Idle);
            return;
        }

        if (targetZone.TeamOwner == myTeam.teamId)
        {
            SetState(TankState.Idle);
            return;
        }

        if (targetZone.tanksInZone.Contains(this.myTeam))
        {
            SetState(TankState.Capturing);
            return;
        }

        if (lastSetDestination != Vector3.positiveInfinity)
        {
            moveToPointElapsedTime += Time.deltaTime;
            if (moveToPointElapsedTime < moveToZoneInterval)
                return;
            moveToPointElapsedTime = 0;
        }

        Vector3 targetPos = targetZone.GetValidNavMeshPoint();

        if (NavMesh.SamplePosition(targetPos, out NavMeshHit hit, 5f, NavMesh.AllAreas))
        {
            SetDestinationIfNeeded(hit.position);
        }
    }

    void StayInZone()
    {
        if (targetZone == null)
        {
            SetState(TankState.Idle);
            return;
        }

        zoneMoveTimer -= Time.deltaTime;

        if (zoneMoveTimer <= 0f)
        {
            Vector3 targetPos = targetZone.GetValidNavMeshPoint();

            if (NavMesh.SamplePosition(targetPos, out NavMeshHit hit, 5f, NavMesh.AllAreas))
            {
                SetDestinationIfNeeded(hit.position);
            }

            zoneMoveTimer = Random.Range(2f, 5f); // change position every few seconds
        }

        // If captured → leave
        if (targetZone.TeamOwner == myTeam.teamId)
        {
            SetState(TankState.Idle);
        }
    }

    Vector3 patrolPoint;
    bool hasPatrolPoint;

    void Patrol()
    {
        if (!hasPatrolPoint)
        {
            Vector3 random = Random.insideUnitSphere * 40f;
            random.y = 0;

            patrolPoint = transform.position + random;

            if (NavMesh.SamplePosition(patrolPoint, out NavMeshHit hit, 10f, NavMesh.AllAreas))
            {
                patrolPoint = hit.position;
            }

            hasPatrolPoint = true;

            agent.ResetPath(); // 🔥 important
        }

        SetDestinationIfNeeded(patrolPoint);

        float dist = Vector3.Distance(transform.position, patrolPoint);

        if (dist < 3f)
        {
            hasPatrolPoint = false;
        }
    }

    void SetState(TankState newState)
    {
        if (currentState == newState) return;
        //Debug.Log($"Tank {name} changing state from {currentState} to {newState}");
        currentState = newState;

        agent.ResetPath();
        lastSetDestination = Vector3.positiveInfinity; 
        currentRandomOffset = Vector3.zero;
    }

    bool HasLineOfSight(Collider collider)
    {
        Vector3 dir = (collider.bounds.center - Cannon.firePoint.position).normalized;

        if (Physics.Raycast(Cannon.firePoint.position, dir, out RaycastHit hit, maxAimDistance, LOSLayer))
        {
            //Debug.DrawLine(Cannon.firePoint.position, hit.point, Color.cyan, 1f);
            if (hit.collider == collider)
            {
                return true;
            }
        }

        return false;
    }

    void SetDestinationIfNeeded(Vector3 newPos)
    {
        // Only update if far enough from last destination
        if ((newPos - lastSetDestination).sqrMagnitude < repathThreshold * repathThreshold)
            return;

        //Debug.Log($"Setting new destination for {gameObject.name} - Current State: {currentState}");
        agent.SetDestination(newPos);
        lastSetDestination = newPos;
    }

    void UpdateRandomOffset()
    {
        // Only update if we have inaccuracy
        if (accuracy < 0.99f)
        {
            float inaccuracyAmount = (1f - accuracy) * 10f;
            currentRandomOffset = Random.insideUnitSphere * inaccuracyAmount;
        }
        else
        {
            currentRandomOffset = Vector3.zero;
        }
    }

    void AimTurret()
    {
        if (target == null) return;

        Vector3 firePointPos = Cannon.firePoint.position;
        float distanceToTarget = Vector3.Distance(firePointPos, target.position);
        if (distanceToTarget > maxAimDistance) return;

        // 1. Target velocity detection
        Vector3 targetVel = Vector3.zero;
        if (target.TryGetComponent<Rigidbody>(out var trb))
            targetVel = trb.linearVelocity;

        // 2. High-precision Time of Flight prediction iteration
        float timeToTarget = distanceToTarget / Cannon.muzzleVelocity;
        Vector3 predictedPos = target.position + (targetVel * timeToTarget * predictionMultiplier) + currentRandomOffset;

        Vector3 toPredicted = predictedPos - firePointPos;
        float y = toPredicted.y;
        float x = new Vector3(toPredicted.x, 0, toPredicted.z).magnitude;

        float g = Mathf.Abs(Physics.gravity.y) * Cannon.gravityScale;
        if (g < 0.001f) g = 0.001f;

        float v = Cannon.muzzleVelocity;
        float v2 = v * v;
        float v4 = v2 * v2;

        float root = v4 - g * (g * (x * x) + 2 * y * v2);

        if (root >= 0)
        {
            // 3. Solve exact launch angle (theta)
            float tanTheta = (v2 - Mathf.Sqrt(root)) / (g * x);
            float angle = Mathf.Atan(tanTheta);

            // 4. Update prediction pass based on true flight time
            float exactTimeToTarget = x / (v * Mathf.Cos(angle));
            predictedPos = target.position + (targetVel * exactTimeToTarget * predictionMultiplier) + currentRandomOffset;

            // Recalculate geometric bounds for final accuracy pass
            toPredicted = predictedPos - firePointPos;
            y = toPredicted.y;
            x = new Vector3(toPredicted.x, 0, toPredicted.z).magnitude;

            root = v4 - g * (g * (x * x) + 2 * y * v2);
            if (root >= 0)
            {
                tanTheta = (v2 - Mathf.Sqrt(root)) / (g * x);
                angle = Mathf.Atan(tanTheta);

                // 5. VECTOR ELEVATION ALIGNMENT (The Fix)
                // Isolate horizontal facing direction
                Vector3 horizontalDir = new Vector3(toPredicted.x, 0, toPredicted.z).normalized;

                // Create a clean rotation vector elevated upwards by our exact 'angle' radians
                Vector3 fireDirection = (horizontalDir * Mathf.Cos(angle)) + (Vector3.up * Mathf.Sin(angle));

                // Project out 50+ units forward in that exact vector angle to give SetAimPoint a clear, high-altitude target
                Vector3 finalAimPoint = firePointPos + (fireDirection * Mathf.Max(50f, distanceToTarget));

                // Calculates a continuous multiplier that starts at 1.0 at 100 meters, and scales up linearly
                float multiplier = 1f + Mathf.Max(0f, (distanceToTarget - 80f) / 50f) * 0.5f;

                finalAimPoint.y += yOffset * multiplier;

                Cannon.SetAimPoint(finalAimPoint);
                return;
            }
        }

        // Out of range fallback
        Cannon.SetAimPoint(predictedPos);
    }

    void TryFire()
    {
        if (target == null) return;

        float distanceToTarget = Vector3.Distance(Cannon.firePoint.position, target.position);
        if (distanceToTarget > maxAimDistance)
        {
            return; // Too far to shoot
        }

        Vector3 dirToTarget = (target.position - Cannon.firePoint.position).normalized;
        float aimAccuracy = Vector3.Dot(Cannon.barrel.forward, dirToTarget);

        // Use the adjustable aimThreshold instead of hardcoded 0.75f
        if (aimAccuracy > aimThreshold)
        {
            Cannon.Fire();
        }
    }

    void SetTarget(Transform newTarget)
    {
        if (newTarget != null && newTarget == target) return; // No change

        //Cannon.SetTarget(newTarget);
        target = newTarget;
    }

#if UNITY_EDITOR
    // Draw editor gizmos for visualization
    void OnDrawGizmosSelected()
    {
        if (Cannon.barrel == null || Cannon.firePoint == null) return;

        // Draw turret's aim direction
        Gizmos.color = Color.red;
        Vector3 aimDirection = Cannon.barrel.forward * 10f;
        Gizmos.DrawRay(Cannon.barrel.position, aimDirection);

        // Draw turret's forward arc (based on aimThreshold)
        DrawAimThresholdGizmo();

        // Draw max aim distance sphere
        DrawMaxAimDistanceGizmo();

        // Draw current random offset if in play mode
        if (Application.isPlaying)
        {
            DrawInaccuracyGizmo();
        }
    }

    void DrawAimThresholdGizmo()
    {
        // Calculate the angle from the aimThreshold (0.75 = ~41 degrees)
        float aimAngle = Mathf.Acos(aimThreshold) * Mathf.Rad2Deg;

        // Draw a cone showing the acceptable aim area
        Gizmos.color = new Color(1f, 0.5f, 0f, 0.3f); // Orange, semi-transparent

        int segments = 20;
        float radius = Mathf.Tan(aimAngle * Mathf.Deg2Rad) * 10f;

        Vector3 forward = Cannon.barrel.forward * 10f;
        Vector3 up = Cannon.barrel.up;
        Vector3 right = Cannon.barrel.right;

        // Draw circle at the end of the cone
        Vector3 previousPoint = Cannon.barrel.position + forward + up * radius;
        for (int i = 1; i <= segments; i++)
        {
            float angle = 2f * Mathf.PI * i / segments;
            Vector3 point = Cannon.barrel.position + forward +
                           up * (radius * Mathf.Sin(angle)) +
                           right * (radius * Mathf.Cos(angle));
            Gizmos.DrawLine(Cannon.barrel.position + forward, point);
            Gizmos.DrawLine(previousPoint, point);
            previousPoint = point;
        }

        // Draw threshold angle lines
        Gizmos.color = Color.yellow;
        Vector3 leftBoundary = Quaternion.AngleAxis(-aimAngle, Cannon.barrel.up) * Cannon.barrel.forward * 10f;
        Vector3 rightBoundary = Quaternion.AngleAxis(aimAngle, Cannon.barrel.up) * Cannon.barrel.forward * 10f;
        Vector3 upBoundary = Quaternion.AngleAxis(aimAngle, -Cannon.barrel.right) * Cannon.barrel.forward * 10f;
        Vector3 downBoundary = Quaternion.AngleAxis(aimAngle, Cannon.barrel.right) * Cannon.barrel.forward * 10f;

        Gizmos.DrawRay(Cannon.barrel.position, leftBoundary);
        Gizmos.DrawRay(Cannon.barrel.position, rightBoundary);
        Gizmos.DrawRay(Cannon.barrel.position, upBoundary);
        Gizmos.DrawRay(Cannon.barrel.position, downBoundary);
    }

    void DrawMaxAimDistanceGizmo()
    {
        Gizmos.color = new Color(0f, 1f, 0f, 0.1f); // Green, very transparent
        Gizmos.DrawWireSphere(Cannon.barrel.position, maxAimDistance);

        // Draw distance to target if target exists
        if (target != null)
        {
            float distance = Vector3.Distance(Cannon.barrel.position, target.position);
            Gizmos.color = distance <= maxAimDistance ? Color.green : Color.red;
            Gizmos.DrawLine(Cannon.barrel.position, target.position);

            // Draw text showing distance (requires Handles, which we can't use in OnDrawGizmos)
            // You could use Handles.Label in OnDrawGizmos if needed
        }
    }

    void DrawInaccuracyGizmo()
    {
        if (accuracy >= 0.99f) return; // No inaccuracy to show

        // Draw the current random offset as a sphere at the predicted aim point
        if (target != null)
        {
            // Calculate predicted position (simplified for gizmo)
            Vector3 toTarget = target.position - Cannon.firePoint.position;
            float timeToTarget = (toTarget.magnitude / Cannon.muzzleVelocity) * predictionMultiplier;
            Vector3 predictedPos = target.position;

            // Add the current random offset
            Vector3 offsetPos = predictedPos + currentRandomOffset;

            Gizmos.color = new Color(1f, 0f, 1f, 0.5f); // Magenta
            Gizmos.DrawWireSphere(offsetPos, 0.5f);
            Gizmos.DrawLine(predictedPos, offsetPos);

            // Draw a sphere showing the maximum inaccuracy area
            float maxInaccuracy = (1f - accuracy) * 10f;
            Gizmos.color = new Color(1f, 0f, 1f, 0.1f);
            Gizmos.DrawWireSphere(predictedPos, maxInaccuracy);
        }
    }
#endif
}
