using NUnit.Framework.Internal;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.AI;

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
    [Header("Target")]
    public Transform target;
    public float enemyDetectionRadius = 100f;

    [Header("Turret & Firing")]
    public Transform turret;
    public Transform firePoint;
    public GameObject shellPrefab;

    [Header("Ballistics")]
    public float muzzleVelocity = 20f;
    public float gravityScale = 0.1f;
    [SerializeField] float elevationFactor = 0.01f;
    public float predictionMultiplier = 1f;

    [Header("Aiming Settings")]
    [Range(0, 1)]
    public float accuracy = 0.9f;
    [Range(0, 1)]
    public float aimThreshold = 0.75f; // Formerly hardcoded 0.75f
    public float maxAimDistance = 100f; // Distance limit for aiming
    public float turretTurnSpeed = 2f;

    [Header("Firing Settings")]
    public float fireCooldown = 2f;
    public float inaccuracyUpdateInterval = 2f; // Update random offset every 2 seconds

    [Header("Navigation")]
    [SerializeField] private NavMeshAgent agent;

    // Private variables
    private float fireTimer;
    private Vector3 currentRandomOffset;
    private float accuracyUpdateElapsedTime;
    private bool canSeeTarget = true; // Will be used for future LOS implementation
    private Vector3 targetZoneRandomOffset;

    [SerializeField] private TankState currentState;
    private CaptureZone targetZone;
    private Team myTeam;

    float evaluateStateStep = 1f;
    float randomEvaluateStep = 0;
    float elapsedTimeSinceEvaluation = 0;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        myTeam = GetComponentInParent<Team>();
        var health = GetComponentInParent<Health>();
        if (health != null)
        {
            health.OnDie.AddListener(() =>
            {
                SetState(TankState.Idle);
                target = null;

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
        if (!agent.enabled) return;

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
            target = closestEnemy;
            SetState(TankState.Attacking);
            return;
        }

        // If we lost enemy
        target = null;

        // Capture logic
        CaptureZone bestZone = FindBestZone();

        if (bestZone != null)
        {
            targetZone = bestZone;
            targetZoneRandomOffset = Random.insideUnitSphere * 10f;
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
            if (zone.teamOwner == myTeam.teamId)
                continue; // Skip already owned zones

            float dist = Vector3.Distance(transform.position, zone.centerPoint.position);

            float score = 0;

            // Prefer closer zones
            score -= dist;

            // Prefer enemy zones
            if (zone.teamOwner != myTeam.teamId)
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

    float preferredDistance = 40f;

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
            movePos = target.position;
        }
        else if (dist < preferredDistance * 0.5f)
        {
            movePos = transform.position - (target.position - transform.position).normalized * 10f;
        }
        else
        {
            // 🔥 THIS is the magic → strafe instead of stopping
            movePos = transform.position + strafeDirection * 2f;
        }

        if (NavMesh.SamplePosition(movePos, out NavMeshHit hit, 10f, NavMesh.AllAreas))
        {
            agent.SetDestination(hit.position);
        }

        AimTurret();
        TryFire();
    }

    bool IsEnemyNearby()
    {
        float detectionRadius = 60f;

        Collider[] hits = Physics.OverlapSphere(transform.position, detectionRadius);

        foreach (var hit in hits)
        {
            Team other = hit.GetComponentInParent<Team>();

            if (other != null && other != this && other.teamId != myTeam.teamId)
            {
                bool hasLOS = HasLineOfSight(other.transform);

                if (hasLOS || Vector3.Distance(transform.position, other.transform.position) < 10f)
                {
                    target = hit.transform;
                    return true;
                }
            }
        }

        target = null;
        return false;
    }

    Transform GetClosestEnemy()
    {
        Collider[] hits = Physics.OverlapSphere(transform.position, enemyDetectionRadius);

        Transform bestTarget = null;
        float closestDist = float.MaxValue;

        foreach (var hit in hits)
        {
            Team other = hit.GetComponentInParent<Team>();

            if (other == null || other.teamId == myTeam.teamId)
                continue;

            var enemyTank = hit.gameObject;

            float dist = Vector3.Distance(transform.position, enemyTank.transform.position);

            bool hasLOS = HasLineOfSight(enemyTank.transform);

            // Allow close enemies even without LOS
            if (!hasLOS && dist > 15f)
                continue;

            if (dist < closestDist)
            {
                closestDist = dist;
                bestTarget = enemyTank.transform;
            }
        }

        return bestTarget;
    }

    void MoveToZone()
    {
        if (targetZone == null)
        {
            SetState(TankState.Idle);
            return;
        }

        Vector3 targetPos = targetZone.centerPoint.position + targetZoneRandomOffset;

        if (NavMesh.SamplePosition(targetPos, out NavMeshHit hit, 5f, NavMesh.AllAreas))
        {
            agent.SetDestination(hit.position);
        }

        //float dist = Vector3.Distance(transform.position, targetZone.centerPoint.position);

        if (targetZone.tanksInZone.Contains(this.myTeam))
        {
            SetState(TankState.Capturing);
        }

        if (targetZone.teamOwner == myTeam.teamId)
        {
            SetState(TankState.Idle);
        }
    }

    Vector3 zoneMovePoint;
    float zoneMoveTimer;

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
            Vector3 randomOffset = Random.insideUnitSphere * 10f;
            randomOffset.y = 0;

            Vector3 targetPos = targetZone.centerPoint.position + randomOffset;

            if (NavMesh.SamplePosition(targetPos, out NavMeshHit hit, 10f, NavMesh.AllAreas))
            {
                zoneMovePoint = hit.position;
                agent.SetDestination(zoneMovePoint);
            }

            zoneMoveTimer = Random.Range(2f, 5f); // change position every few seconds
        }

        // If captured → leave
        if (targetZone.teamOwner == myTeam.teamId)
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

        agent.SetDestination(patrolPoint);

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
    }

    bool HasLineOfSight(Transform target)
    {
        Vector3 dir = (target.position - firePoint.position).normalized;

        if (Physics.Raycast(firePoint.position, dir, out RaycastHit hit, maxAimDistance))
        {
            return hit.transform.root == target.root;
        }

        return false;
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

        // Check distance limit
        float distanceToTarget = Vector3.Distance(firePoint.position, target.position);
        if (distanceToTarget > maxAimDistance)
        {
            // Target is too far to aim at properly
            return;
        }

        // Estimate target velocity (if it has a Rigidbody)
        Vector3 targetVel = Vector3.zero;
        if (target.TryGetComponent<Rigidbody>(out var trb))
            targetVel = trb.linearVelocity;

        // Predict position
        Vector3 toTarget = target.position - firePoint.position;
        float timeToTarget = (toTarget.magnitude / muzzleVelocity) * predictionMultiplier;
        Vector3 predictedPos = target.position + targetVel * timeToTarget;

        // Add persistent random offset (updated every 2 seconds)
        predictedPos += currentRandomOffset;

        // Add height to compensate for bullet drop
        float horizontalDistance = new Vector2(toTarget.x, toTarget.z).magnitude;
        Vector3 heightOffset = new Vector3(0, (horizontalDistance * horizontalDistance) * elevationFactor, 0);
        predictedPos += heightOffset;

        // Aim
        Vector3 dir = (predictedPos - turret.position).normalized;
        Quaternion lookRotation = Quaternion.LookRotation(dir);
        turret.rotation = Quaternion.Slerp(turret.rotation, lookRotation, Time.deltaTime * turretTurnSpeed);
    }

    void TryFire()
    {
        fireTimer -= Time.deltaTime;
        if (fireTimer > 0) return;

        // Check distance limit for firing
        if (target == null) return;
        float distanceToTarget = Vector3.Distance(firePoint.position, target.position);
        if (distanceToTarget > maxAimDistance)
        {
            return; // Too far to shoot
        }

        Vector3 dirToTarget = (target.position - firePoint.position).normalized;
        float aimAccuracy = Vector3.Dot(turret.forward, dirToTarget);

        // Use the adjustable aimThreshold instead of hardcoded 0.75f
        if (aimAccuracy > aimThreshold)
        {
            FireShell();
            fireTimer = fireCooldown;
        }
    }

    void FireShell()
    {
        if (shellPrefab == null || firePoint == null) return;

        Quaternion shotRot = firePoint.rotation;
        GameObject shell = Instantiate(shellPrefab, firePoint.position, firePoint.rotation);

        var bullet = shell.GetComponent<TankBullet>();
        if (bullet != null)
        {
            bullet.ownerTeam = myTeam.teamId;
            bullet.SetOwner(gameObject);
        }

        if (shell.TryGetComponent<Rigidbody>(out var srb))
        {
            srb.linearVelocity = firePoint.forward * muzzleVelocity;
            srb.useGravity = false;

            var projectile = shell.AddComponent<ProjectileGravity>();
            projectile.gravityScale = gravityScale;
        }
    }

#if UNITY_EDITOR
    // Draw editor gizmos for visualization
    void OnDrawGizmosSelected()
    {
        if (turret == null || firePoint == null) return;

        // Draw turret's aim direction
        Gizmos.color = Color.red;
        Vector3 aimDirection = turret.forward * 10f;
        Gizmos.DrawRay(turret.position, aimDirection);

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

        Vector3 forward = turret.forward * 10f;
        Vector3 up = turret.up;
        Vector3 right = turret.right;

        // Draw circle at the end of the cone
        Vector3 previousPoint = turret.position + forward + up * radius;
        for (int i = 1; i <= segments; i++)
        {
            float angle = 2f * Mathf.PI * i / segments;
            Vector3 point = turret.position + forward +
                           up * (radius * Mathf.Sin(angle)) +
                           right * (radius * Mathf.Cos(angle));
            Gizmos.DrawLine(turret.position + forward, point);
            Gizmos.DrawLine(previousPoint, point);
            previousPoint = point;
        }

        // Draw threshold angle lines
        Gizmos.color = Color.yellow;
        Vector3 leftBoundary = Quaternion.AngleAxis(-aimAngle, turret.up) * turret.forward * 10f;
        Vector3 rightBoundary = Quaternion.AngleAxis(aimAngle, turret.up) * turret.forward * 10f;
        Vector3 upBoundary = Quaternion.AngleAxis(aimAngle, -turret.right) * turret.forward * 10f;
        Vector3 downBoundary = Quaternion.AngleAxis(aimAngle, turret.right) * turret.forward * 10f;

        Gizmos.DrawRay(turret.position, leftBoundary);
        Gizmos.DrawRay(turret.position, rightBoundary);
        Gizmos.DrawRay(turret.position, upBoundary);
        Gizmos.DrawRay(turret.position, downBoundary);
    }

    void DrawMaxAimDistanceGizmo()
    {
        Gizmos.color = new Color(0f, 1f, 0f, 0.1f); // Green, very transparent
        Gizmos.DrawWireSphere(turret.position, maxAimDistance);

        // Draw distance to target if target exists
        if (target != null)
        {
            float distance = Vector3.Distance(turret.position, target.position);
            Gizmos.color = distance <= maxAimDistance ? Color.green : Color.red;
            Gizmos.DrawLine(turret.position, target.position);

            // Draw text showing distance (requires Handles, which we can't use in OnDrawGizmos)
            // You could use Handles.Label in OnDrawGizmos if needed
        }
    }

    void DrawInaccuracyGizmo()
    {
        if (accuracy >= 0.99f) return; // No inaccuracy to show

        // Draw the current random offset as a sphere at the predicted aim point
        if (target != null && canSeeTarget)
        {
            // Calculate predicted position (simplified for gizmo)
            Vector3 toTarget = target.position - firePoint.position;
            float timeToTarget = (toTarget.magnitude / muzzleVelocity) * predictionMultiplier;
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
