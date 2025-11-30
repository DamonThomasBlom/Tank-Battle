using System.Threading;
using UnityEngine;
using UnityEngine.AI;

public class TankAI : MonoBehaviour
{
    public Transform target;
    public Transform turret;
    public GameObject shellPrefab;
    public Transform firePoint;
    public float gravityScale = 0.1f;
    public float muzzleVelocity = 20f;
    public float predictionMultiplier = 1f;

    [SerializeField] float elevationFactor = 0.01f; // tweak in Inspector
    public float turretTurnSpeed = 2f;
    public float fireCooldown = 2f;
    [Range(0, 1)]
    public float accuracy = 0.9f; // 1 = perfect, 0 = random mess
    private float fireTimer;
    private NavMeshAgent agent;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
    }

    void Update()
    {
        if (target == null) return;

        // Move towards player
        agent.SetDestination(target.position);

        // Aim turret
        AimTurret();

        // Fire
        TryFire();
    }


    void AimTurret()
    {
        if (target == null) return;

        // Estimate target velocity (if it has a Rigidbody)
        Vector3 targetVel = Vector3.zero;
        if (target.TryGetComponent<Rigidbody>(out var trb))
            targetVel = trb.linearVelocity;

        // Predict position
        Vector3 toTarget = target.position - firePoint.position;
        float timeToTarget = (toTarget.magnitude / muzzleVelocity) * predictionMultiplier;
        Vector3 predictedPos = target.position + targetVel * timeToTarget;

        // Add inaccuracy
        Vector3 randomOffset = Random.insideUnitSphere * (1f - accuracy) * 10f;
        predictedPos += randomOffset;

        // Add height to compensate for bullet drop
        float distance = new Vector2(toTarget.x, toTarget.z).magnitude;
        Vector3 heightOffset = new Vector3(0, (distance * distance) * elevationFactor, 0);
        predictedPos += heightOffset;

        // Aim
        Vector3 dir = (predictedPos - turret.position).normalized;
        Quaternion lookRotation = Quaternion.LookRotation(dir);
        turret.rotation = Quaternion.Slerp(turret.rotation, lookRotation, Time.deltaTime * turretTurnSpeed);

        //if (target == null) return;

        //// Base direction to target
        //Vector3 toTarget = target.position - turret.position;

        //// Flat distance on XZ
        //float distance = new Vector2(toTarget.x, toTarget.z).magnitude;

        //// Add an upward offset based on distance
        //Vector3 elevatedDir = (toTarget + Vector3.up * (distance * elevationFactor)).normalized;

        //// Smoothly rotate the turret to look in that direction
        //Quaternion desiredRot = Quaternion.LookRotation(elevatedDir, Vector3.up);
        //turret.rotation = Quaternion.Slerp(
        //    turret.rotation,
        //    desiredRot,
        //    Time.deltaTime * turretTurnSpeed
        //);
    }

    void TryFire()
    {
        fireTimer -= Time.deltaTime;
        if (fireTimer > 0) return;

        Vector3 dirToTarget = (target.position - firePoint.position).normalized;
        float dot = Vector3.Dot(turret.forward, dirToTarget);

        if (dot > 0.75f)
        {
            // Fire straight forward from the fire point
            Quaternion shotRot = firePoint.rotation;
            GameObject shell = Instantiate(shellPrefab, firePoint.position, firePoint.rotation);

            if (shell.TryGetComponent<Rigidbody>(out var srb))
            {
                srb.linearVelocity = firePoint.forward * muzzleVelocity;
                srb.useGravity = false; // disable Unity's default gravity

                var projectile = shell.AddComponent<ProjectileGravity>();
                projectile.gravityScale = gravityScale; // custom gravity scaling
            }

            fireTimer = fireCooldown;
        }
    }
}
