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

    [SerializeField] float elevationFactor = 0.01f; // tweak in Inspector
    public float turretTurnSpeed = 2f;
    public float fireCooldown = 2f;
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

        // Base direction to target
        Vector3 toTarget = target.position - turret.position;

        // Flat distance on XZ
        float distance = new Vector2(toTarget.x, toTarget.z).magnitude;

        // Add an upward offset based on distance
        Vector3 elevatedDir = (toTarget + Vector3.up * (distance * elevationFactor)).normalized;

        // Smoothly rotate the turret to look in that direction
        Quaternion desiredRot = Quaternion.LookRotation(elevatedDir, Vector3.up);
        turret.rotation = Quaternion.Slerp(
            turret.rotation,
            desiredRot,
            Time.deltaTime * turretTurnSpeed
        );
    }

    void TryFire()
    {
        fireTimer -= Time.deltaTime;
        if (fireTimer > 0) return;

        Vector3 dirToTarget = (target.position - firePoint.position).normalized;
        float dot = Vector3.Dot(turret.forward, dirToTarget);

        if (dot > 0.95f)
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
