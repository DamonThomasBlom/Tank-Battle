using UnityEngine;

public class TankCannon : MonoBehaviour
{
    public Transform DebugTarget;
    public bool IsLocalPlayer;

    [Header("References")]
    [SerializeField] private Transform turret;
    public Transform barrel;
    public Transform firePoint;
    public GameObject shellPrefab;
    public Rigidbody TankRigidBody;
    public Collider OwnerCollider;
    public Animator FireAnimator;
    public ParticleSystem MuzzleFlashParticle;

    [Header("Aiming")]
    public float turretTurnSpeed = 50f;
    public float barrelTurnSpeed = 50f;

    [Header("Shooting")]
    public float muzzleVelocity = 30f;
    public float gravityScale = .4f;
    public float fireCooldown = 2f;
    public float recoilImpulse = 20f;
    public float damage = 30f;

    float fireTimer;

    Transform currentTarget;
    Vector3 manualAimPoint;
    bool useManualAim;

    Team myTeam;

    void Awake()
    {
        myTeam = GetComponentInParent<Team>();
    }

    void Update()
    {
        Aim();
        fireTimer -= Time.deltaTime;

        if (DebugTarget != null)
            SetTarget(DebugTarget);
    }

    // --- PUBLIC API ---
    public void SetTarget(Transform target)
    {
        currentTarget = target;
        useManualAim = false;
    }

    public void SetAimPoint(Vector3 worldPoint)
    {
        manualAimPoint = worldPoint;
        useManualAim = true;
    }

    public bool CanFire()
    {
        return fireTimer <= 0f;
    }

    public void Fire()
    {
        if (!CanFire()) return;

        if (shellPrefab == null || firePoint == null) return;

        ShakeCameraLocalPlayer();
        FireAnimator.SetTrigger("Fire");
        MuzzleFlashParticle.Play();
        GameObject shell = PoolManager.Instance.Spawn(shellPrefab, firePoint.position, firePoint.rotation);

        var bullet = shell.GetComponent<TankBullet>();
        if (bullet != null)
        {
            bullet.ownerTeam = myTeam.teamId;
            bullet.SetOwner(OwnerCollider);
            bullet.SetDamage(damage);
        }

        if (shell.TryGetComponent<Rigidbody>(out var rb))
        {
            rb.linearVelocity = firePoint.forward * muzzleVelocity;
            rb.useGravity = false;

            var projectile = shell.GetComponent<ProjectileGravity>();
            projectile.SetGravityScale(gravityScale);
        }

        if (TankRigidBody)
            TankRigidBody.AddForceAtPosition(-firePoint.forward * recoilImpulse, firePoint.position, ForceMode.Impulse);

        fireTimer = fireCooldown;
    }

    // --- INTERNAL ---
    void Aim()
    {
        Vector3 targetPoint;

        if (useManualAim)
            targetPoint = manualAimPoint;
        else if (currentTarget != null)
            targetPoint = currentTarget.position;
        else
            return;

        // Aim direction
        Vector3 aimDir = targetPoint - firePoint.position;
        if (aimDir.sqrMagnitude < 0.0001f)
            return;
        aimDir.Normalize();

        // Pitch barrel
        Quaternion lookRot = Quaternion.LookRotation(aimDir);
        barrel.rotation = Quaternion.RotateTowards(barrel.rotation, lookRot, barrelTurnSpeed * Time.deltaTime);

        // Match turret's Y
        turret.localEulerAngles = new Vector3(0, barrel.localEulerAngles.y, 0);
    }

    void ShakeCameraLocalPlayer()
    {
        if (!IsLocalPlayer) return;

        CameraShake.Instance.Shake(.25f);
    }
}
