using UnityEngine;

public class TankBullet : MonoBehaviour
{
    [Header("Explosion")]
    public GameObject ParticleEffect;   // Prefab with particles + optional shockwave
    public GameObject TrailParticleEffect;   // Prefab with particles
    public Transform bulletGraphic;  // assign your bullet mesh here in Inspector

    [Header("Damage")]
    public float damage = 25f;
    public int ownerTeam;

    [Header("Settings")]
    public float destroyDelay = 0.05f;
    public float explosionCameraShakeRadius = 10f;
    public float maxShakeIntensity = 1f;
    public float bulletLifetime = 7f;

    public TrailRenderer Trail;

    public float damageMultiplier = 1f;

    private Rigidbody rb;
    private Collider ownerCollider;
    bool hasExploded = false;
    private TankStats OwnerStats;
    private GameObject _currentTrail;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void OnEnable()
    {
        hasExploded = false;
        if (rb == null)
            rb = GetComponent<Rigidbody>();

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        PoolManager.Instance.DespawnAfterDelay(gameObject, bulletLifetime);
        Trail.Clear();

        SpawnTrail();
    }

    void Update()
    {
        // Make the bullet graphic face the velocity direction
        if (bulletGraphic != null && rb != null && rb.linearVelocity.sqrMagnitude > 0.01f)
        {
            bulletGraphic.forward = rb.linearVelocity.normalized;
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (hasExploded) return;
        if (other == ownerCollider) return;

        hasExploded = true;
        Explode();
        DespawnTrail();

        // Spawn particle effect
        if (ParticleEffect != null)
        {
            GameObject effect = PoolManager.Instance.Spawn(ParticleEffect, transform.position, Quaternion.identity);
            ParticleSystem ps = effect.GetComponentInChildren<ParticleSystem>();
            if (ps != null)
            {
                PoolManager.Instance.DespawnAfterDelay(effect, ps.main.duration + ps.main.startLifetime.constantMax);
            }
            else
            {
                PoolManager.Instance.DespawnAfterDelay(effect, 3f);
            }
        }

        // Camera shake
        if (CameraShake.Instance != null)
        {
            float dist = Vector3.Distance(transform.position, CameraShake.Instance.transform.position);
            if (dist < explosionCameraShakeRadius)
            {
                float intensity = Mathf.Lerp(maxShakeIntensity, 0f, dist / explosionCameraShakeRadius);
                CameraShake.Instance.Shake(intensity);
            }
        }

        PoolManager.Instance.DespawnAfterDelay(gameObject, destroyDelay);
    }

    private void Explode()
    {
        float radius = explosionCameraShakeRadius;

        var healths = Utility.GetHealthInRadius(transform.position, radius);

        foreach (var health in healths)
        {
            if (health == null) continue;

            Team targetTeam = health.GetComponentInParent<Team>();

            // 🚫 Friendly fire check
            if (targetTeam != null && targetTeam.teamId == ownerTeam)
                continue;

            // 📏 Distance-based damage falloff
            float distance = Vector3.Distance(transform.position, health.TankObject.transform.position);

            float damagePercent = 1f - (distance / radius);
            damagePercent = Mathf.Clamp01(damagePercent);

            float finalDamage = damage * damagePercent * damageMultiplier;

            health.TakeDamage(finalDamage, ownerTeam, OwnerStats);
            OwnerStats.IncreaseDamage(finalDamage);
        }
    }

    public void SetOwner(Collider owner)
    {
        ownerCollider = owner;
        OwnerStats = owner.GetComponentInParent<TankStats>();
    }

    public void SetDamage(float damageAmount)
    {
        damage = damageAmount;
    }

    private void SpawnTrail()
    {
        if (TrailParticleEffect == null) return;

        _currentTrail = PoolManager.Instance.Spawn(TrailParticleEffect, transform.position, Quaternion.identity);
        _currentTrail.transform.parent = transform;

        ParticleSystem ps = _currentTrail.GetComponentInChildren<ParticleSystem>();
        ps.Play();
    }

    private void DespawnTrail()
    {
        if (TrailParticleEffect == null) return;

        _currentTrail.transform.parent = null;
        ParticleSystem ps = _currentTrail.GetComponentInChildren<ParticleSystem>();
        ps.Stop();
        PoolManager.Instance.DespawnAfterDelay(_currentTrail, ps.main.duration + 0.5f);
    }
}
