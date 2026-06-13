using UnityEngine;
using UnityEngine.UIElements;

public class TankBullet : MonoBehaviour
{
    [Header("Explosion")]
    public GameObject ParticleEffect;   // Prefab with particles + optional shockwave
    public Transform bulletGraphic;  // assign your bullet mesh here in Inspector

    [Header("Damage")]
    public float damage = 25f;
    public int ownerTeam;

    [Header("Settings")]
    public float destroyDelay = 0.05f;
    public float explosionCameraShakeRadius = 10f;
    public float maxShakeIntensity = 1f;

    private Rigidbody rb;
    private Collider ownerCollider;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
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
        if (other == ownerCollider) return;

        Explode();

        // Spawn particle effect
        if (ParticleEffect != null)
        {
            GameObject effect = Instantiate(
                ParticleEffect,
                transform.position,
                Quaternion.identity
            );

            ParticleSystem ps = effect.GetComponentInChildren<ParticleSystem>();
            if (ps != null)
            {
                Destroy(effect, ps.main.duration + ps.main.startLifetime.constantMax);
            }
            else
            {
                Destroy(effect, 3f);
            }
        }

        // Camera shake
        if (CameraShake.instance != null)
        {
            float dist = Vector3.Distance(transform.position, CameraShake.instance.transform.position);
            if (dist < explosionCameraShakeRadius)
            {
                float intensity = Mathf.Lerp(maxShakeIntensity, 0f, dist / explosionCameraShakeRadius);
                CameraShake.instance.Shake(intensity, 0.3f);
            }
        }

        Destroy(gameObject, destroyDelay);
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

            float finalDamage = damage * damagePercent;

            health.TakeDamage(finalDamage, ownerTeam);
        }
    }

    public void SetOwner(GameObject owner)
    {
        ownerCollider = owner.GetComponent<Collider>();
    }
}
