using UnityEngine;

public class TankBullet : MonoBehaviour
{
    [Header("Explosion")]
    public GameObject ParticleEffect;   // Prefab with particles + optional shockwave
    public Transform bulletGraphic;  // assign your bullet mesh here in Inspector

    [Header("Settings")]
    public float destroyDelay = 0.05f;
    public float explosionCameraShakeRadius = 10f;
    public float maxShakeIntensity = 1f;

    private Rigidbody rb;

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
        // Spawn particle effect
        if (ParticleEffect != null)
        {
            GameObject effect = Instantiate(
                ParticleEffect,
                transform.position,
                Quaternion.identity
            );

            // Destroy effect after its max particle lifetime
            ParticleSystem ps = effect.GetComponentInChildren<ParticleSystem>();
            if (ps != null)
            {
                Destroy(effect, ps.main.duration + ps.main.startLifetime.constantMax);
            }
            else
            {
                Destroy(effect, 3f); // fallback
            }
        }

        // Shake camera
        if (CameraShake.instance != null)
        {
            float dist = Vector3.Distance(transform.position, CameraShake.instance.transform.position);
            if (dist < explosionCameraShakeRadius)
            {
                float intensity = Mathf.Lerp(maxShakeIntensity, 0f, dist / explosionCameraShakeRadius);
                CameraShake.instance.Shake(intensity, 0.3f);
            }
        }

        // Destroy bullet
        Destroy(gameObject, destroyDelay);
    }
}
