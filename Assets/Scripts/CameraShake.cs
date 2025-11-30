using UnityEngine;

public class CameraShake : MonoBehaviour
{
    public static CameraShake instance;

    Vector3 originalPos;

    void Awake()
    {
        instance = this;
    }

    public void Shake(float intensity, float duration)
    {
        originalPos = transform.localPosition;

        // Cancel any ongoing tweens
        LeanTween.cancel(gameObject);

        // Small punch in random direction
        Vector3 punch = new Vector3(
            Random.Range(-1f, 1f) * intensity,
            Random.Range(-1f, 1f) * intensity,
            0f
        );

        // PunchPosition auto eases back to original
        LeanTween.moveLocal(gameObject, originalPos + punch, duration)
            .setEasePunch();
    }
}
