using UnityEngine;

public class CameraShake : MonoBehaviour
{
    public static CameraShake Instance;

    [Header("Settings")]
    public float maxPositionShake = 0.35f;
    public float maxRotationShake = 4f;
    public float traumaDecay = 1.5f;
    public float noiseFrequency = 25f;

    float trauma;

    float seedX;
    float seedY;
    float seedRot;

    public Vector3 PositionOffset { get; private set; }
    public Quaternion RotationOffset { get; private set; }

    void Awake()
    {
        Instance = this;

        seedX = Random.value * 100f;
        seedY = Random.value * 100f;
        seedRot = Random.value * 100f;
    }

    public void Shake(float amount)
    {
        trauma = Mathf.Clamp01(trauma + amount);
    }

    void LateUpdate()
    {
        trauma = Mathf.MoveTowards(trauma, 0f, traumaDecay * Time.deltaTime);

        float shake = trauma * trauma;

        float time = Time.time * noiseFrequency;

        PositionOffset = maxPositionShake * shake * new Vector3(
            (Mathf.PerlinNoise(seedX, time) - 0.5f) * 2f,
            (Mathf.PerlinNoise(seedY, time) - 0.5f) * 2f,
            0f
        );

        float rot =
            (Mathf.PerlinNoise(seedRot, time) - 0.5f) * 2f *
            maxRotationShake *
            shake;

        RotationOffset = Quaternion.Euler(0f, 0f, rot);
    }
}
