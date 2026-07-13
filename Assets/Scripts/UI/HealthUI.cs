using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using Random = UnityEngine.Random;

public class HealthUI : MonoBehaviour
{
    public Health Health;
    public Slider HealthSlider;
    public TextMeshProUGUI HealthText;

    [Header("Distance Scaling")]
    public bool EnableDistanceScaling = true;
    public float MinDistance = 10f;
    public float MaxDistance = 50f;
    public float MinScale = 0.4f;
    public float MaxScale = 1f;
    public float UpdateInterval = 0.1f;

    private Transform mainCamera;

    private float sqrMaxDistance;
    private float timer;

    private Vector3 originalScale;
    private bool currentlyVisible = true;

    private void Awake()
    {
        originalScale = HealthSlider.transform.localScale;
    }

    private void Start()
    {
        if (Health == null)
        {
            Debug.LogError("Health reference is not set on HealthUI.");
            return;
        }
        if (HealthSlider == null)
        {
            Debug.LogError("HealthSlider reference is not set on HealthUI.");
            return;
        }

        mainCamera = Camera.main.transform;

        sqrMaxDistance = MaxDistance * MaxDistance;

        timer = Random.Range(0f, UpdateInterval);

        HealthSlider.maxValue = Health.maxHealth;
        HealthSlider.value = Health.currentHealth;

        Health.OnHealthChange.AddListener(UpdateHealthUI);
        Invoke(nameof(UpdateHealthUI), 1f);
    }

    private void Update()
    {
        if (!EnableDistanceScaling || mainCamera == null)
            return;

        timer -= Time.deltaTime;

        if (timer > 0f)
            return;

        timer = UpdateInterval;

        UpdateDistanceScaling();
    }

    private void UpdateDistanceScaling()
    {
        float sqrDistance = (mainCamera.position - HealthSlider.transform.position).sqrMagnitude;

        bool shouldBeVisible = sqrDistance <= sqrMaxDistance;

        if (currentlyVisible != shouldBeVisible)
        {
            currentlyVisible = shouldBeVisible;
            HealthSlider.gameObject.SetActive(shouldBeVisible);
        }

        if (!shouldBeVisible)
            return;

        float distance = Mathf.Sqrt(sqrDistance);

        float t = Mathf.InverseLerp(MinDistance, MaxDistance, distance);
        float scale = Mathf.Lerp(MinScale, MaxScale, t);

        Vector3 targetScale = originalScale * scale;

        if ((HealthSlider.transform.localScale - targetScale).sqrMagnitude > 0.0001f)
            HealthSlider.transform.localScale = targetScale;
    }

    private void UpdateHealthUI()
    {
        HealthSlider.value = Health.currentHealth;

        if (HealthText)
        {
            HealthText.text = $"{Math.Max(Health.currentHealth, 0).ToString("F0")}/{Health.maxHealth}";
        }
    }
}
