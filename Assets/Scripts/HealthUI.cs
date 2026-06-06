using UnityEngine;
using UnityEngine.UI;

public class HealthUI : MonoBehaviour
{
    public Health Health;
    public Slider HealthSlider;

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
        // Initialize the slider's max value and current value
        HealthSlider.maxValue = Health.maxHealth;
        HealthSlider.value = Health.currentHealth;
        // Subscribe to health change events
        Health.OnTakeDamage.AddListener(UpdateHealthUI);
    }

    private void Update()
    {
        UpdateHealthUI();
    }

    private void UpdateHealthUI()
    {
        HealthSlider.value = Health.currentHealth;
    }


}
