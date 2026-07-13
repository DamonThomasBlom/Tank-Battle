using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class ZoneStatusUIItem : MonoBehaviour
{
    public QuickZoneItemUI ZoneUI;
    public QuickZoneItemUI QuickZoneUI;
    public Slider CaptureProgressSlider;
    public Image SliderImage;
    public TextMeshProUGUI ZoneStatus;

    public void Setup(string zoneId, float maxCaptureValue)
    {
        ZoneUI.Setup(zoneId);
        QuickZoneUI.Setup(zoneId);
        CaptureProgressSlider.maxValue = maxCaptureValue;
        CaptureProgressSlider.value = 0f;
    }

    public void SetCaptureProgress(float progress)
    {
        CaptureProgressSlider.value = progress;
    }

    public void SetStatus(
        CaptureZoneState state,
        Color ownerColour,
        Color capturingColour)
    {
        // Neutral zones are always white
        if (state == CaptureZoneState.Neutral)
            ownerColour = Color.white;

        ZoneUI.UpdateCaptureColour(ownerColour);
        QuickZoneUI.UpdateCaptureColour(ownerColour);

        switch (state)
        {
            case CaptureZoneState.Neutral:
                ZoneStatus.text = "NEUTRAL";
                SliderImage.color = Color.white;
                QuickZoneUI.SetShadowBackToDefault();
                break;

            case CaptureZoneState.Capturing:
                ZoneStatus.text = "CAPTURING";
                SliderImage.color = capturingColour;
                QuickZoneUI.SetShadowColour(capturingColour);
                break;

            case CaptureZoneState.EnemyCapturing:
                ZoneStatus.text = "ENEMY CAPTURING";
                SliderImage.color = capturingColour;
                QuickZoneUI.SetShadowColour(capturingColour);
                break;

            case CaptureZoneState.Contested:
                ZoneStatus.text = "CONTESTED";
                SliderImage.color = Color.white;
                QuickZoneUI.SetShadowBackToDefault();
                break;

            case CaptureZoneState.Losing:
                ZoneStatus.text = "UNDER ATTACK";
                SliderImage.color = capturingColour;
                QuickZoneUI.SetShadowColour(capturingColour);
                break;

            case CaptureZoneState.Secured:
                ZoneStatus.text = "SECURED";
                SliderImage.color = ownerColour;
                QuickZoneUI.SetShadowBackToDefault();
                break;

            case CaptureZoneState.Enemy:
                ZoneStatus.text = "ENEMY";
                SliderImage.color = ownerColour;
                QuickZoneUI.SetShadowBackToDefault();
                break;
        }
    }

    public void TurnOff()
    {
        QuickZoneUI.gameObject.SetActive(false);
        gameObject.SetActive(false);
    }
}
