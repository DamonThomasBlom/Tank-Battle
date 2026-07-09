using System.Collections.Generic;
using TMPro;
using TriInspector;
using UnityEngine;
using UnityEngine.UI;

public class QuickZoneItemUI : MonoBehaviour
{
    public TextMeshProUGUI ZoneIdText;
    public ColourSetter ColourSetter;
    public List<Image> ShadowImages;

    private Color _originalShadowColour;

    private void Awake()
    {
        _originalShadowColour = ShadowImages.Count > 0 ? ShadowImages[0].color : Color.black;
    }

    public void Setup(string zoneId)
    {
        ZoneIdText.text = zoneId;
    }

    [Button]
    public void UpdateCaptureColour(Color colour)
    {
        ColourSetter.SetColour(colour);
    }

    [Button]
    public void SetToDefaultColour()
    {
        ColourSetter.SetToDefaultColours();
    }

    [Button]
    public void SetShadowColour(Color shadowColor)
    {
        foreach (var image in ShadowImages)
        {
            image.color = shadowColor;
        }
    }

    [Button]
    public void SetShadowBackToDefault()
    {
        foreach (var image in ShadowImages)
        {
            image.color = _originalShadowColour;
        }
    }
}
