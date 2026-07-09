using System.Collections.Generic;
using TriInspector;
using UnityEngine;
using UnityEngine.UI;

public class ColourSetter : MonoBehaviour
{
    public List<Image> FullColourChange = new();
    public List<Image> PartialColourChange = new();

    public Color Colour;

    private readonly Dictionary<Image, Color> _originalColours = new();

    private void Awake()
    {
        foreach (var image in FullColourChange)
        {
            _originalColours[image] = image.color;
        }

        foreach (var image in PartialColourChange)
        {
            _originalColours[image] = image.color;
        }
    }

    [Button]
    public void SetColour(Color colour)
    {
        foreach (var image in FullColourChange)
        {
            image.color = colour;
        }

        foreach (var image in PartialColourChange)
        {
            image.color = _originalColours[image] * colour;
        }
    }

    [Button]
    public void SetToDefaultColours()
    {
        foreach (var image in FullColourChange)
        {
            image.color = _originalColours[image];
        }

        foreach (var image in PartialColourChange)
        {
            image.color = _originalColours[image];
        }
    }
}
