using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ColourSetter : MonoBehaviour
{
    public List<Image> FullColourChange = new();
    public List<Image> PartialColourChange = new();

    public Color Colour;

    private void Start()
    {
        SetColour(Colour);
    }

    public void SetColour(Color colour)
    {
        foreach (var image in FullColourChange)
        {
            image.color = colour;
        }
        foreach (var image in PartialColourChange)
        {
            image.color *= colour;
        }
    }
}
