using UnityEngine;
using UnityEngine.UI;

public class CaptureUI : MonoBehaviour
{
    public Slider Progress;
    public CaptureZone CaptureZone;

    private void Update()
    {
        Progress.value = CaptureZone.captureProgress;
    }
}
