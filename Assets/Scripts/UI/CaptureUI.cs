using UnityEngine;
using UnityEngine.UI;

public class CaptureUI : MonoBehaviour
{
    public Slider Progress;
    public CaptureZone CaptureZone;
    public Image Fill;

    int previousTeamOwner = -1;

    private void Update()
    {
        if (Progress.value != CaptureZone.captureProgress)
            Progress.value = CaptureZone.captureProgress;

        if (CaptureZone.TeamOwner != previousTeamOwner)
        {
            previousTeamOwner = CaptureZone.TeamOwner;
            var teamColour = GameManager.Instance.GetTeamColour(CaptureZone.TeamOwner);
            Fill.color = teamColour;
        }
    }
}
