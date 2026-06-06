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
        Progress.value = CaptureZone.captureProgress;

        if (CaptureZone.teamOwner != previousTeamOwner)
        {
            previousTeamOwner = CaptureZone.teamOwner;
            var teamColour = GameManager.Instance.GetTeamColour(CaptureZone.teamOwner);
            Fill.color = teamColour;
        }
    }
}
