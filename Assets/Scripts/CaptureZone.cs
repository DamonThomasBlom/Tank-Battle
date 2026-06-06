using System.Collections.Generic;
using UnityEngine;

public class CaptureZone : MonoBehaviour
{
    [Header("Ownership")]
    public int teamOwner = -1; // -1 = neutral

    [Header("Capture")]
    public float captureProgress = 0f;
    public float captureSpeed = 5f;
    public float decaptureSpeed = 7f;
    public float maxCaptureProgress = 100f;

    [Header("Scoring")]
    public float scorePerSecond = 1f;

    [Header("References")]
    public Transform centerPoint;

    [Header("Tanks Inside")]
    public List<Team> tanksInZone = new List<Team>();

    void Update()
    {
        UpdateCapture();
        UpdateScore();
    }

    void UpdateScore()
    {
        // Only give score if fully captured
        if (teamOwner == -1) return;
        if (captureProgress < maxCaptureProgress) return;

        GameManager.Instance.IncrementTeamScore(teamOwner, scorePerSecond * Time.deltaTime);
    }

    void UpdateCapture()
    {
        if (tanksInZone.Count == 0)
            return; // ❌ DO NOTHING → no more decay when empty

        Dictionary<int, int> teamCounts = new Dictionary<int, int>();

        foreach (var t in tanksInZone)
        {
            if (!teamCounts.ContainsKey(t.teamId))
                teamCounts[t.teamId] = 0;

            teamCounts[t.teamId]++;
        }

        // Find top 2 teams
        int bestTeam = -1;
        int bestCount = 0;

        int secondBestCount = 0;

        foreach (var kvp in teamCounts)
        {
            if (kvp.Value > bestCount)
            {
                secondBestCount = bestCount;
                bestCount = kvp.Value;
                bestTeam = kvp.Key;
            }
            else if (kvp.Value > secondBestCount)
            {
                secondBestCount = kvp.Value;
            }
        }

        int netAdvantage = bestCount - secondBestCount;

        // If equal → contested → no movement
        if (netAdvantage <= 0)
            return;

        float delta = netAdvantage * captureSpeed * Time.deltaTime;

        // 🔥 If zone owned by someone else → move toward neutral first
        if (teamOwner != -1 && teamOwner != bestTeam)
        {
            captureProgress -= delta;

            if (captureProgress <= 0f)
            {
                captureProgress = 0f;
                teamOwner = -1;
            }

            return;
        }

        // 🔥 Capturing or reinforcing
        captureProgress += delta;

        if (captureProgress >= maxCaptureProgress)
        {
            captureProgress = maxCaptureProgress;
            teamOwner = bestTeam;
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        Team tank = other.GetComponentInParent<Team>();

        if (tank != null && !tanksInZone.Contains(tank))
        {
            tanksInZone.Add(tank);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        Team tank = other.GetComponentInParent<Team>();

        if (tank != null)
        {
            tanksInZone.Remove(tank);
        }
    }
}
