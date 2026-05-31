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

    [Header("References")]
    public Transform centerPoint;

    [Header("Tanks Inside")]
    public List<Team> tanksInZone = new List<Team>();

    void Update()
    {
        UpdateCapture();
    }

    void UpdateCapture()
    {
        if (tanksInZone.Count == 0)
        {
            // Slowly decay back to neutral
            captureProgress = Mathf.MoveTowards(captureProgress, 0f, decaptureSpeed * Time.deltaTime);

            if (captureProgress <= 0f)
                teamOwner = -1;

            return;
        }

        int attackingTeam = GetDominantTeam();

        // If mixed teams → contested → slow or no capture
        if (attackingTeam == -2)
        {
            captureProgress = Mathf.MoveTowards(captureProgress, 0f, decaptureSpeed * Time.deltaTime);
            return;
        }

        // If already owned by same team → no need to capture
        if (teamOwner == attackingTeam)
        {
            captureProgress = Mathf.MoveTowards(captureProgress, 100f, captureSpeed * Time.deltaTime);
            return;
        }

        // If neutral or enemy owned → capture
        captureProgress += captureSpeed * Time.deltaTime;

        if (captureProgress >= 100f)
        {
            teamOwner = attackingTeam;
            captureProgress = 100f;
        }
    }

    int GetDominantTeam()
    {
        if (tanksInZone.Count == 0)
            return -1;

        int team = tanksInZone[0].teamId;

        foreach (var tank in tanksInZone)
        {
            // If multiple teams present → contested
            if (tank.teamId != team)
                return -2;
        }

        return team;
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
