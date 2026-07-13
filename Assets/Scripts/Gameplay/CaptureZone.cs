using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Events;

public enum CaptureZoneState
{
    Neutral,
    Capturing,
    Contested,
    Losing,
    Secured,
    Enemy,
    EnemyCapturing
}

public class CaptureZone : MonoBehaviour
{
    [Header("Ownership")]
    public string CaptureZoneId;
    public int TeamOwner = -1; // -1 = neutral
    public int CurrentBestTeam = -1; 
    public CaptureZoneState ZoneState; // Purely local state used for UI and visual feedback
    public UnityEvent<CaptureZoneState> OnZoneStateChanged;

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

    private Collider zoneCollider;

    void Awake()
    {
        zoneCollider = GetComponent<Collider>();
    }

    private void Start()
    {
        SetZoneState(CaptureZoneState.Neutral);
    }

    void Update()
    {
        UpdateCapture();
        UpdateScore();
    }

    void UpdateScore()
    {
        // Only give score if fully captured
        if (TeamOwner == -1) return;
        if (captureProgress < maxCaptureProgress) return;

        GameManager.Instance.IncrementTeamScore(TeamOwner, scorePerSecond * Time.deltaTime);
    }

    void UpdateCapture()
    {
        if (tanksInZone.Count == 0)
        {
            CurrentBestTeam = -1;

            // Update idle state for the UI
            if (TeamOwner == -1)
                SetZoneState(CaptureZoneState.Neutral);
            else if (TeamOwner == Player.Instance.TeamID)
                SetZoneState(CaptureZoneState.Secured);
            else
                SetZoneState(CaptureZoneState.Enemy);

            return;
        }

        Dictionary<int, int> teamCounts = new();

        foreach (var t in tanksInZone)
        {
            if (!teamCounts.ContainsKey(t.teamId))
                teamCounts[t.teamId] = 0;

            teamCounts[t.teamId]++;
        }

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

        CurrentBestTeam = bestTeam;

        int netAdvantage = bestCount - secondBestCount;

        if (netAdvantage <= 0)
        {
            SetZoneState(CaptureZoneState.Contested);
            return;
        }

        float delta = netAdvantage * captureSpeed * Time.deltaTime;

        // Enemy is removing ownership
        if (TeamOwner != -1 && TeamOwner != bestTeam)
        {
            captureProgress -= delta;

            if (TeamOwner == Player.Instance.TeamID)
                SetZoneState(CaptureZoneState.Losing);
            else if (bestTeam == Player.Instance.TeamID)
                SetZoneState(CaptureZoneState.Capturing); // We are reclaiming an enemy zone
            else 
                SetZoneState(CaptureZoneState.EnemyCapturing);

            if (captureProgress <= 0f)
            {
                captureProgress = 0f;
                TeamOwner = -1;
            }

            return;
        }

        // Capturing/reinforcing
        captureProgress += delta;

        if (bestTeam == Player.Instance.TeamID)
            SetZoneState(CaptureZoneState.Capturing);
        else 
            SetZoneState(CaptureZoneState.EnemyCapturing);

        if (captureProgress >= maxCaptureProgress)
        {
            captureProgress = maxCaptureProgress;
            TeamOwner = bestTeam;
            CurrentBestTeam = -1;

            if (TeamOwner == Player.Instance.TeamID)
                SetZoneState(CaptureZoneState.Secured);
            else
                SetZoneState(CaptureZoneState.Enemy);
        }
    }

    public void SetZoneState(CaptureZoneState state)
    {
        if (ZoneState == state)
            return;
        ZoneState = state;
        OnZoneStateChanged.Invoke(state);
    }

    public void UpdateCaptureZoneData(GameManager.CaptureZoneConfigData config)
    {
        captureSpeed = config.captureSpeed;
        decaptureSpeed = config.decaptureSpeed;
        maxCaptureProgress = config.maxCaptureProgress;
        scorePerSecond = config.scorePerSecond;
    }

    public Vector3 GetValidNavMeshPoint(int maxAttempts = 10)
    {
        if (zoneCollider == null)
            return transform.position;

        for (int i = 0; i < maxAttempts; i++)
        {
            Vector3 randomPoint = GetRandomPointInCollider(zoneCollider);

            // Raycast down (like your spawner)
            Vector3 rayStart = randomPoint + Vector3.up * 50f;

            if (Physics.Raycast(rayStart, Vector3.down, out RaycastHit hit, 100f))
            {
                Vector3 pos = hit.point;

                // Snap to NavMesh
                if (NavMesh.SamplePosition(pos, out NavMeshHit navHit, 4f, NavMesh.AllAreas))
                {
                    return navHit.position;
                }
            }
        }

        // 🔥 Fallback (VERY IMPORTANT)
        if (NavMesh.SamplePosition(centerPoint.position, out NavMeshHit fallback, 5f, NavMesh.AllAreas))
            return fallback.position;

        return centerPoint.position;
    }

    private Vector3 GetRandomPointInCollider(Collider col)
    {
        if (col is BoxCollider box)
        {
            Vector3 local = new Vector3(
                Random.Range(-box.size.x * 0.5f, box.size.x * 0.5f),
                Random.Range(-box.size.y * 0.5f, box.size.y * 0.5f),
                Random.Range(-box.size.z * 0.5f, box.size.z * 0.5f)
            );

            return box.transform.TransformPoint(box.center + local);
        }

        if (col is SphereCollider sphere)
        {
            Vector3 local = Random.insideUnitSphere * sphere.radius;
            return sphere.transform.TransformPoint(sphere.center + local);
        }

        // fallback
        return col.bounds.center;
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
