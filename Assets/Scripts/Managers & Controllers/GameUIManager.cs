using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class GameUIManager : MonoBehaviour
{
    [System.Serializable]
    public class TeamUI
    {
        public int teamId;
        public TextMeshProUGUI scoreText;
        public TextMeshProUGUI killsText;
        public Image teamColorImage;
    }

    //public List<TeamUI> teamUIs = new();
    [Header("Top Bar Game Status UI")]
    public List<TeamScoreItemUI> TeamUIs = new();
    public TextMeshProUGUI TimerText;
    public TextMeshProUGUI WinScoreText;

    [Header("Player Stats UI")]
    public TankStats PlayerStats;
    public TextMeshProUGUI PlayerNameText;
    public TextMeshProUGUI PlayerKillsText;
    public TextMeshProUGUI PlayerDeathsText;
    public TextMeshProUGUI PlayerDamageText;

    [Header("Zone Status UI")]
    public List<ZoneStatusUIItem> ZoneUIs = new();

    private Dictionary<ZoneStatusUIItem, CaptureZone> zoneUIToCaptureZoneMap = new();

    void Start()
    {
        if (GameManager.Instance == null) return;

        // Subscribe to events
        GameManager.Instance.OnScoreChanged += UpdateScores;
        if (PlayerStats != null)
        {
            PlayerStats.OnStatsUpdated += UpdatePlayerStats;
            UpdatePlayerStats();
        }

        // Initial update
        WinScoreText.text = GameManager.Instance.WinScore.ToString();
        UpdateScores();
        UpdateTeamColours();
        MapZonesUIToCaptureZones();
    }

    void OnDestroy()
    {
        if (GameManager.Instance == null) return;

        GameManager.Instance.OnScoreChanged -= UpdateScores;
        if (PlayerStats != null)
        {
            PlayerStats.OnStatsUpdated -= UpdatePlayerStats;
        }
    }

    private void Update()
    {
        UpdateGameTime();
        UpdateCaptureZonesStatus();
    }

    void UpdateGameTime()
    {
        TimerText.text = TimeSpan.FromSeconds(GameManager.Instance.GameTimeLeft).ToString(@"mm\:ss");
    }

    void UpdatePlayerStats()
    {
        if (PlayerStats == null) return;

        PlayerKillsText.text = PlayerStats.Kills.ToString();
        PlayerDeathsText.text = PlayerStats.Deaths.ToString();
        PlayerDamageText.text = PlayerStats.Damage.ToString("F0");
    }

    void UpdateTeamColours()
    {
        var AllTeams = GameManager.Instance.GetAllTeamsData();
        int teamsCount = AllTeams.Count;

        for (int i = 0; i < Math.Max(AllTeams.Count, TeamUIs.Count); i++)
        {
            if (i >= teamsCount)
            {
                TeamUIs[i].gameObject.SetActive(false);
                continue;
            }

            TeamUIs[i].Setup(0, GameManager.Instance.WinScore, AllTeams[i].Colour);
        }
    }

    void UpdateScores()
    {
        var AllTeams = GameManager.Instance.GetAllTeamsData();
        int teamsCount = AllTeams.Count;

        for (int i = 0; i < Math.Min(AllTeams.Count, TeamUIs.Count); i++)
        {
            TeamUIs[i].UpdateScore(AllTeams[i].TeamScore);
        }
    }

    void MapZonesUIToCaptureZones()
    {
        var allCaptureZones = FindObjectsByType<CaptureZone>();
        int zonesCount = allCaptureZones.Length;
        for (int i = 0; i < Math.Max(allCaptureZones.Length, ZoneUIs.Count); i++)
        {
            if (i >= zonesCount)
            {
                ZoneUIs[i].TurnOff();
                continue;
            }
            zoneUIToCaptureZoneMap.Add(ZoneUIs[i], allCaptureZones[i]);
            ZoneUIs[i].Setup(allCaptureZones[i].CaptureZoneId, allCaptureZones[i].maxCaptureProgress, allCaptureZones[i].transform);
        }
    }

    void UpdateCaptureZonesStatus()
    {
        foreach (var kvp in zoneUIToCaptureZoneMap)
        {
            var zoneUI = kvp.Key;
            var captureZone = kvp.Value;
            zoneUI.SetCaptureProgress(captureZone.captureProgress);
            Color zoneOwner = GameManager.Instance.GetTeamColour(captureZone.TeamOwner);
            Color capturingTeam = GameManager.Instance.GetTeamColour(captureZone.CurrentBestTeam);
            zoneUI.SetStatus(captureZone.ZoneState, zoneOwner, capturingTeam);
        }
    }
}
