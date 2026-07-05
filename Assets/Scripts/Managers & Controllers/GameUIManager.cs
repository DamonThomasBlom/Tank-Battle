using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using static GameManager;

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

    public List<TeamUI> teamUIs = new();

    void Start()
    {
        if (GameManager.Instance == null) return;

        // Subscribe to events
        GameManager.Instance.OnScoreChanged += UpdateScores;
        GameManager.Instance.OnKillsChanged += UpdateKills;

        // Initial update
        UpdateAll();
        UpdateTeamColours();
    }

    void OnDestroy()
    {
        if (GameManager.Instance == null) return;

        GameManager.Instance.OnScoreChanged -= UpdateScores;
        GameManager.Instance.OnKillsChanged -= UpdateKills;
    }

    void UpdateTeamColours()
    {
        foreach (var ui in teamUIs)
        {
            var teamData = GameManager.Instance.GetTeamData(ui.teamId);
            if (teamData != null && ui.teamColorImage != null)
            {
                ui.teamColorImage.color = teamData.Colour;
            }
        }
    }

    void UpdateAll()
    {
        UpdateScores();
        UpdateKills();
    }

    void UpdateScores()
    {
        foreach (var ui in teamUIs)
        {
            var teamData = GameManager.Instance.GetTeamData(ui.teamId);
            if (teamData == null) continue;

            ui.scoreText.text = $"Score: {Mathf.RoundToInt(teamData.TeamScore)}";
        }
    }

    void UpdateKills()
    {
        foreach (var ui in teamUIs)
        {
            var teamData = GameManager.Instance.GetTeamData(ui.teamId);
            if (teamData == null) continue;

            ui.killsText.text = $"Kills: {teamData.TeamKills}";
        }
    }
}
