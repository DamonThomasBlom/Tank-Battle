using System;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    public static GameManager Instance;

    private void Awake()
    {
        Instance = this;
    }

    [Serializable]
    public class TeamData
    {
        public int TeamId;
        public Color Colour;
        public float TeamScore;
        public int TeamKills;
    }

    public List<TeamData> TeamsData = new();

    public event Action OnScoreChanged;
    public event Action OnKillsChanged;

    public Color GetTeamColour(int teamId)
    {
        foreach(var teamData in TeamsData) { if  (teamData.TeamId == teamId) return teamData.Colour; }

        return Color.white;
    }

    public void IncrementTeamScore(int teamId, float amount)
    {
        foreach (var teamData in TeamsData)
        {
            if (teamData.TeamId == teamId)
            {
                teamData.TeamScore += amount;
                OnScoreChanged?.Invoke();
            }
        }
    }

    public void IncrementTeamKills(int teamId, int amount)
    {
        foreach (var teamData in TeamsData)
        {
            if (teamData.TeamId == teamId)
            {
                teamData.TeamKills += amount;
                OnKillsChanged?.Invoke();
            }
        }
    }

    public TeamData GetTeamData(int teamId)
    {
        foreach (var team in TeamsData)
        {
            if (team.TeamId == teamId)
                return team;
        }
        return null;
    }
}
