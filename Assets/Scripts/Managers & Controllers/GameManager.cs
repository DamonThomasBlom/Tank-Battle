using System;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    #region SINGLETON
    public static GameManager Instance;

    private void Awake()
    {
        Instance = this;
    }
    #endregion

    public enum EGameState
    {
        WaitingForPlayers,
        InProgress,
        GameOver
    }

    [Serializable]
    public class TeamData
    {
        public int TeamId;
        public Color Colour;
        public float TeamScore;
        public int TeamKills;
    }

    [Serializable]
    public class CaptureZoneConfigData
    {
        public float captureSpeed = 5f;
        public float decaptureSpeed = 7f;
        public float maxCaptureProgress = 100f;
        public float scorePerSecond = 1f;
    }

    public List<TeamData> TeamsData = new();

    public CaptureZoneConfigData CaptureZoneConfig = new CaptureZoneConfigData();
    public bool useCaptureZoneConfigData;

    public Action OnScoreChanged;
    public Action OnKillsChanged;
    public Action<EGameState> OnGameStateChanged;

    [Header("Game Settings")]
    public EGameState GameState = EGameState.WaitingForPlayers;
    public int WinScore = 500;
    public float GameStartCountDown = 10f;
    public float GameTime;
    public float GameTimeLimit = 300f; // 5 minutes
    public int TeamIdWinner = -1;

    private void Start()
    {
        if (useCaptureZoneConfigData)
        {
            foreach(var captureZone in FindObjectsByType<CaptureZone>()) 
            { 
                captureZone.UpdateCaptureZoneData(CaptureZoneConfig); 
            }
        }

        SetGameState(EGameState.WaitingForPlayers);
    }

    private void Update()
    {
        switch (GameState)
        {
            case EGameState.WaitingForPlayers:
                GameStartCountDown -= Time.deltaTime;
                if (GameStartCountDown <= 0f)
                    SetGameState(EGameState.InProgress);
                break;

            case EGameState.InProgress:
                GameTime += Time.deltaTime;
                if (GameTime >= GameTimeLimit)
                {
                    SetGameState(EGameState.GameOver);
                }
                break;

            case EGameState.GameOver:
                break;
        }
    }

    void SetGameState(EGameState newState)
    {
        if (GameState == newState) return;

        Debug.Log($"Game State Changed: {GameState} -> {newState}");
        GameState = newState;

        // If its game over, calculate the winner
        if (newState == EGameState.GameOver)
            CalculateWinner();

        OnGameStateChanged?.Invoke(newState);
    }

    void CalculateWinner()
    {
        int winningTeamId = -1;
        float highestScore = -1f;
        foreach (var teamData in TeamsData)
        {
            if (teamData.TeamScore > highestScore)
            {
                highestScore = teamData.TeamScore;
                winningTeamId = teamData.TeamId;
            }
        }
        TeamIdWinner = winningTeamId;
    }

    public bool CanPlayerMove()
    {
        return GameState == EGameState.InProgress;
    }

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
                int previousWholeScore = Mathf.FloorToInt(teamData.TeamScore);

                teamData.TeamScore += amount;

                int currentWholeScore = Mathf.FloorToInt(teamData.TeamScore);

                if (teamData.TeamScore >= WinScore)
                {
                    SetGameState(EGameState.GameOver);
                }

                if (previousWholeScore != currentWholeScore)
                {
                    OnScoreChanged?.Invoke();
                }

                break;
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
