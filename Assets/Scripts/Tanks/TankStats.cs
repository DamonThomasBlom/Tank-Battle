using System;
using System.Collections.Generic;
using UnityEngine;

public class TankStats : MonoBehaviour
{
    public static readonly List<TankStats> All = new();

    public int TeamId;
    public string TankName;
    public int Kills;
    public int Deaths;
    public float Damage;

    public Action OnStatsUpdated;

    private void OnEnable()
    {
        TeamId = GetComponent<Team>().teamId;
        Invoke(nameof(AssignTankName), 0.1f);
        All.Add(this);
    }

    private void OnDisable()
    {
        All.Remove(this);
    }

    void AssignTankName()
    {
        TankName = gameObject.name;
    }

    public void AddKill()
    {
        Kills++;
        OnStatsUpdated?.Invoke();
    }

    public void AddDeath()
    {
        Deaths++;
        OnStatsUpdated?.Invoke();
    }

    public void IncreaseDamage(float amount)
    {
        Damage += amount;
        OnStatsUpdated?.Invoke();
    }
}
