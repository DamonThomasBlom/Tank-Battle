using NUnit.Framework;
using System.Collections.Generic;
using UnityEngine;

public class Team : MonoBehaviour
{
    public int teamId;

    public Transform TankObject;
    public List<Collider> TanksColliders = new List<Collider>();
    public List<Collider> CollidersToExclude = new List<Collider>();

    public static List<Team> AllTeams = new List<Team>();

    private void Awake()
    {
        var collliders = GetComponentsInChildren<Collider>();
        foreach (var col in collliders)
        {
            if (!CollidersToExclude.Contains(col))
                TanksColliders.Add(col);
        }
    }

    private void OnEnable() => AllTeams.Add(this);
    private void OnDisable() => AllTeams.Remove(this);

}