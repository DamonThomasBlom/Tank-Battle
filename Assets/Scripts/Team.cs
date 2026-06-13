using NUnit.Framework;
using System.Collections.Generic;
using UnityEngine;

public class Team : MonoBehaviour
{
    public int teamId;

    public static List<Team> AllTeams = new List<Team>();

    private void OnEnable() => AllTeams.Add(this);
    private void OnDisable() => AllTeams.Remove(this);

}