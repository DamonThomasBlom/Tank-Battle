using NUnit.Framework;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Utility : MonoBehaviour
{
    public static List<Team> GetSpecificTeamInRadius(int teamId, Vector3 position, float radius)
    {
        Collider[] hits = Physics.OverlapSphere(position, radius);
        HashSet<Team> uniqueTeamMembers = new HashSet<Team>();

        foreach (var hit in hits)
        {
            Team team = hit.GetComponentInParent<Team>();

            if (team == null) continue;
            if (team.teamId != teamId) continue;

            uniqueTeamMembers.Add(team);
        }

        return uniqueTeamMembers.ToList();
    }

    public static List<Team> GetTeamsInRadius(Vector3 position, float radius)
    {
        Collider[] hits = Physics.OverlapSphere(position, radius);
        HashSet<Team> uniqueTeamMembers = new HashSet<Team>();

        foreach (var hit in hits)
        {
            Team team = hit.GetComponentInParent<Team>();

            if (team == null) continue;

            uniqueTeamMembers.Add(team);
        }

        return uniqueTeamMembers.ToList();
    }

    public static List<Health> GetHealthInRadius(Vector3 position, float radius)
    {
        Collider[] hits = Physics.OverlapSphere(position, radius);
        HashSet<Health> uniqueHealths = new HashSet<Health>();

        foreach (var hit in hits)
        {
            Health health = hit.GetComponentInParent<Health>();
            if (health == null) continue;
            uniqueHealths.Add(health);
        }

        return uniqueHealths.ToList();
    }
}
