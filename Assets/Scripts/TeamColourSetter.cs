using System.Collections.Generic;
using UnityEngine;

public class TeamColourSetter : MonoBehaviour
{
    public List<Renderer> Renderers = new();

    private Team _team;

    private void Start()
    {
        _team = GetComponentInParent<Team>();
        if (_team != null)
        {
            foreach(var renderer in Renderers)
            {
                // Create material instance so we don't change the material for all objects using the same material
                renderer.material = new Material(renderer.material);
                renderer.material.color = GameManager.Instance.GetTeamColour(_team.teamId);
            }
        }
    }
}
