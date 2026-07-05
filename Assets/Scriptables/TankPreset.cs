using UnityEngine;

[CreateAssetMenu(fileName = "TankPreset", menuName = "Tanks/Tank Preset")]
public class TankPreset : ScriptableObject
{
    public string TankName = "Default Tank";

    [Header("AI Behaviour")]
    public float preferredDistance = 40f;
    public float accuracy = 0.8f;
    public float aimThreshold = 0.9f;
    public float detectionRadius = 75f;
    public float maxAimDistance = 75f;

    [Header("Movement")]
    public float moveSpeed = 6f;
    public float acceleration = 8f;
    public float angularSpeed = 60f;

    [Header("Combat")]
    public float fireCooldown = 2f;
    public float muzzleVelocity = 40f;
    public float gravityScale = 0.4f;
    public float damage = 30f;

    [Header("Stats")]
    public float maxHP = 100f;
    public float scale = 1f;
}
