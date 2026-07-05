using UnityEngine;
using UnityEngine.AI;

public class TankPresetApplier : MonoBehaviour
{
    private TankAI ai;
    private TankCannon cannon;
    private NavMeshAgent agent;
    private Health health;

    void Awake()
    {
        ai = GetComponentInChildren<TankAI>();
        cannon = GetComponentInChildren<TankCannon>();
        agent = GetComponentInChildren<NavMeshAgent>();
        health = GetComponentInChildren<Health>();
    }

    public void ApplyPreset(TankPreset preset)
    {
        if (preset == null) return;

        // --- AI ---
        ai.preferredDistance = preset.preferredDistance;
        ai.accuracy = preset.accuracy;
        ai.aimThreshold = preset.aimThreshold;
        ai.enemyDetectionRadius = preset.detectionRadius;
        ai.maxAimDistance = preset.maxAimDistance;

        // --- Cannon ---
        cannon.fireCooldown = preset.fireCooldown;
        cannon.muzzleVelocity = preset.muzzleVelocity;
        cannon.gravityScale = preset.gravityScale;
        cannon.damage = preset.damage;

        // --- Movement ---
        agent.speed = preset.moveSpeed;
        agent.acceleration = preset.acceleration;
        agent.angularSpeed = preset.angularSpeed;

        // --- Stats ---
        if (health != null)
        {
            health.SetMaxHealth(preset.maxHP); // make sure you have this
        }

        ai.transform.localScale = Vector3.one * preset.scale;
    }
}
