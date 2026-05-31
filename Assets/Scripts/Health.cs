using System.Collections;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Events;

public class Health : MonoBehaviour
{
    public float maxHealth = 100f;
    public float currentHealth;

    [Header("Respawn")]
    public Transform TankObject;
    public float respawnDelay = 3f;

    public UnityEvent OnDie = new UnityEvent();

    private bool isDead = false;
    private NavMeshAgent agent;

    void Start()
    {
        agent = GetComponentInChildren<NavMeshAgent>();
        currentHealth = maxHealth;
    }

    public void TakeDamage(float amount, int attackerTeam)
    {
        if (isDead) return;

        Team team = GetComponent<Team>();

        // 🚫 Friendly fire check
        if (team != null && team.teamId == attackerTeam)
            return;

        currentHealth -= amount;

        //Debug.Log("Take Damage: " + amount + ", Current Health: " + currentHealth);

        if (currentHealth <= 0)
        {
            Die();
        }
    }

    void Die()
    {
        //Debug.Log("Died: " + gameObject.name);
        OnDie.Invoke();
        isDead = true;

        StartCoroutine(RespawnRoutine());
    }

    IEnumerator RespawnRoutine()
    {
        yield return null;
        TankObject.gameObject.SetActive(false);

        yield return new WaitForSeconds(respawnDelay);

        TankObject.gameObject.SetActive(true);

        // Reset position
        Vector3 spawn = RespawnManager.Instance.RandomSpawnPoint();

        if (spawn != null)
        {
            if (agent != null)
                agent.enabled = false;

            TankObject.transform.position = spawn;

            yield return null;

            if (agent != null)
            {
                agent.enabled = true;
                agent.Warp(spawn);
            }
        }

        // Reset health
        currentHealth = maxHealth;
        isDead = false;

    }
}