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

    public bool debug;

    public UnityEvent OnDie = new UnityEvent();
    public UnityEvent OnTakeDamage = new UnityEvent();

    private bool isDead = false;
    private NavMeshAgent agent;
    private Team team;

    void Start()
    {
        agent = GetComponentInChildren<NavMeshAgent>();
        team = GetComponentInChildren<Team>();
        currentHealth = maxHealth;
    }

    public void SetMaxHealth(float value)
    {
        maxHealth = value;
        currentHealth = value;
    }

    public void TakeDamage(float amount, int attackerTeam)
    {
        if (isDead) return;

        Team team = GetComponent<Team>();

        // 🚫 Friendly fire check
        if (team != null && team.teamId == attackerTeam)
            return;

        currentHealth -= amount;

        if (debug)
            Debug.Log("Take Damage: " + amount + ", Current Health: " + currentHealth);

        OnTakeDamage.Invoke();

        if (currentHealth <= 0)
        {
            GameManager.Instance?.IncrementTeamKills(attackerTeam, 1);
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
        Vector3 spawn = RespawnManager.Instance.GetSmartSpawnPoint(team.teamId);

        if (spawn != null)
        {
            if (agent != null)
                agent.enabled = false;

            TankObject.transform.position = spawn;
            var rb = TankObject.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.MovePosition(spawn);
            }

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