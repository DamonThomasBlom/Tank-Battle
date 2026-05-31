using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class RespawnManager : MonoBehaviour
{
    public static RespawnManager Instance;

    [Header("Spawn Points")]
    public List<Transform> spawnPoints = new List<Transform>();

    public float randomSpawnRadius = 3f;

    void Awake()
    {
        Instance = this;
    }

    public Transform GetRandomSpawnPoint()
    {
        if (spawnPoints.Count == 0)
        {
            Debug.LogWarning("No spawn points assigned!");
            return null;
        }

        int index = Random.Range(0, spawnPoints.Count);
        return spawnPoints[index];
    }

    public Vector3 RandomSpawnPoint()
    {
        Transform t = GetRandomSpawnPoint();

        if (t == null)
            return Vector3.zero;

        for (int i = 0; i < 10; i++) // try multiple times
        {
            Vector3 randomOffset = Random.insideUnitSphere * randomSpawnRadius;
            randomOffset.y = 0;

            Vector3 candidate = t.position + randomOffset;

            if (NavMesh.SamplePosition(candidate, out NavMeshHit hit, 5f, NavMesh.AllAreas))
            {
                return hit.position;
            }
        }

        // fallback (center point)
        return t.position;
    }
}
