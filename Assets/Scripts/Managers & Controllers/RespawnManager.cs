using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.AI;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class RespawnManager : MonoBehaviour
{
    public static RespawnManager Instance;

    [Header("Spawn Points")]
    public List<Transform> spawnPoints = new List<Transform>();

    public float randomSpawnRadius = 3f;
    public float smartSpawnRadius = 100f;

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

    public Vector3 GetSmartSpawnPoint(int teamId)
    {
        if (spawnPoints.Count == 0)
        {
            Debug.LogWarning("No spawn points assigned!");
            return Vector3.zero;
        }

        Transform bestPoint = null;
        float bestScore = float.MinValue;

        foreach (var point in spawnPoints)
        {
            float score = EvaluateSpawnPoint(point.position, teamId);

            if (score > bestScore)
            {
                bestScore = score;
                bestPoint = point;
            }
        }

        if (bestPoint == null)
            bestPoint = spawnPoints[Random.Range(0, spawnPoints.Count)];

        return GetRandomizedPoint(bestPoint.position);
    }

    float EvaluateSpawnPoint(Vector3 position, int teamId)
    {
        var teams = Utility.GetTeamsInRadius(position, smartSpawnRadius);

        int friendlyCount = 0;
        int enemyCount = 0;

        foreach (var team in teams)
        {
            if (team == null) continue;

            if (team.teamId == teamId)
                friendlyCount++;
            else
                enemyCount++;
        }

        float score = 0f;

        //score += friendlyCount * 5f;   // prefer teammates
        score -= enemyCount * 15f;      // avoid enemies harder

        // small randomness to prevent stacking
        score += Random.Range(0f, 20f);

        return score;
    }

    Vector3 GetRandomizedPoint(Vector3 basePosition)
    {
        for (int i = 0; i < 10; i++)
        {
            Vector3 offset = Random.insideUnitSphere * randomSpawnRadius;
            offset.y = 0;

            Vector3 candidate = basePosition + offset;

            if (NavMesh.SamplePosition(candidate, out NavMeshHit hit, 5f, NavMesh.AllAreas))
            {
                return hit.position;
            }
        }

        return basePosition;
    }



    [ContextMenu("Align Spawn Points To Ground")]
    void AlignSpawnPointsToGround()
    {
        if (spawnPoints == null || spawnPoints.Count == 0)
        {
            Debug.LogWarning("No spawn points assigned.");
            return;
        }

        int alignedCount = 0;

        foreach (var point in spawnPoints)
        {
            if (point == null) continue;

            Vector3 rayStart = point.position + Vector3.up * 50f;

            if (Physics.Raycast(rayStart, Vector3.down, out RaycastHit hit, 100f))
            {
    #if UNITY_EDITOR
                Undo.RecordObject(point, "Align Spawn Point");
    #endif
                point.position = hit.point;
                alignedCount++;
            }
        }

        Debug.Log($"Aligned {alignedCount} spawn points to ground.");
    }

private void OnDrawGizmos()
    {
        if (spawnPoints == null) return;

        foreach (Transform t in spawnPoints)
        {
            if (t == null) continue;

            // Draw the core spawn point
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(t.position, 0.5f);

            // Draw the flattened random spawn radius zone
            Gizmos.color = new Color(0f, 1f, 0f, 0.15f); // Semi-transparent green
            Matrix4x4 originalMatrix = Gizmos.matrix;

            // Squash the sphere gizmo into a flat cylinder/disc shape since y-offset is 0
            Gizmos.matrix = Matrix4x4.TRS(t.position, Quaternion.identity, new Vector3(1f, 0.05f, 1f));
            Gizmos.DrawWireSphere(Vector3.zero, randomSpawnRadius);
            Gizmos.DrawSphere(Vector3.zero, randomSpawnRadius);

            // Restore matrix to avoid breaking other gizmos
            Gizmos.matrix = originalMatrix;

            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(t.position, smartSpawnRadius);
        }
    }
}
