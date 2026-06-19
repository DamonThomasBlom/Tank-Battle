using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;

public class VolumeTankSpawner : MonoBehaviour
{
    public bool spawnOnStart = true;
    public GameObject tankPrefab;
    public int spawnCount = 50;

    [Header("Validation")]
    public float minDistance = 8f;
    public LayerMask groundMask;
    public LayerMask obstacleMask;

    [Header("Raycast")]
    public float raycastHeight = 100f;

    [Header("NavMesh")]
    public bool requireNavMesh = true;
    public float navMeshSampleRadius = 4f;

    private List<Vector3> spawnedPositions = new List<Vector3>();

    private void Start()
    {
        if (spawnOnStart)
            Spawn();
    }

    [ContextMenu("Spawn Tanks")]
    public void Spawn()
    {
        spawnedPositions.Clear();

        Collider col = GetComponent<Collider>();
        if (col == null)
        {
            Debug.LogError("No collider found on spawner.");
            return;
        }

        int attempts = 0;
        int maxAttempts = spawnCount * 30;

        while (spawnedPositions.Count < spawnCount && attempts < maxAttempts)
        {
            attempts++;

            Vector3 randomPoint = GetRandomPointInCollider(col);

            // ☄️ Raycast down
            Vector3 rayStart = randomPoint + Vector3.up * raycastHeight;

            if (!Physics.Raycast(rayStart, Vector3.down, out RaycastHit hit, raycastHeight * 2f, groundMask))
                continue;

            Vector3 pos = hit.point;

            // 🧠 NavMesh check
            if (requireNavMesh)
            {
                if (!NavMesh.SamplePosition(pos, out NavMeshHit navHit, navMeshSampleRadius, NavMesh.AllAreas))
                    continue;

                pos = navHit.position;
            }

            // 🚫 Avoid obstacles
            if (Physics.CheckSphere(pos, 2f, obstacleMask))
                continue;

            // 🚫 Avoid overlap
            if (IsTooClose(pos))
                continue;

            SpawnTank(pos);
            spawnedPositions.Add(pos);
        }

        Debug.Log($"Spawned {spawnedPositions.Count}/{spawnCount} tanks (Attempts: {attempts})");
        col.enabled = false;
    }

    private Vector3 GetRandomPointInCollider(Collider col)
    {
        if (col is BoxCollider box)
        {
            Vector3 center = box.center;
            Vector3 size = box.size;

            Vector3 randomLocal = new Vector3(
                Random.Range(-size.x / 2f, size.x / 2f),
                Random.Range(-size.y / 2f, size.y / 2f),
                Random.Range(-size.z / 2f, size.z / 2f)
            );

            return box.transform.TransformPoint(center + randomLocal);
        }
        else if (col is SphereCollider sphere)
        {
            Vector3 randomDir = Random.insideUnitSphere * sphere.radius;
            return sphere.transform.TransformPoint(sphere.center + randomDir);
        }

        return transform.position;
    }

    private bool IsTooClose(Vector3 pos)
    {
        foreach (var p in spawnedPositions)
        {
            if (Vector3.Distance(p, pos) < minDistance)
                return true;
        }
        return false;
    }

    private void SpawnTank(Vector3 pos)
    {
        Quaternion rot = Quaternion.Euler(0, Random.Range(0, 360), 0);

        GameObject tank = Instantiate(tankPrefab, pos, rot, transform);

        AlignToGround(tank);
    }

    private void AlignToGround(GameObject obj)
    {
        if (Physics.Raycast(obj.transform.position + Vector3.up * 5f, Vector3.down, out RaycastHit hit, 20f))
        {
            obj.transform.up = hit.normal;
        }
    }
}