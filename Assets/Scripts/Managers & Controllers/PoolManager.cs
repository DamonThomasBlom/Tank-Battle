using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class PoolManager : MonoBehaviour
{
    public static PoolManager Instance;

    [Serializable]
    public class PoolEntry
    {
        public int NumberToSpawn;
        public GameObject Prefab;
    };

    public List<PoolEntry> PoolsToPreload = new();

    Dictionary<GameObject, Queue<GameObject>> pools = new();
    Dictionary<GameObject, GameObject> instanceToPrefab = new(); // Maps instances to their prefabs

    void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }
        Instance = this;

        foreach(var entry in PoolsToPreload)
        {
            BatchSpawn(entry.Prefab, entry.NumberToSpawn);
        }
    }

    public void BatchSpawn(GameObject prefab, int count, bool childToPoolManager = true)
    {
        if (!pools.ContainsKey(prefab))
            pools[prefab] = new Queue<GameObject>();

        for (int i = 0; i < count; i++)
        {
            GameObject newPooledItem = Instantiate(prefab);
            if (childToPoolManager)
                newPooledItem.transform.SetParent(transform);
            newPooledItem.name = prefab.name + "_Pooled_" + i;
            newPooledItem.SetActive(false);

            // Store reference to prefab
            if (!instanceToPrefab.ContainsKey(newPooledItem))
                instanceToPrefab.Add(newPooledItem, prefab);

            pools[prefab].Enqueue(newPooledItem);
        }
    }

    public GameObject Spawn(GameObject prefab, Vector3 pos, Quaternion rot, bool childToPoolManager = true)
    {
        if (!pools.ContainsKey(prefab))
            pools[prefab] = new Queue<GameObject>();

        GameObject obj;

        // Try to get from pool
        while (pools[prefab].Count > 0)
        {
            obj = pools[prefab].Dequeue();

            // Safety check - object might have been destroyed elsewhere
            if (obj == null)
                continue;

            obj.transform.SetPositionAndRotation(pos, rot);
            obj.SetActive(true);

            // Ensure mapping exists
            if (!instanceToPrefab.ContainsKey(obj))
                instanceToPrefab.Add(obj, prefab);

            return obj;
        }

        // Create new if pool is empty
        obj = Instantiate(prefab, pos, rot);
        if (childToPoolManager)
            obj.transform.SetParent(transform);

        // Store reference to prefab
        if (!instanceToPrefab.ContainsKey(obj))
            instanceToPrefab.Add(obj, prefab);

        return obj;
    }

    public void Despawn(GameObject obj)
    {
        if (obj == null) return;

        if (despawnVersions.ContainsKey(obj))
            despawnVersions[obj]++;

        // Find which prefab this instance belongs to
        if (instanceToPrefab.TryGetValue(obj, out GameObject prefab))
        {
            if (!pools.ContainsKey(prefab))
                pools[prefab] = new Queue<GameObject>();

            if (!obj.activeSelf && pools[prefab].Contains(obj)) return; // Already in pool

            pools[prefab].Enqueue(obj);
            obj.SetActive(false);
        }
        else
        {
            // Object wasn't created through pool system
            Debug.LogWarning($"PoolManager: Object {obj.name} was not spawned through pool. Destroying instead.");
            Destroy(obj);
        }
    }

    // Optional helper method

    public void DespawnAfterDelay(GameObject obj, float delay)
    {
        if (!despawnVersions.ContainsKey(obj))
            despawnVersions[obj] = 0;

        despawnVersions[obj]++;

        int version = despawnVersions[obj];
        StartCoroutine(DelayedDespawn(obj, delay, version));
    }

    Dictionary<GameObject, int> despawnVersions = new();

    IEnumerator DelayedDespawn(GameObject obj, float delay, int version)
    {
        yield return new WaitForSeconds(delay);

        if (!despawnVersions.ContainsKey(obj)) yield break;

        // If version changed -> this is an old coroutine
        if (despawnVersions[obj] != version) yield break;

        Despawn(obj);
    }
}