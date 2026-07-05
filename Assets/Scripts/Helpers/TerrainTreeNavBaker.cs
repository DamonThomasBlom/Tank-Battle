using UnityEditor;
using UnityEngine;

public class TerrainTreeNavBaker
{
    private static GameObject parent;

    [MenuItem("Tools/NavMesh/Step 1 - Generate Obstacles")]
    public static void GenerateObstacles()
    {
        Terrain terrain = Terrain.activeTerrain;
        if (terrain == null)
        {
            Debug.LogError("No active terrain found.");
            return;
        }

        if (parent != null)
        {
            Debug.LogWarning("Obstacles already exist. Clean up first.");
            return;
        }

        parent = new GameObject("Temp_Nav_Obstacles");

        TerrainData data = terrain.terrainData;
        var prototypes = data.treePrototypes;

        foreach (TreeInstance tree in data.treeInstances)
        {
            GameObject obstacle = CreateObstacleForTree(tree, prototypes, terrain, data);

            if (obstacle == null) continue;

            obstacle.transform.parent = parent.transform;
        }

        EditorUtility.DisplayDialog(
            "Step 1 Complete",
            "Obstacles generated.\n\nNow bake your NavMesh.\n\nThen run Step 2 to clean up.",
            "OK"
        );
    }

    [MenuItem("Tools/NavMesh/Step 2 - Cleanup Obstacles")]
    public static void Cleanup()
    {
        if (parent != null)
        {
            Object.DestroyImmediate(parent);
            parent = null;
        }
    }

    private static GameObject CreateObstacleForTree(TreeInstance tree, TreePrototype[] prototypes, Terrain terrain, TerrainData data)
    {
        var proto = prototypes[tree.prototypeIndex];

        if (proto.prefab == null)
            return null;

        Debug.Log("Scale " + proto.prefab.transform.localScale.x);

        // 🌍 Calculate correct world position FIRST
        Vector3 worldPos = Vector3.Scale(tree.position, data.size) + terrain.transform.position;
        Quaternion rotation = Quaternion.Euler(0, tree.rotation * Mathf.Rad2Deg, 0);
        Vector3 scale = new Vector3(tree.widthScale, tree.heightScale, tree.widthScale);

        // 👻 Temp instance for accurate bounds
        GameObject tempInstance = (GameObject)PrefabUtility.InstantiatePrefab(proto.prefab);
        tempInstance.hideFlags = HideFlags.HideAndDontSave;

        tempInstance.transform.position = worldPos;
        tempInstance.transform.rotation = rotation;
        tempInstance.transform.localScale = scale;

        Bounds bounds = CalculateRendererBounds(tempInstance);

        // 🧱 Create final collider object
        GameObject go = new GameObject("NavObstacle_" + proto.prefab.name);
        go.transform.localScale = proto.prefab.transform.localScale * .5f; // maintain prefab scale
        go.transform.position = worldPos;

        string name = proto.prefab.name.ToLower();

        if (name.Contains("bush"))
        {
            Object.DestroyImmediate(tempInstance);
            Object.DestroyImmediate(go);
            return null; // skip bushes
        }

        if (name.Contains("tree"))
        {
            CreateTreeCollider(go, bounds);
        }
        else
        {
            // rocks + fallback
            CreateBoxFromBounds(go, bounds);
        }

        Object.DestroyImmediate(tempInstance);

        return go;
    }

    private static Bounds CalculateRendererBounds(GameObject obj)
    {
        Renderer[] renderers = obj.GetComponentsInChildren<Renderer>();

        if (renderers.Length == 0)
            return new Bounds(obj.transform.position, Vector3.one);

        Bounds bounds = renderers[0].bounds;

        for (int i = 1; i < renderers.Length; i++)
        {
            bounds.Encapsulate(renderers[i].bounds);
        }

        return bounds;
    }

    private static void CreateBoxFromBounds(GameObject go, Bounds bounds)
    {
        var box = go.AddComponent<BoxCollider>();

        // ✅ Convert world → local
        box.center = bounds.center - go.transform.position;

        // Optional clamp (prevents weird tiny Y issues)
        Vector3 size = bounds.size;
        size.y = Mathf.Max(size.y, 1f);

        box.size = size;
    }

    private static void CreateTreeCollider(GameObject go, Bounds bounds)
    {
        var capsule = go.AddComponent<CapsuleCollider>();

        float radius = Mathf.Min(bounds.size.x, bounds.size.z) * 0.25f;
        float height = bounds.size.y;

        capsule.radius = radius;
        capsule.height = height;

        // ✅ Convert world → local (THIS WAS BROKEN BEFORE)
        capsule.center = bounds.center - go.transform.position;
    }
}