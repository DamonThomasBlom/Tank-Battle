using System.Collections.Generic;
using UnityEngine;

public class TankRangeReticle : MonoBehaviour
{
    [Header("References")]
    public Camera cam;
    public Transform firePoint;
    public TankCannon Cannon;

    [Header("UI")]
    public RectTransform tickPrefab;
    public RectTransform tickParent;

    [Header("Settings")]
    public int maxRange = 800;
    public int step = 100;

    private readonly List<RectTransform> ticks = new();

    private Vector3 lastAimDir;
    private float lastMuzzleVelocity;

    private void Start()
    {
        BuildTicks();
        RebuildReticle();
    }

    // Call this when turret rotates or weapon changes
    public void RebuildReticle()
    {
        if (Cannon == null || firePoint == null || cam == null) return;

        Vector3 aimDir = firePoint.forward;

        // avoid unnecessary rebuilds
        if (aimDir == lastAimDir && Mathf.Approximately(lastMuzzleVelocity, Cannon.muzzleVelocity))
            return;

        lastAimDir = aimDir;
        lastMuzzleVelocity = Cannon.muzzleVelocity;

        UpdateReticle();
    }

    private void BuildTicks()
    {
        int count = maxRange / step;

        for (int i = 0; i < count; i++)
        {
            var tick = Instantiate(tickPrefab, tickParent);
            ticks.Add(tick);
        }
    }

    private void UpdateReticle()
    {
        Vector3 origin = firePoint.position;
        Vector3 dir = firePoint.forward;

        float v = Cannon.muzzleVelocity;
        float g = Mathf.Abs(Physics.gravity.y) * Cannon.gravityScale;
        if (g < 0.001f) g = 0.001f;

        int index = 0;

        for (int range = step; range <= maxRange; range += step)
        {
            if (index >= ticks.Count) break;

            Vector3 worldPoint = GetBallisticPoint(origin, dir, range, v, g);
            Vector3 screenPoint = cam.WorldToScreenPoint(worldPoint);

            var tick = ticks[index];

            tick.position = screenPoint;
            tick.gameObject.SetActive(screenPoint.z > 0);

            index++;
        }
    }

    private Vector3 GetBallisticPoint(Vector3 origin, Vector3 dir, float distance, float muzzleVelocity, float gravity)
    {
        // time based on horizontal travel
        float t = distance / muzzleVelocity;

        Vector3 horizontal = dir.normalized * distance;
        float drop = 0.5f * gravity * t * t;

        return origin + horizontal + Vector3.down * drop;
    }

    // Optional: call this from turret script
    public void ForceRebuild()
    {
        RebuildReticle();
    }
}