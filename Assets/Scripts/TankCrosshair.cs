using UnityEngine;

public class TankCrosshair : MonoBehaviour
{
    [Header("References")]
    public Camera cam;                // main camera
    public Transform firePoint;       // where shells are fired from (usually barrel tip)
    public RectTransform crosshairUI; // UI Image or sprite on a canvas
    public float maxDistance = 1000f; // how far ahead to check
    public float minScale = .5f;
    public float maxScale = 1f;
    public LayerMask aimMask;         // what the barrel can "hit" (e.g. ground, enemies)

    void Update()
    {
        if (!cam || !firePoint || !crosshairUI) return;

        Vector3 targetPoint;

        // Raycast to find where barrel is pointing
        if (Physics.Raycast(firePoint.position, firePoint.forward, out RaycastHit hit, maxDistance, aimMask))
        {
            targetPoint = hit.point;
        }
        else
        {
            targetPoint = firePoint.position + firePoint.forward * maxDistance;
        }

        // Convert world point to screen position
        Vector3 screenPoint = cam.WorldToScreenPoint(targetPoint);

        // If target is in front of camera, show crosshair
        if (screenPoint.z > 0f)
        {
            crosshairUI.gameObject.SetActive(true);
            crosshairUI.position = screenPoint;

            float distance = Vector3.Distance(firePoint.position, targetPoint);
            float scale = GetCrossScale(distance);
            crosshairUI.localScale = Vector3.one * scale;

        }
        else
        {
            crosshairUI.gameObject.SetActive(false);
        }
    }

    float GetCrossScale(float hitDistance)
    {
        float t = Mathf.InverseLerp(0f, maxDistance, hitDistance);
        return Mathf.Lerp(maxScale, minScale, t);
    }
}
