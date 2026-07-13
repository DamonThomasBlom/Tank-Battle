using UnityEngine;

public class WaypointUI : MonoBehaviour
{
    [Header("References")]
    public Transform Target;
    public RectTransform Icon;
    public RectTransform Arrow;

    [Header("Settings")]
    public Camera TargetCamera;
    public float ScreenEdgePadding = 40f;
    public float IconOffsetFromArrow = 40f;
    public Vector3 WorldOffset = Vector3.up * 3f;

    private RectTransform rectTransform;
    private Canvas canvas;

    private void Awake()
    {
        rectTransform = GetComponent<RectTransform>();
        canvas = GetComponentInParent<Canvas>();

        if (TargetCamera == null)
            TargetCamera = Camera.main;
    }

    private void LateUpdate()
    {
        if (Target == null || TargetCamera == null)
            return;

        Vector3 screenPos = TargetCamera.WorldToScreenPoint(Target.position + WorldOffset);

        bool behindCamera = screenPos.z < 0f;

        if (behindCamera)
        {
            screenPos.x = Screen.width - screenPos.x;
            screenPos.y = Screen.height - screenPos.y;
        }

        bool onScreen =
            !behindCamera &&
            screenPos.x >= 0 &&
            screenPos.x <= Screen.width &&
            screenPos.y >= 0 &&
            screenPos.y <= Screen.height;

        RectTransform canvasRect = canvas.transform as RectTransform;

        if (onScreen)
        {
            Arrow.gameObject.SetActive(false);

            RectTransformUtility.ScreenPointToLocalPointInRectangle(
                canvasRect,
                screenPos,
                canvas.renderMode == RenderMode.ScreenSpaceOverlay ? null : TargetCamera,
                out Vector2 localPos);

            rectTransform.localPosition = localPos;

            // Keep icon centered when on-screen
            Icon.localPosition = Vector3.zero;
        }
        else
        {
            Arrow.gameObject.SetActive(true);

            Vector2 screenCenter = new(Screen.width * 0.5f, Screen.height * 0.5f);
            Vector2 direction = ((Vector2)screenPos - screenCenter).normalized;

            // Arrow stays near the edge
            Vector2 arrowScreenPos = screenCenter + new Vector2(
                direction.x * ((Screen.width * 0.5f) - ScreenEdgePadding),
                direction.y * ((Screen.height * 0.5f) - ScreenEdgePadding));

            arrowScreenPos.x = Mathf.Clamp(
                arrowScreenPos.x,
                ScreenEdgePadding,
                Screen.width - ScreenEdgePadding);

            arrowScreenPos.y = Mathf.Clamp(
                arrowScreenPos.y,
                ScreenEdgePadding,
                Screen.height - ScreenEdgePadding);

            // Convert arrow position to canvas space
            RectTransformUtility.ScreenPointToLocalPointInRectangle(
                canvasRect,
                arrowScreenPos,
                canvas.renderMode == RenderMode.ScreenSpaceOverlay ? null : TargetCamera,
                out Vector2 arrowLocalPos);

            // Parent sits where the arrow is
            rectTransform.localPosition = arrowLocalPos;

            // Pull icon inward
            Icon.localPosition = -direction * IconOffsetFromArrow;

            // Arrow stays at the parent origin
            Arrow.localPosition = Vector3.zero;

            // Rotate arrow
            float angle = Mathf.Atan2(direction.y, direction.x) * Mathf.Rad2Deg;
            Arrow.localRotation = Quaternion.Euler(0f, 0f, angle - 90f);
        }
    }
}