using UnityEngine;

public class TankCameraController : MonoBehaviour
{
    [Header("References")]
    public Camera cam;
    public Transform target;
    public Transform camPivot;

    [Header("Orbit Settings")]
    public float camDistance = 12f;
    public float camMinDistance = 4f;
    public float camMaxDistance = 22f;
    public float camSensitivity = 2.5f;
    public float camPitchMin = -45f;
    public float camPitchMax = 20f;

    [Header("Pivot Height")]
    public float pivotMinHeight = 1.5f;
    public float pivotMaxHeight = 4f;
    public float pivotLerpSpeed = 10f;

    [Header("Zoom")]
    public float zoomDistance = 6f;
    public float zoomFOV = 40f;
    public float normalFOV = 60f;
    public float zoomLerpSpeed = 10f;

    [Header("Collision")]
    public float camCollisionRadius = 0.4f;
    public LayerMask collisionMask;

    [Header("Follow")]
    public float camFollowLerp = 12f;

    float yaw;
    float pitch;
    float targetDistance;
    float targetFOV;

    void Awake()
    {
        Cursor.lockState = CursorLockMode.Locked;

        targetDistance = camDistance;
        targetFOV = cam.fieldOfView;

        if (!cam && Camera.main)
            cam = Camera.main;

        if (!camPivot && target)
        {
            GameObject pivot = new GameObject("CamPivot");
            camPivot = pivot.transform;
            camPivot.SetParent(target);
            camPivot.localPosition = new Vector3(0f, 2f, 0f);
        }

        if (cam && camPivot)
        {
            Vector3 toCam = cam.transform.position - camPivot.position;
            Vector3 flat = new Vector3(toCam.x, 0, toCam.z);

            yaw = Mathf.Atan2(flat.x, flat.z) * Mathf.Rad2Deg;
            pitch = Mathf.Clamp(Vector3.SignedAngle(flat, toCam, Vector3.right), camPitchMin, camPitchMax);
        }
    }

    void Update()
    {
        HandleOrbit();
    }

    void LateUpdate()
    {
        UpdatePivotHeight();
        UpdateCameraPosition();
    }

    void HandleOrbit()
    {
        if (!cam) return;

        bool isZooming = Input.GetMouseButton(1);
        float sensitivityMultiplier = isZooming ? 0.5f : 1f;

        yaw += Input.GetAxis("Mouse X") * camSensitivity * sensitivityMultiplier;
        pitch -= Input.GetAxis("Mouse Y") * camSensitivity * sensitivityMultiplier;
        pitch = Mathf.Clamp(pitch, camPitchMin, camPitchMax);

        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.001f)
        {
            targetDistance = Mathf.Clamp(targetDistance - scroll * 5f, camMinDistance, camMaxDistance);
        }

        float desiredDistance = isZooming ? zoomDistance : targetDistance;
        float desiredFOV = isZooming ? zoomFOV : normalFOV;

        // Smooth transitions
        camDistance = Mathf.Lerp(camDistance, desiredDistance, Time.deltaTime * zoomLerpSpeed);
        cam.fieldOfView = Mathf.Lerp(cam.fieldOfView, desiredFOV, Time.deltaTime * zoomLerpSpeed);
    }

    void UpdateCameraPosition()
    {
        if (!cam || !camPivot) return;

        Quaternion rot = Quaternion.Euler(pitch, yaw, 0f);
        Vector3 direction = rot * Vector3.back;

        Vector3 targetPos = camPivot.position;
        Vector3 desiredPos = targetPos + direction * camDistance;

        if (Physics.SphereCast(targetPos, camCollisionRadius, (desiredPos - targetPos).normalized,
            out RaycastHit hit, Vector3.Distance(targetPos, desiredPos), collisionMask, QueryTriggerInteraction.Ignore))
        {
            desiredPos = hit.point + hit.normal * camCollisionRadius;
        }

        Vector3 finalPos = desiredPos;
        Quaternion finalRot = rot;

        if (CameraShake.Instance != null)
        {
            finalPos += rot * CameraShake.Instance.PositionOffset;
            finalRot *= CameraShake.Instance.RotationOffset;
        }

        cam.transform.position = Vector3.Lerp(
            cam.transform.position,
            finalPos,
            Time.deltaTime * camFollowLerp);

        cam.transform.rotation = Quaternion.Slerp(
            cam.transform.rotation,
            finalRot,
            Time.deltaTime * camFollowLerp);

        //cam.transform.position = Vector3.Lerp(cam.transform.position, desiredPos, Time.deltaTime * camFollowLerp);
        //cam.transform.rotation = Quaternion.Slerp(cam.transform.rotation, rot, Time.deltaTime * camFollowLerp);
    }

    void UpdatePivotHeight()
    {
        if (!camPivot) return;

        float t = Mathf.InverseLerp(camMinDistance, camMaxDistance, camDistance);
        float targetHeight = Mathf.Lerp(pivotMinHeight, pivotMaxHeight, t);

        Vector3 localPos = camPivot.localPosition;
        localPos.y = Mathf.Lerp(localPos.y, targetHeight, Time.deltaTime * pivotLerpSpeed);
        camPivot.localPosition = localPos;
    }
}