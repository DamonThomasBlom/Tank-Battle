using UnityEngine;

public class AlignToGround : MonoBehaviour
{
    [Header("References")]
    public Transform Visual;

    [Header("Settings")]
    public float RaycastDistance = 3f;
    public LayerMask GroundMask = ~0;
    public float RotationSpeed = 10f;
    public float RaycastOffset = 0.5f;

    void LateUpdate()
    {
        Vector3 origin = transform.position + Vector3.up * RaycastOffset;

        if (!Physics.Raycast(origin, Vector3.down, out RaycastHit hit, RaycastDistance, GroundMask))
            return;

        Vector3 localNormal = transform.InverseTransformDirection(hit.normal);

        float pitch = Mathf.Atan2(localNormal.z, localNormal.y) * Mathf.Rad2Deg;
        float roll = -Mathf.Atan2(localNormal.x, localNormal.y) * Mathf.Rad2Deg;

        Quaternion target = Quaternion.Euler(pitch, 0f, roll);

        Visual.localRotation = Quaternion.Slerp(
            Visual.localRotation,
            target,
            RotationSpeed * Time.deltaTime);
    }
}
