using UnityEngine;

public class BillboardUI : MonoBehaviour
{
    private Transform camTransform;

    void Start()
    {
        // Cache the main camera transform for better performance
        camTransform = Camera.main.transform;
    }

    void LateUpdate()
    {
        Vector3 lookAtPos = camTransform.position;
        lookAtPos.y = transform.position.y; // Lock vertical axis
        transform.LookAt(lookAtPos);
    }
}
