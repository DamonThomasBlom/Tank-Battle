using UnityEngine;

public class ProjectileGravity : MonoBehaviour
{
    public float gravityScale = 1f;
    private Rigidbody rb;

    void Awake() => rb = GetComponent<Rigidbody>();

    void FixedUpdate()
    {
        rb.AddForce(Physics.gravity * gravityScale, ForceMode.Acceleration);
    }
}
