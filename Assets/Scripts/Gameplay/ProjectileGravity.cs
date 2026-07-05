using UnityEngine;

public class ProjectileGravity : MonoBehaviour
{
    [SerializeField] private float gravityScale = 1f;
    [SerializeField] private Vector3 gravity;
    private Rigidbody rb;

    void Awake() => rb = GetComponent<Rigidbody>();

    void FixedUpdate()
    {
        rb.AddForce(gravity, ForceMode.Acceleration);
    }

    public void SetGravityScale(float scale)
    {
        gravityScale = scale;
        gravity = Physics.gravity * gravityScale;
    }
}
