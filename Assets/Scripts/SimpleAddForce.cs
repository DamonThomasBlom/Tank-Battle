using UnityEngine;

public class SimpleAddForce : MonoBehaviour
{
    public Rigidbody body;
    public float force;

    private void Update()
    {
        body.AddForce(transform.forward * force);
    }
}
