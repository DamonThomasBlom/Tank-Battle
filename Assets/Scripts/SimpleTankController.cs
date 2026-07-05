using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class SimpleTankController : MonoBehaviour
{
    [Header("Movement")]
    public float moveSpeed = 10f;
    public float turnSpeed = 120f;
    public float acceleration = 8f;
    public float deceleration = 10f;

    [Header("Grounding")]
    public float groundCheckDistance = 1.5f;
    public LayerMask groundMask;

    [Header("Ground Sampling")]
    public List<Transform> groundPoints = new List<Transform>();

    Rigidbody rb;
    Vector3 smoothedNormal = Vector3.up;
    float currentSpeed;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.interpolation = RigidbodyInterpolation.Interpolate;

        rb.angularDamping = 0.1f;
        rb.linearDamping = 0.1f;
    }

    void FixedUpdate()
    {
        if (!GameManager.Instance.CanPlayerMove()) return;
        if (!GetGroundNormal(out Vector3 groundNormal))
            groundNormal = Vector3.up;

        smoothedNormal = Vector3.Slerp(smoothedNormal, groundNormal, 2f * Time.fixedDeltaTime);
        HandleMovement(smoothedNormal);
        HandleRotation(smoothedNormal, Input.GetAxis("Horizontal"), Input.GetAxis("Vertical"));
    }

    void HandleMovement(Vector3 groundNormal)
    {
        float moveInput = Input.GetAxis("Vertical");
        float turnInput = Input.GetAxis("Horizontal");

        // Movement direction projected onto ground
        Vector3 forward = Vector3.ProjectOnPlane(transform.forward, groundNormal).normalized;

        // Smooth speed
        float targetSpeed = moveInput * moveSpeed;

        if (Mathf.Abs(moveInput) > 0.01f)
            currentSpeed = Mathf.Lerp(currentSpeed, targetSpeed, acceleration * Time.fixedDeltaTime);
        else
            currentSpeed = Mathf.Lerp(currentSpeed, 0f, deceleration * Time.fixedDeltaTime);

        // Apply velocity (respecting slope)
        Vector3 velocity = forward * currentSpeed;
        rb.linearVelocity = new Vector3(velocity.x, rb.linearVelocity.y, velocity.z);
    }

    void HandleRotation(Vector3 groundNormal, float turnInput, float moveInput)
    {
        float turn = turnInput * turnSpeed;

        if (moveInput < 0)
            turn *= -1f;

        Quaternion yawRotation = Quaternion.AngleAxis(turn * Time.fixedDeltaTime, groundNormal);

        Quaternion targetRotation = rb.rotation * yawRotation;

        // Align to ground
        Quaternion alignToGround = Quaternion.FromToRotation(targetRotation * Vector3.up, groundNormal);
        targetRotation = alignToGround * targetRotation;

        // Smooth it
        rb.MoveRotation(Quaternion.Slerp(rb.rotation, targetRotation, 8f * Time.fixedDeltaTime));
    }

    //void StickToGround()
    //{
    //    if (GetGroundNormal(out Vector3 groundNormal))
    //    {
    //        Quaternion targetRot = Quaternion.FromToRotation(transform.up, groundNormal) * transform.rotation;

    //        rb.MoveRotation(Quaternion.Slerp(rb.rotation, targetRot, 10f * Time.fixedDeltaTime));
    //    }
    //}

    bool GetGroundNormal(out Vector3 normal)
    {
        Vector3 startPoint = new Vector3(transform.position.x, transform.position.y + 0.25f, transform.position.z);
        if (Physics.Raycast(startPoint, Vector3.down, out RaycastHit hit, groundCheckDistance, groundMask))
        {
            normal = hit.normal;
            return true;
        }

        normal = Vector3.up;
        return false;
    }
}