using System.Collections;
using UnityEngine;

public class TankFlipper : MonoBehaviour
{
    [Header("Auto-Flip Settings")]
    public float flipCheckInterval = 1f;           // How often to check
    public float upsideDownThreshold = 0.3f;       // 1 = upright, 0 = sideways, -1 = upside down
    public float stuckTimeThreshold = 3f;          // Seconds stuck before auto-flip
    public float resetHeight = 0.5f;               // How high to raise the tank above ground
    public KeyCode manualFlipKey = KeyCode.R;      // Manual flip key
    public LayerMask groundMask = ~0;              // What layers are considered ground

    [Header("Debug")]
    [SerializeField] private bool isUpsideDown = false;
    [SerializeField] private float stuckTimer = 0f;
    [SerializeField] private float timeUntilNextFlip = 0f;

    private Rigidbody rb;
    private float lastCheckTime = 0f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        // Manual flip
        if (Input.GetKeyDown(manualFlipKey))
        {
            FlipTank();
            return;
        }

        // Cooldown check
        if (Time.time < timeUntilNextFlip) return;

        // Periodic check
        if (Time.time - lastCheckTime > flipCheckInterval)
        {
            CheckOrientation();
            lastCheckTime = Time.time;

            if (isUpsideDown)
            {
                stuckTimer += flipCheckInterval;

                if (stuckTimer >= stuckTimeThreshold)
                {
                    FlipTank();
                }
            }
            else
            {
                stuckTimer = 0f;
            }
        }
    }

    void CheckOrientation()
    {
        // Check if tank is upside down using dot product
        float upDot = Vector3.Dot(transform.up, Vector3.up);
        isUpsideDown = upDot < upsideDownThreshold;

        // Optional: Also check if we're moving
        if (isUpsideDown && rb != null)
        {
            // If we're moving, reset the timer (maybe we're doing a cool stunt)
            bool isMoving = rb.linearVelocity.magnitude > 2f || rb.angularVelocity.magnitude > 1f;
            if (isMoving)
            {
                stuckTimer = Mathf.Max(0, stuckTimer - flipCheckInterval);
            }
        }
    }

    void FlipTank()
    {
        Debug.Log("Flipping tank...");

        // Stop all physics movement
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.Sleep(); // Pause physics
        }

        // Find ground position
        Vector3 groundPosition = FindSafeGroundPosition();

        // Set new position (above ground)
        transform.position = groundPosition + Vector3.up * resetHeight;

        // Set rotation upright
        Quaternion targetRotation = GetUprightRotation();
        transform.rotation = targetRotation;

        // Wake up physics
        if (rb != null)
        {
            rb.WakeUp();
        }

        // Reset state
        isUpsideDown = false;
        stuckTimer = 0f;
        timeUntilNextFlip = Time.time + 2f; // 2 second cooldown

        Debug.Log("Tank flipped and repositioned.");
    }

    Vector3 FindSafeGroundPosition()
    {
        // Raycast down from current position to find ground
        if (Physics.Raycast(transform.position + Vector3.up * 10f, Vector3.down, out RaycastHit hit, 20f, groundMask))
        {
            return hit.point;
        }

        // If no ground found, try from higher up
        if (Physics.Raycast(transform.position + Vector3.up * 50f, Vector3.down, out hit, 100f, groundMask))
        {
            return hit.point;
        }

        // Last resort: return current XZ position with y = 0
        return new Vector3(transform.position.x, 0, transform.position.z);
    }

    Quaternion GetUprightRotation()
    {
        // Keep current yaw (horizontal rotation) but make upright
        Vector3 currentEuler = transform.eulerAngles;

        // Options for upright rotation:

        // Option 1: Just make it flat (0, current yaw, 0)
        return Quaternion.Euler(0, currentEuler.y, 0);

        // Option 2: Or align with ground normal if you want to match terrain slope
        // if (Physics.Raycast(transform.position, Vector3.down, out RaycastHit hit, 10f, groundMask))
        // {
        //     // Align with ground normal
        //     return Quaternion.FromToRotation(transform.up, hit.normal) * transform.rotation;
        // }
        // else
        // {
        //     return Quaternion.Euler(0, currentEuler.y, 0);
        // }
    }

    // Optional: Visual feedback before flip
    IEnumerator FlashWarningBeforeFlip()
    {
        Debug.LogWarning("Tank stuck! Auto-flip in 2 seconds...");

        // You could add visual/audio warnings here
        // Flash lights, play sound, etc.

        yield return new WaitForSeconds(2f);

        if (isUpsideDown && stuckTimer >= stuckTimeThreshold)
        {
            FlipTank();
        }
    }

    // Optional: Call this to manually check and flip from other scripts
    public void TryFlipTank()
    {
        CheckOrientation();
        if (isUpsideDown)
        {
            FlipTank();
        }
    }

    // Draw gizmos for debugging
    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying) return;

        // Draw orientation indicator
        Gizmos.color = isUpsideDown ? Color.red : Color.green;
        Gizmos.DrawLine(transform.position, transform.position + transform.up * 3f);

        // Draw ground check ray
        Gizmos.color = Color.yellow;
        Vector3 groundPos = FindSafeGroundPosition();
        Gizmos.DrawLine(transform.position + Vector3.up * 10f, groundPos);
        Gizmos.DrawSphere(groundPos, 0.5f);
    }
}
