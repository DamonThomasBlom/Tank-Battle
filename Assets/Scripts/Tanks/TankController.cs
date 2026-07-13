using UnityEngine;

public class TankController : MonoBehaviour
{
    public TankCannon Cannon;
    public LayerMask AimMask;

    public float CameraRaycastYOffset;

    [Header("Input")]
    public KeyCode fireKey = KeyCode.Mouse0;

    Team _team;
    Camera _camera;
    float _lookDistance = 500f;


    void Awake()
    {
        _team = GetComponentInParent<Team>();
        _camera = Camera.main;
    }

    void Update()
    {
        if (!GameManager.Instance.CanPlayerMove()) return;
        AimTurretAndBarrel();
        HandleShooting();
    }

    // === AIMING ===
    void AimTurretAndBarrel()
    {
        Ray ray = new Ray(_camera.transform.position, _camera.transform.forward);

        Vector3 targetPoint;

        if (Physics.Raycast(ray, out RaycastHit hit, _lookDistance, AimMask))
        {
            targetPoint = hit.point;
        }
        else
        {
            targetPoint = ray.origin + ray.direction * _lookDistance;
        }

        targetPoint.y += CameraRaycastYOffset;

        Cannon.SetAimPoint(targetPoint);
    }

    // === SHOOTING ===

    void HandleShooting()
    {
        if (Input.GetKey(fireKey))
        {
            Cannon.Fire();
        }
    }
}
