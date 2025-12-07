using UnityEngine;

using UnityEngine;

public class HeadingController : MonoBehaviour
{
    [Header("Random Rotation Settings")]
    [Tooltip("Enable or disable random turning.")]
    public bool randomRotate = true;

    [Tooltip("Minimum turn speed in degrees per second (negative means turning left).")]
    public float minTurnSpeed = -90f;

    [Tooltip("Maximum turn speed in degrees per second.")]
    public float maxTurnSpeed = 90f;

    [Tooltip("How often a new random turn speed is picked (seconds).")]
    public float directionChangeInterval = 3f;

    private float currentAngularSpeed;   // degrees per second
    private float timeToNextChange;


    public Vector3 Heading
    {
        get
        {
            Vector3 h = transform.forward;
            h.y = 0f;
            if (h.sqrMagnitude < 1e-6f)
                return Vector3.forward;

            return h.normalized;
        }
    }

    private void Start()
    {
        PickNewAngularSpeed();
    }

    private void Update()
    {
        if (!randomRotate)
            return;

        // Apply rotation around Y axis
        transform.Rotate(Vector3.up, currentAngularSpeed * Time.deltaTime, Space.World);

        // Countdown until the next random turn speed
        timeToNextChange -= Time.deltaTime;
        if (timeToNextChange <= 0f)
        {
            PickNewAngularSpeed();
        }
    }

    private void PickNewAngularSpeed()
    {
        currentAngularSpeed = Random.Range(minTurnSpeed, maxTurnSpeed);
        timeToNextChange = directionChangeInterval;
    }

    private void OnDrawGizmos()
    {
        Vector3 h = Heading;
        if (h.sqrMagnitude < 1e-6f) return;

        Vector3 start = transform.position;
        Gizmos.DrawLine(start, start + h * 2f);
        Gizmos.DrawSphere(start + h * 2f, 0.05f);
    }
}
