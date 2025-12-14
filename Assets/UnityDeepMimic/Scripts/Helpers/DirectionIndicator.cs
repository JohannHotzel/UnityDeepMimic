using UnityEngine;

public class DirectionIndicator : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Usually the agent's hips transform")]
    public Transform hips;

    [Tooltip("Heading controller providing the desired heading direction")]
    public HeadingController headingController;

    [Header("Visual Settings")]
    public float heightOffset = 0.5f;

    private float m_StartingYPos;

    private void OnEnable()
    {
        m_StartingYPos = transform.position.y;
    }

    private void Update()
    {
        if (hips == null || headingController == null)
            return;

        transform.position = new Vector3(hips.position.x, m_StartingYPos + heightOffset, hips.position.z);

        Vector3 heading = headingController.Heading;
        heading.y = 0f;

        if (heading.sqrMagnitude < 1e-6f)
            return;

        transform.rotation = Quaternion.LookRotation(heading, Vector3.up);
    }

    public void MatchOrientation(Transform t)
    {
        transform.position = new Vector3(t.position.x, m_StartingYPos + heightOffset, t.position.z);
        transform.rotation = t.rotation;
    }
}
