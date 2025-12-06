using UnityEngine;

public class FollowCamera : MonoBehaviour
{
    public Transform target;
    public Vector3 offset = new Vector3(0f, 2f, -4f);
    public float smoothSpeed = 5f;

    void LateUpdate()
    {
        if (target == null) return;

        Vector3 targetPos = target.position + offset;

        transform.position = Vector3.Lerp(transform.position, targetPos, smoothSpeed * Time.deltaTime);
        transform.LookAt(target.position, Vector3.up);

        // Vector3 targetPos = target.position + target.TransformDirection(offset);

    }

    private void OnDrawGizmosSelected()
    {
        if (target == null) return;

        Vector3 camPos = target.position + offset;

        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(camPos, 0.5f);
    }
}
