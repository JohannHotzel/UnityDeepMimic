using UnityEngine;

public class SensorRigTransformer : MonoBehaviour
{
    public Transform hips;

    public void SyncNow()
    {
        if (!hips) return;

        transform.position = hips.position;

        Vector3 fwd = hips.forward;
        fwd.y = 0f;
        if (fwd.sqrMagnitude < 1e-6f) fwd = Vector3.forward;
        fwd.Normalize();

        transform.rotation = Quaternion.LookRotation(fwd, Vector3.up);
    }

    void FixedUpdate()
    {
        SyncNow(); 
    }
}