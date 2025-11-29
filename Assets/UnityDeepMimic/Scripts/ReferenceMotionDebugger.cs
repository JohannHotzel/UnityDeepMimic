using System.Collections.Generic;
using UnityEngine;

public class ReferenceMotionDebugger : MonoBehaviour
{
    public ReferenceMotionSampler sampler;
    public Transform targetRoot;
    public List<Transform> targetBones = new List<Transform>();

    [Range(0f, 1f)]
    public float phase = 0f;
    private float lastPhase = -1f;

    private void OnValidate()
    {
        lastPhase = -1f;
    }

    private void Update()
    {
        if (sampler == null || sampler.clip == null || sampler.animator == null || sampler.rootBone == null)
            return;

        if (Mathf.Approximately(phase, lastPhase))
            return;

        ApplyPhaseToTargets(phase);
        lastPhase = phase;
    }

    public void ApplyPhaseToTargets(float phi)
    {
        var features = sampler.SampleAndExtract(phi, out Vector3 com);
        if (features == null || features.Count == 0)
            return;

        int count = Mathf.Min(targetBones.Count, features.Count);

        for (int i = 0; i < count; i++)
        {
            Transform t = targetBones[i];
            if (t == null)
                continue;

            var f = features[i];

          //  t.position = targetRoot.TransformPoint(f.localPos);
            t.rotation = targetRoot.rotation * f.localRot;
        }
    }
}
