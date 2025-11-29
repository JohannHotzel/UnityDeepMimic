using UnityEngine;
using System.Collections.Generic;


public class ReferenceJointDebugger : MonoBehaviour
{
    public ReferenceMotionSampler sampler;
    public Transform targetRoot;
    public ConfigurableJoint[] joints;

    [Range(0f, 1f)]
    public float phase = 0f;
    private float lastPhase = -1f;

    private Quaternion[] startLocalRotations;

    private void OnValidate()
    {
        lastPhase = -1f;
    }

    private void Start()
    {
        if (sampler == null || sampler.clip == null || sampler.animator == null || sampler.rootBone == null)
        {
            Debug.LogError("ReferenceJointDebugger: Sampler/Clip/Animator/RootBone nicht korrekt gesetzt.", this);
            enabled = false;
            return;
        }

        if (targetRoot == null)
        {
            Debug.LogError("ReferenceJointDebugger: targetRoot nicht gesetzt.", this);
            enabled = false;
            return;
        }

        if (joints == null || joints.Length == 0)
        {
            Debug.LogError("ReferenceJointDebugger: Keine Joints zugewiesen.", this);
            enabled = false;
            return;
        }

        startLocalRotations = new Quaternion[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] == null)
            {
                Debug.LogError($"ReferenceJointDebugger: Joint an Index {i} ist null.", this);
                enabled = false;
                return;
            }

            startLocalRotations[i] = joints[i].transform.localRotation;
        }
    }

    private void Update()
    {
        if (!enabled)
            return;

        if (Mathf.Approximately(phase, lastPhase))
            return;

        ApplyPhaseToJoints(phase);
        lastPhase = phase;
    }

    public void ApplyPhaseToJoints(float phi)
    {
        var features = sampler.SampleAndExtract(phi, out Vector3 com);
        if (features == null || features.Count == 0)
            return;

        int count = Mathf.Min(joints.Length, features.Count);

        for (int i = 0; i < count; i++)
        {
            var joint = joints[i];
            if (joint == null)
                continue;

            var f = features[i + 1];


            Quaternion targetWorldRotation = targetRoot.rotation * f.localRot;
            ApplyWorldRotationToJoint(joint, targetWorldRotation, startLocalRotations[i]);
        }
    }

    private void ApplyWorldRotationToJoint(ConfigurableJoint joint, Quaternion targetWorldRotation, Quaternion startLocalRotation)
    {
        Quaternion targetLocalRotation = Quaternion.Inverse(joint.transform.parent.rotation) * targetWorldRotation;
     //   joint.SetTargetRotationLocal(targetLocalRotation, startLocalRotation);
    }
}

