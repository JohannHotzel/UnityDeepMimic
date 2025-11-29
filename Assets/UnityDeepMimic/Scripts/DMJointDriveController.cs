using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;
using UnityEngine.Serialization;


public class DMBodyPart
{
    [Header("Body Part Info")]
    [Space(10)]
    public ConfigurableJoint joint;
    public Rigidbody rb;
    [HideInInspector] public Vector3 startingPos;
    [HideInInspector] public Quaternion startingRot;
    [HideInInspector] public Quaternion startingLocalRot;

    [Header("Ground & Target Contact")]
    [Space(10)]
    public GroundContact groundContact;
    public TargetContact targetContact;

    [FormerlySerializedAs("thisJDController")]
    [HideInInspector] public DMJointDriveController thisJdController;

    [Header("Current Joint Settings")]
    [Space(10)]
    public Vector3 currentEularJointRotation;

    [HideInInspector] public float currentStrength;
    public float currentXNormalizedRot;
    public float currentYNormalizedRot;
    public float currentZNormalizedRot;

    [Header("Other Debug Info")]
    [Space(10)]
    public Vector3 currentJointForce;
    public float currentJointForceSqrMag;
    public Vector3 currentJointTorque;
    public float currentJointTorqueSqrMag;
    public AnimationCurve jointForceCurve = new AnimationCurve();
    public AnimationCurve jointTorqueCurve = new AnimationCurve();

    /// <summary>
    /// Reset body part to initial configuration.
    /// </summary>
    public void Reset()
    {
        rb.transform.position = startingPos;
        rb.transform.rotation = startingRot;
        rb.transform.localRotation = startingLocalRot;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        if (groundContact) groundContact.touchingGround = false;
        if (targetContact) targetContact.touchingTarget = false;
    }

    /// <summary>
    /// Apply torque according to defined goal `x, y, z` angle and force `strength`.
    /// </summary>
    public void SetJointTargetRotation(float x, float y, float z)
    {
        if (joint == null || thisJdController == null || thisJdController.root == null)
            return;

        // Actions [-1,1] -> [0,1]
        x = (x + 1f) * 0.5f;
        y = (y + 1f) * 0.5f;
        z = (z + 1f) * 0.5f;

        // [0,1] -> [-180,180] (Rootspace-Winkel)
        float xDeg = Mathf.Lerp(-180f, 180f, x);
        float yDeg = Mathf.Lerp(-180f, 180f, y);
        float zDeg = Mathf.Lerp(-180f, 180f, z);

        Quaternion targetRootSpaceRot = Quaternion.Euler(xDeg, yDeg, zDeg);

        Quaternion targetWorldRotation = thisJdController.root.rotation * targetRootSpaceRot;

        Transform parent = joint.transform.parent;

        Quaternion targetLocalRotation = Quaternion.Inverse(parent.rotation) * targetWorldRotation;
        joint.SetTargetRotationLocal(targetLocalRotation, startingLocalRot);

        currentXNormalizedRot = Mathf.InverseLerp(-180f, 180f, xDeg);
        currentYNormalizedRot = Mathf.InverseLerp(-180f, 180f, yDeg);
        currentZNormalizedRot = Mathf.InverseLerp(-180f, 180f, zDeg);

        currentEularJointRotation = new Vector3(xDeg, yDeg, zDeg);
    }
    public void SetJointTargetRotationLocal(float x, float y, float z)
    {
        x = (x + 1f) * 0.5f;
        y = (y + 1f) * 0.5f;
        z = (z + 1f) * 0.5f;

        var xRot = Mathf.Lerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, x);
        var yRot = Mathf.Lerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, y);
        var zRot = Mathf.Lerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, z);

        currentXNormalizedRot = Mathf.InverseLerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, xRot);
        currentYNormalizedRot = Mathf.InverseLerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, yRot);
        currentZNormalizedRot = Mathf.InverseLerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, zRot);

        joint.targetRotation = Quaternion.Euler(xRot, yRot, zRot);
        currentEularJointRotation = new Vector3(xRot, yRot, zRot);
    }
    public void SetJointTargetRotation(Quaternion rootSpaceRotation)
    {
        if (joint == null || thisJdController == null || thisJdController.root == null)
            return;

        Quaternion targetWorldRotation = thisJdController.root.rotation * rootSpaceRotation;
        Quaternion targetLocalRotation = Quaternion.Inverse(joint.transform.parent.rotation) * targetWorldRotation;
        joint.SetTargetRotationLocal(targetLocalRotation, startingLocalRot);
    }

    public void SetJointStrength(float strength)
    {
        var rawVal = (strength + 1f) * 0.5f * thisJdController.maxJointForceLimit;
        var jd = new JointDrive
        {
            positionSpring = thisJdController.maxJointSpring,
            positionDamper = thisJdController.jointDampen,
            maximumForce = rawVal
        };
        joint.slerpDrive = jd;
        currentStrength = jd.maximumForce;
    }
    public void SetJointStrengthConstant(float strength01)
    {
        var rawVal = Mathf.Clamp01(strength01) * thisJdController.maxJointForceLimit;
        var jd = new JointDrive
        {
            positionSpring = thisJdController.maxJointSpring,
            positionDamper = thisJdController.jointDampen,
            maximumForce = rawVal
        };
        joint.slerpDrive = jd;
        currentStrength = jd.maximumForce;
    }

}



public class DMJointDriveController : MonoBehaviour
{
    [Header("Joint Drive Settings")]
    [Space(10)]
    public Transform root;
    public float maxJointSpring;
    public float jointDampen;
    public float maxJointForceLimit;

    [HideInInspector] public Dictionary<Transform, DMBodyPart> bodyPartsDict = new Dictionary<Transform, DMBodyPart>();

    [HideInInspector] public List<DMBodyPart> bodyPartsList = new List<DMBodyPart>();
    const float k_MaxAngularVelocity = 50.0f;

    /// <summary>
    /// Create BodyPart object and add it to dictionary.
    /// </summary>
    public void SetupBodyPart(Transform t)
    {
        var bp = new DMBodyPart
        {
            rb = t.GetComponent<Rigidbody>(),
            joint = t.GetComponent<ConfigurableJoint>(),
            startingPos = t.position,
            startingRot = t.rotation,
            startingLocalRot = t.localRotation
        };
        bp.rb.maxAngularVelocity = k_MaxAngularVelocity;

        // Add & setup the ground contact script
        bp.groundContact = t.GetComponent<GroundContact>();
        if (!bp.groundContact)
        {
            bp.groundContact = t.gameObject.AddComponent<GroundContact>();
            bp.groundContact.agent = gameObject.GetComponent<Agent>();
        }
        else
        {
            bp.groundContact.agent = gameObject.GetComponent<Agent>();
        }

        if (bp.joint)
        {
            var jd = new JointDrive
            {
                positionSpring = maxJointSpring,
                positionDamper = jointDampen,
                maximumForce = maxJointForceLimit
            };
            bp.joint.slerpDrive = jd;
        }

        bp.thisJdController = this;
        bodyPartsDict.Add(t, bp);
        bodyPartsList.Add(bp);
    }

    public void GetCurrentJointForces()
    {
        foreach (var bodyPart in bodyPartsDict.Values)
        {
            if (bodyPart.joint)
            {
                bodyPart.currentJointForce = bodyPart.joint.currentForce;
                bodyPart.currentJointForceSqrMag = bodyPart.joint.currentForce.magnitude;
                bodyPart.currentJointTorque = bodyPart.joint.currentTorque;
                bodyPart.currentJointTorqueSqrMag = bodyPart.joint.currentTorque.magnitude;
                if (Application.isEditor)
                {
                    if (bodyPart.jointForceCurve.length > 1000)
                    {
                        bodyPart.jointForceCurve = new AnimationCurve();
                    }

                    if (bodyPart.jointTorqueCurve.length > 1000)
                    {
                        bodyPart.jointTorqueCurve = new AnimationCurve();
                    }

                    bodyPart.jointForceCurve.AddKey(Time.time, bodyPart.currentJointForceSqrMag);
                    bodyPart.jointTorqueCurve.AddKey(Time.time, bodyPart.currentJointTorqueSqrMag);
                }
            }
        }
    }
}






public static class ConfigurableJointExtensions
{
    /// <summary>
    /// Sets a joint's targetRotation to match a given local rotation.
    /// The joint transform's local rotation must be cached on Start and passed into this method.
    /// </summary>
    public static void SetTargetRotationLocal(this ConfigurableJoint joint, Quaternion targetLocalRotation, Quaternion startLocalRotation)
    {
        if (joint.configuredInWorldSpace)
        {
            Debug.LogError("SetTargetRotationLocal should not be used with joints that are configured in world space. For world space joints, use SetTargetRotation.", joint);
        }
        SetTargetRotationInternal(joint, targetLocalRotation, startLocalRotation, Space.Self);
    }

    /// <summary>
    /// Sets a joint's targetRotation to match a given world rotation.
    /// The joint transform's world rotation must be cached on Start and passed into this method.
    /// </summary>
    public static void SetTargetRotation(this ConfigurableJoint joint, Quaternion targetWorldRotation, Quaternion startWorldRotation)
    {
        if (!joint.configuredInWorldSpace)
        {
            Debug.LogError("SetTargetRotation must be used with joints that are configured in world space. For local space joints, use SetTargetRotationLocal.", joint);
        }
        SetTargetRotationInternal(joint, targetWorldRotation, startWorldRotation, Space.World);
    }

    static void SetTargetRotationInternal(ConfigurableJoint joint, Quaternion targetRotation, Quaternion startRotation, Space space)
    {
        // Calculate the rotation expressed by the joint's axis and secondary axis
        var right = joint.axis;
        var forward = Vector3.Cross(joint.axis, joint.secondaryAxis).normalized;
        var up = Vector3.Cross(forward, right).normalized;
        Quaternion worldToJointSpace = Quaternion.LookRotation(forward, up);

        // Transform into world space
        Quaternion resultRotation = Quaternion.Inverse(worldToJointSpace);

        // Counter-rotate and apply the new local rotation.
        // Joint space is the inverse of world space, so we need to invert our value
        if (space == Space.World)
        {
            resultRotation *= startRotation * Quaternion.Inverse(targetRotation);
        }
        else
        {
            resultRotation *= Quaternion.Inverse(targetRotation) * startRotation;
        }

        // Transform back into joint space
        resultRotation *= worldToJointSpace;

        // Set target rotation to our newly calculated rotation
        joint.targetRotation = resultRotation;
    }
}
