using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;
using UnityEngine.Serialization;


public class DMBodyPart
{
    [Header("Body Part Info")]
    [Space(10)]
    public ArticulationBody joint;
    public Vector3 startingPos;
    public Quaternion startingRot;
    public Quaternion startingLocalRot;

    [Header("Ground & Target Contact")]
    [Space(10)]
    public GroundContact groundContact;
    public TargetContact targetContact;

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
        var t = joint.transform;

        t.position = startingPos;
        t.rotation = startingRot;
        t.localRotation = startingLocalRot;

        joint.linearVelocity = Vector3.zero;
        joint.angularVelocity = Vector3.zero;

        if (groundContact) groundContact.touchingGround = false;
        if (targetContact) targetContact.touchingTarget = false;
    }


    public void SetJointTargetRotation(float x, float y, float z)
    {
        if (joint == null)
            return;

        switch (joint.jointType)
        {
            case ArticulationJointType.RevoluteJoint:
                {
                    var drive = joint.xDrive;

                    if (Mathf.Approximately(drive.lowerLimit, drive.upperLimit))
                        return;

                   
                    float t01 = (x + 1f) * 0.5f;
          
                    float target = Mathf.Lerp(drive.lowerLimit, drive.upperLimit, t01);

                    drive.target = target;
                    joint.xDrive = drive;

                    currentXNormalizedRot = Mathf.InverseLerp(drive.lowerLimit, drive.upperLimit, target);
                    currentEularJointRotation = new Vector3(target, 0f, 0f);
                    break;
                }

case ArticulationJointType.SphericalJoint:
{
    // Twist (X)
    var xDrive = joint.xDrive;
    if (!Mathf.Approximately(xDrive.lowerLimit, xDrive.upperLimit))
    {
        float t = (x + 1f) * 0.5f;
        float target = Mathf.Lerp(xDrive.lowerLimit, xDrive.upperLimit, t);
        xDrive.target = target;
        joint.xDrive = xDrive;

        currentXNormalizedRot = Mathf.InverseLerp(xDrive.lowerLimit, xDrive.upperLimit, target);
        currentEularJointRotation.x = target;
    }

    // Swing Y
    var yDrive = joint.yDrive;
    if (!Mathf.Approximately(yDrive.lowerLimit, yDrive.upperLimit))
    {
        float t = (y + 1f) * 0.5f;
        float target = Mathf.Lerp(yDrive.lowerLimit, yDrive.upperLimit, t);
        yDrive.target = target;
        joint.yDrive = yDrive;

        currentYNormalizedRot = Mathf.InverseLerp(yDrive.lowerLimit, yDrive.upperLimit, target);
        currentEularJointRotation.y = target;
    }

    // Swing Z
    var zDrive = joint.zDrive;
    if (!Mathf.Approximately(zDrive.lowerLimit, zDrive.upperLimit))
    {
        float t = (z + 1f) * 0.5f;
        float target = Mathf.Lerp(zDrive.lowerLimit, zDrive.upperLimit, t);
        zDrive.target = target;
        joint.zDrive = zDrive;

        currentZNormalizedRot = Mathf.InverseLerp(zDrive.lowerLimit, zDrive.upperLimit, target);
        currentEularJointRotation.z = target;
    }
    break;
}

            case ArticulationJointType.FixedJoint:
            default:

                break;
        }
    }

    public void SetJointStrengthConstant(float strength01)
    {
        float forceLimit = Mathf.Clamp01(strength01) * thisJdController.maxJointForceLimit;



        ArticulationDrive xDrive = joint.xDrive;
        xDrive.driveType = ArticulationDriveType.Force;
        xDrive.stiffness = thisJdController.maxJointSpring;
        xDrive.damping = thisJdController.jointDampen;
        xDrive.forceLimit = forceLimit;
        joint.xDrive = xDrive;

        ArticulationDrive yDrive = joint.yDrive;
        yDrive.driveType = ArticulationDriveType.Force;
        yDrive.stiffness = thisJdController.maxJointSpring;
        yDrive.damping = thisJdController.jointDampen;
        yDrive.forceLimit = forceLimit;
        joint.yDrive = yDrive;

        ArticulationDrive zDrive = joint.zDrive;
        zDrive.driveType = ArticulationDriveType.Force;
        zDrive.stiffness = thisJdController.maxJointSpring;
        zDrive.damping = thisJdController.jointDampen;
        zDrive.forceLimit = forceLimit;
        joint.zDrive = zDrive;

        currentStrength = forceLimit;
    }

}



public class DMJointDriveController : MonoBehaviour
{
    [Header("Joint Drive Settings")]
    [Space(10)]
    public Transform root;
    public float maxJointSpring = 1000f;
    public float jointDampen = 50f;
    public float maxJointForceLimit = 500f;

    [HideInInspector] public Dictionary<Transform, DMBodyPart> bodyPartsDict = new();
    [HideInInspector] public List<DMBodyPart> bodyPartsList = new();

    const float k_MaxAngularVelocity = 50.0f;



    public void SetupBodyPart(Transform t)
    {
        var ab = t.GetComponent<ArticulationBody>();
        if (ab == null)
        {
            Debug.LogError($"No ArticulationBody found on {t.name}. Please add one.");
            return;
        }

        var bp = new DMBodyPart
        {
            joint = ab,
            startingPos = t.position,
            startingRot = t.rotation,
            startingLocalRot = t.localRotation,
            thisJdController = this
        };

        ab.maxAngularVelocity = k_MaxAngularVelocity;


        bp.groundContact = t.GetComponent<GroundContact>();
        if (!bp.groundContact)
        {
            bp.groundContact = t.gameObject.AddComponent<GroundContact>();
            bp.groundContact.agent = GetComponent<Agent>();
        }
        else
        {
            bp.groundContact.agent = GetComponent<Agent>();
        }


        // Drives initialisieren (Force Mode)
        var xDrive = ab.xDrive;
        xDrive.driveType = ArticulationDriveType.Force;
        xDrive.stiffness = maxJointSpring;
        xDrive.damping = jointDampen;
        xDrive.forceLimit = maxJointForceLimit;
        ab.xDrive = xDrive;

        var yDrive = ab.yDrive;
        yDrive.driveType = ArticulationDriveType.Force;
        yDrive.stiffness = maxJointSpring;
        yDrive.damping = jointDampen;
        yDrive.forceLimit = maxJointForceLimit;
        ab.yDrive = yDrive;

        var zDrive = ab.zDrive;
        zDrive.driveType = ArticulationDriveType.Force;
        zDrive.stiffness = maxJointSpring;
        zDrive.damping = jointDampen;
        zDrive.forceLimit = maxJointForceLimit;
        ab.zDrive = zDrive;

        bodyPartsDict.Add(t, bp);
        bodyPartsList.Add(bp);
    }

}



