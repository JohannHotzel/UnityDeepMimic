using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

public class DeepMimicAgent : Agent
{
    [Header("Body Parts")]
    public Transform hips;
    public Transform chest;
    public Transform spine;
    public Transform head;
    public Transform thighL;
    public Transform shinL;
    public Transform footL;
    public Transform thighR;
    public Transform shinR;
    public Transform footR;
    public Transform armL;
    public Transform forearmL;
    public Transform handL;
    public Transform armR;
    public Transform forearmR;
    public Transform handR;

    [Header("Reference Motion")]
    public ReferenceMotionSampler referenceSampler;

    [Header("End Effector Targets and Com (Debug Spheres)")]
    public Transform leftHandEndEffector; 
    public Transform rightHandEndEffector;  
    public Transform leftFootEndEffector;  
    public Transform rightFootEndEffector;
    public Transform comTarget;   
    public Transform comRagdoll;


    [Header("Phase Settings")]
    public float phaseSpeed = 1f; // cycles per second
    private float phase;

    private DMJointDriveController jd;
    DirectionIndicator m_DirectionIndicator;


    [Header("Task: Target Heading")]
    public Transform target;         
    public float desiredSpeed = 1.0f;  
    [Range(0f, 1f)]
    public float imitationWeight = 0.7f;
    [Range(0f, 1f)]
    public float taskWeight = 0.3f;



    protected override void Awake()
    {
        base.Awake();
        Time.fixedDeltaTime = 1f / 60f;   // Decision Frequency = 2 to achive 30 Hz 
        Debug.Log($"Fixed Timestep to: {Time.fixedDeltaTime}");
    }

    public override void Initialize()
    {
        jd = GetComponent<DMJointDriveController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

        // Setup body parts
        jd.SetupBodyPart(hips);
        jd.SetupBodyPart(chest);
        jd.SetupBodyPart(spine);
        jd.SetupBodyPart(head);
        jd.SetupBodyPart(thighL);
        jd.SetupBodyPart(shinL);
        jd.SetupBodyPart(footL);
        jd.SetupBodyPart(thighR);
        jd.SetupBodyPart(shinR);
        jd.SetupBodyPart(footR);
        jd.SetupBodyPart(armL);
        jd.SetupBodyPart(forearmL);
        jd.SetupBodyPart(handL);
        jd.SetupBodyPart(armR);
        jd.SetupBodyPart(forearmR);
        jd.SetupBodyPart(handR);

        foreach (var bp in jd.bodyPartsList)
        {
            if (bp.joint)
            {
                bp.SetJointStrengthConstant(1.0f);
            }
        }
    }
    public override void OnEpisodeBegin()
    {
        foreach (var bp in jd.bodyPartsList)
            bp.Reset();

        hips.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        phase = Random.Range(0f, 1f);

        InitializeToReferencePose(phase, true);
    }
    private void InitializeToReferencePose(float phase, bool setVelocities)
    {
        var refFeatures = referenceSampler.SampleAndExtract(phase, out Vector3 com);
        int count = Mathf.Min(jd.bodyPartsList.Count, refFeatures.Count);

        for (int i = 0; i < count; i++)
        {
            var bp = jd.bodyPartsList[i];
            var feat = refFeatures[i];

            Vector3 worldPos = hips.TransformPoint(feat.localPos);
            Quaternion worldRot = hips.rotation * feat.localRot;

            bp.rb.transform.position = worldPos;
            bp.rb.transform.rotation = worldRot;

            if (setVelocities)
            {
                bp.rb.linearVelocity = Vector3.zero;
                bp.rb.angularVelocity = Vector3.zero;
            }

            if (bp.groundContact) bp.groundContact.touchingGround = false;
            if (bp.targetContact) bp.targetContact.touchingTarget = false;

            bp.SetJointTargetRotation(feat.localRot);
        }

        UpdateEndEffectorTargetsDebug(refFeatures);
        UpdateCenterOfMassDebug(com);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Transform root = hips;
        Quaternion rootRot = root.rotation;

        foreach (var bp in jd.bodyPartsList)
        {
            Vector3 relPos = root.InverseTransformPoint(bp.rb.position);
            sensor.AddObservation(relPos);

            Quaternion relRot = Quaternion.Inverse(rootRot) * bp.rb.rotation;
            sensor.AddObservation(relRot);

            Vector3 linVel = root.InverseTransformDirection(bp.rb.linearVelocity);
            Vector3 angVel = root.InverseTransformDirection(bp.rb.angularVelocity);
            sensor.AddObservation(linVel);
            sensor.AddObservation(angVel);
        }

        sensor.AddObservation(phase);



        if (target != null)
        {
            Vector3 toTarget = target.position - hips.position;
            toTarget.y = 0f;

            if (toTarget.sqrMagnitude > 1e-6f)
            {
                Vector3 dStarWorld = toTarget.normalized;

                Vector3 dStarLocal = hips.InverseTransformDirection(dStarWorld);
                dStarLocal.y = 0f;

                sensor.AddObservation(dStarLocal.normalized); 
            }

            else
            {
                sensor.AddObservation(Vector3.zero);
            }

            sensor.AddObservation(desiredSpeed);
        }

        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(0f);
        }

    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        
        var continuousActions = actions.ContinuousActions;
        int i = -1;
        var bp = jd.bodyPartsDict;

        bp[chest].SetJointTargetRotationLocal(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bp[spine].SetJointTargetRotationLocal(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        bp[thighL].SetJointTargetRotationLocal(continuousActions[++i], continuousActions[++i], 0);
        bp[thighR].SetJointTargetRotationLocal(continuousActions[++i], continuousActions[++i], 0);
        bp[shinL].SetJointTargetRotationLocal(continuousActions[++i], 0, 0);
        bp[shinR].SetJointTargetRotationLocal(continuousActions[++i], 0, 0);
        bp[footR].SetJointTargetRotationLocal(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bp[footL].SetJointTargetRotationLocal(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        bp[armL].SetJointTargetRotationLocal(continuousActions[++i], continuousActions[++i], 0);
        bp[armR].SetJointTargetRotationLocal(continuousActions[++i], continuousActions[++i], 0);
        bp[forearmL].SetJointTargetRotationLocal(continuousActions[++i], 0, 0);
        bp[forearmR].SetJointTargetRotationLocal(continuousActions[++i], 0, 0);
        bp[head].SetJointTargetRotationLocal(continuousActions[++i], continuousActions[++i], 0);


        var refFeatures = referenceSampler.SampleAndExtract(phase, out Vector3 refComLocal);

        UpdateEndEffectorTargetsDebug(refFeatures);
        UpdateCenterOfMassDebug(refComLocal);

        float imitationReward = ComputeTrackingReward(refFeatures, refComLocal);
        float taskReward = ComputeTargetHeadingReward();

        float totalReward = imitationWeight * imitationReward + taskWeight * taskReward;

        AddReward(totalReward);


        phase += Time.fixedDeltaTime * phaseSpeed;
        if (phase >= 1f) phase -= 1f;

    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {

    }

    private float ComputeTrackingReward(List<ReferenceMotionSampler.BoneFeatures> refFeatures, Vector3 refComLocal)
    {
        int count = Mathf.Min(refFeatures.Count, jd.bodyPartsList.Count);

        // --- Summen aus dem Paper ---
        float sumRotSq = 0f; // Σ_j || q̂_j ⊖ q_j ||^2
        float sumVelSq = 0f; // Σ_j || q̇̂_j - q̇_j ||^2
        float sumEndSq = 0f; // Σ_e || p̂_e - p_e ||^2
        float comSq = 0f; // || p̂_c - p_c ||^2

        Transform root = hips;
        Quaternion rootRot = root.rotation;

        // --------- Center of Mass (Agent) ----------
        Vector3 agentComLocal = ComputeAgentCenterOfMassLocal();
        Vector3 comDiff = refComLocal - agentComLocal;
        comSq = comDiff.sqrMagnitude;

        // --------- Gelenke: Pose + Velocity ----------
        for (int i = 0; i < count; i++)
        {
            var bp = jd.bodyPartsList[i];
            var f = refFeatures[i];

            // Pose: lokale Rotationen im Root-Space
            Quaternion agentLocalRot = Quaternion.Inverse(rootRot) * bp.rb.rotation;
            Quaternion refLocalRot = f.localRot;

            // Quaternion-Differenz -> Rotationswinkel (in Radiant)
            Quaternion delta = refLocalRot * Quaternion.Inverse(agentLocalRot);
            delta.ToAngleAxis(out float angleDeg, out Vector3 axis);
            if (!float.IsNaN(axis.x))
            {
                float angleRad = angleDeg * Mathf.Deg2Rad;
                sumRotSq += angleRad * angleRad;
            }

            // Angular Velocity: in Root-Space vergleichen
            Vector3 agentAngVelLocal = root.InverseTransformDirection(bp.rb.angularVelocity);
            Vector3 velDiff = f.angVel - agentAngVelLocal;
            sumVelSq += velDiff.sqrMagnitude;

            // --- End-Effektor-Term (Hände & Füße) ---
            if (IsEndEffector(bp.rb.transform))
            {
                Vector3 refWorldPos = root.TransformPoint(f.localPos);
                Vector3 diff = refWorldPos - bp.rb.position;
                sumEndSq += diff.sqrMagnitude;
            }
        }

        // --------- Einzel-Rewards wie im Paper ---------
        // r_p = exp( -2 * Σ_j ||Δq_j||^2 )
        float rPose = Mathf.Exp(-2f * sumRotSq);

        // r_v = exp( -0.1 * Σ_j ||Δω_j||^2 )
        float rVel = Mathf.Exp(-0.1f * sumVelSq);

        // r_e = exp( -40 * Σ_e ||Δp_e||^2 )
        float rEnd = Mathf.Exp(-40f * sumEndSq);

        // r_c = exp( -10 * ||Δp_c||^2 )
        float rCom = Mathf.Exp(-10f * comSq);

        // --------- Gewichte (Paper) ---------
        const float w_p = 0.65f;
        const float w_v = 0.10f;
        const float w_e = 0.15f;
        const float w_c = 0.10f;

        float imitationReward =
            w_p * rPose +
            w_v * rVel +
            w_e * rEnd +
            w_c * rCom;

        return imitationReward;
    }
    private bool IsEndEffector(Transform t)
    {
        return t == footL || t == footR || t == handL || t == handR;
    }
    private float ComputeTargetHeadingReward()
    {
        if (target == null)
            return 0f;

        Vector3 toTarget = target.position - hips.position;
        toTarget.y = 0f;


        Vector3 dStar = toTarget.normalized; 

        var hipsBP = jd.bodyPartsDict[hips];
        Vector3 v = hipsBP.rb.linearVelocity;
        v.y = 0f;

        float vParallel = Vector3.Dot(v, dStar);

        float vStar = desiredSpeed;

        float speedError = Mathf.Max(0f, vStar - vParallel);

        float rG = Mathf.Exp(-2.5f * speedError * speedError);

        return rG;
    }

    private Vector3 ComputeAgentCenterOfMassLocal()
    {
        float totalMass = 0f;
        Vector3 accum = Vector3.zero;

        for (int i = 0; i < jd.bodyPartsList.Count; i++)
        {
            var bp = jd.bodyPartsList[i];
            float m = 1f; // Default

            if (bp.rb)
                m = bp.rb.mass;

            totalMass += m;
            accum += bp.rb.transform.position * m;
        }

        if (totalMass <= 0f)
            return hips.position; // fallback

        Vector3 comWorld = accum / totalMass;
        return hips.InverseTransformPoint(comWorld);
    }



    //--------------------------------------------------------------------------------------------------------------
    // Debug: Update End-Effector Targets and COM Spheres
    //--------------------------------------------------------------------------------------------------------------
    private void UpdateEndEffectorTargetsDebug(List<ReferenceMotionSampler.BoneFeatures> refFeatures)
    {
        if (leftHandEndEffector == null && rightHandEndEffector == null &&
            leftFootEndEffector == null && rightFootEndEffector == null)
        {
            return;
        }

        Transform root = hips;
        int count = Mathf.Min(refFeatures.Count, jd.bodyPartsList.Count);

        for (int i = 0; i < count; i++)
        {
            var bp = jd.bodyPartsList[i];
            if (!IsEndEffector(bp.rb.transform))
                continue;

            var f = refFeatures[i];
            Vector3 refWorldPos = root.TransformPoint(f.localPos);

            if (bp.rb.transform == handL && leftHandEndEffector != null)
            {
                leftHandEndEffector.position = refWorldPos;
            }
            else if (bp.rb.transform == handR && rightHandEndEffector != null)
            {
                rightHandEndEffector.position = refWorldPos;
            }
            else if (bp.rb.transform == footL && leftFootEndEffector != null)
            {
                leftFootEndEffector.position = refWorldPos;
            }
            else if (bp.rb.transform == footR && rightFootEndEffector != null)
            {
                rightFootEndEffector.position = refWorldPos;
            }
        }
    }
    private void UpdateCenterOfMassDebug(Vector3 refComLocal)
    {
        if (comTarget == null || comRagdoll == null)
        {
            return;
        }

        Transform root = hips;

        Vector3 refComWorld = root.TransformPoint(refComLocal);
        comTarget.position = refComWorld;

        Vector3 ragdollComLocal = ComputeAgentCenterOfMassLocal();
        Vector3 ragdollComWorld = root.TransformPoint(ragdollComLocal);
        comRagdoll.position = ragdollComWorld;
    }


}
