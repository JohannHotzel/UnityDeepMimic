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
    private Vector3 referenceSamplerInitialPos;
    private Quaternion referenceSamplerInitialRot;



    [Header("Reward Exponents")]

    [Tooltip("Exponent for pose tracking. Default = 2.0 (corresponds to exp(-2 * sumRotSq)).")]
    public float poseExponent = 2f;

    [Tooltip("Exponent for joint angular velocity tracking. Default = 0.1 (corresponds to exp(-0.1 * sumVelSq)).")]
    public float velocityExponent = 0.1f;

    [Tooltip("Exponent for end-effector position tracking. Default = 40.0 (corresponds to exp(-40 * sumEndSq)).")]
    public float endEffectorExponent = 40f;

    [Tooltip("Exponent for center-of-mass tracking. Default = 10.0 (corresponds to exp(-10 * comSq)).")]
    public float comExponent = 10f;

    [Header("End Effector Targets and Com (Debug Spheres)")]
    public Transform leftHandEndEffector; 
    public Transform rightHandEndEffector;  
    public Transform leftFootEndEffector;  
    public Transform rightFootEndEffector;
    public Transform comTarget;   
    public Transform comRagdoll;

    [Header("Action Smoothing")]
    [Range(0f, 1f)]
    public float actionSmoothing = 0.2f; 
    private float[] smoothedActions;


    [Header("Phase Settings")]
    public float phaseSpeed = 1f; // cycles per second
    private float phase;

    private DMJointDriveController jd;
    private DecisionRequester decisionRequester;
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
        Time.fixedDeltaTime = 1f / 60f;  
        Debug.Log($"Fixed Timestep to: {Time.fixedDeltaTime}");
    }


    //--------------------------------------------------------------------------------------------------------------
    // ML-Agents Methods
    //--------------------------------------------------------------------------------------------------------------
    public override void Initialize()
    {
        jd = GetComponent<DMJointDriveController>();
        decisionRequester = GetComponent<DecisionRequester>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

        if (referenceSampler != null)
        {
            referenceSamplerInitialPos = referenceSampler.transform.position;
            referenceSamplerInitialRot = referenceSampler.transform.rotation;
        }

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

        //hips.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

         phase = Random.Range(0f, 1f);

        if (referenceSampler != null)
        {
            referenceSampler.transform.position = referenceSamplerInitialPos;
            referenceSampler.transform.rotation = referenceSamplerInitialRot;
        }

       // phase = 0f;

        InitializeToReferencePose(phase, true);
    }
    private void InitializeToReferencePose(float phase, bool setVelocities)
    {

        if (referenceSampler == null)
            return;

        float dtSim = GetDecisionDeltaTime();
        float deltaPhase = dtSim * phaseSpeed;



        float phaseNow = phase;
        float phasePrev = phaseNow - deltaPhase;
        if (phasePrev < 0f)
            phasePrev += 1f;

        var refFeatures = referenceSampler.SampleAndExtractPhases(phaseNow, phasePrev, dtSim, out Vector3 refComWorld);

        
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
                Vector3 fwd = Vector3.forward;
                Vector3 velocity = Vector3.zero;
                fwd.y = 0f;

                if (fwd.sqrMagnitude > 1e-6f)
                {
                    fwd.Normalize();
                    velocity = fwd * desiredSpeed;
                }

                Vector3 worldLinVel = hips.TransformDirection(feat.linVel);
                Vector3 worldAngVel = hips.TransformDirection(feat.angVel);

                bp.rb.linearVelocity = worldLinVel + velocity;
                bp.rb.angularVelocity = worldAngVel;
            }

            if (bp.groundContact) bp.groundContact.touchingGround = false;
            if (bp.targetContact) bp.targetContact.touchingTarget = false;

            bp.SetJointTargetRotation(feat.localRot);
        }

        UpdateEndEffectorTargetsDebug(refFeatures);
        UpdateCenterOfMassDebug(refComWorld);
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

        // --- Initialize smoothing buffer once or if size changed ---
        if (smoothedActions == null || smoothedActions.Length != continuousActions.Length)
        {
            smoothedActions = new float[continuousActions.Length];
            for (int k = 0; k < continuousActions.Length; k++)
            {
                smoothedActions[k] = continuousActions[k]; // start without a jump
            }
        }

        // --- Apply simple exponential smoothing: smoothed = lerp(old, new, alpha) ---
        float alpha = actionSmoothing; // 0..1
        for (int k = 0; k < continuousActions.Length; k++)
        {
            float raw = continuousActions[k];
            float prev = smoothedActions[k];
            float s = Mathf.Lerp(prev, raw, alpha);

            smoothedActions[k] = s;
        }

        int i = -1;
        var bp = jd.bodyPartsDict;

        bp[chest].SetJointTargetRotationLocal(smoothedActions[++i], smoothedActions[++i], smoothedActions[++i]);
        bp[spine].SetJointTargetRotationLocal(smoothedActions[++i], smoothedActions[++i], smoothedActions[++i]);

        bp[thighL].SetJointTargetRotationLocal(smoothedActions[++i], smoothedActions[++i], 0);
        bp[thighR].SetJointTargetRotationLocal(smoothedActions[++i], smoothedActions[++i], 0);
        bp[shinL].SetJointTargetRotationLocal(smoothedActions[++i], 0, 0);
        bp[shinR].SetJointTargetRotationLocal(smoothedActions[++i], 0, 0);
        bp[footR].SetJointTargetRotationLocal(smoothedActions[++i], smoothedActions[++i], smoothedActions[++i]);
        bp[footL].SetJointTargetRotationLocal(smoothedActions[++i], smoothedActions[++i], smoothedActions[++i]);

        bp[armL].SetJointTargetRotationLocal(smoothedActions[++i], smoothedActions[++i], 0);
        bp[armR].SetJointTargetRotationLocal(smoothedActions[++i], smoothedActions[++i], 0);
        bp[forearmL].SetJointTargetRotationLocal(smoothedActions[++i], 0, 0);
        bp[forearmR].SetJointTargetRotationLocal(smoothedActions[++i], 0, 0);
        bp[head].SetJointTargetRotationLocal(smoothedActions[++i], smoothedActions[++i], 0);



        // --------- Sample Reference Features at Current Phase -----------------------------
        float dtSim = GetDecisionDeltaTime(); 
        float deltaPhase = dtSim * phaseSpeed;


        if (referenceSampler != null)
        {
            Vector3 fwd = Vector3.forward;
            fwd.y = 0f;

            if (fwd.sqrMagnitude > 1e-6f)
            {
                fwd.Normalize();
                Vector3 velocity = fwd * desiredSpeed * dtSim;
                referenceSampler.transform.position += velocity;
            }
        }


        float phaseNow = phase;
        float phasePrev = phaseNow - deltaPhase;
        if (phasePrev < 0f)
            phasePrev += 1f;

        var refFeatures = referenceSampler.SampleAndExtractPhases(phaseNow, phasePrev, dtSim, out Vector3 refComWorld);

        UpdateEndEffectorTargetsDebug(refFeatures);
        UpdateCenterOfMassDebug(refComWorld);

        float imitationReward = ComputeTrackingReward(refFeatures, refComWorld);
        float taskReward = ComputeTargetHeadingReward();

        float totalReward = imitationWeight * imitationReward + taskWeight * taskReward;
        AddReward(totalReward);

        // Phase für den nächsten Schritt fortschreiben
        phase += deltaPhase;
        if (phase >= 1f)
            phase -= 1f;

    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {

    }


    //--------------------------------------------------------------------------------------------------------------
    // Reward Computation
    //--------------------------------------------------------------------------------------------------------------
    private float ComputeTrackingReward(List<ReferenceMotionSampler.BoneFeatures> refFeatures, Vector3 refComWorld)
    {
        int count = Mathf.Min(refFeatures.Count, jd.bodyPartsList.Count);

        // --- Accumulators for the DeepMimic loss terms ---
        float sumRotSq = 0f; // Σ_j || q̂_j ⊖ q_j ||^2
        float sumVelSq = 0f; // Σ_j || q̇̂_j - q̇_j ||^2
        float sumEndSq = 0f; // Σ_e || p̂_e - p_e ||^2
        float comSq = 0f; // || p̂_c - p_c ||^2

        Transform root = hips;
        Quaternion rootRot = root.rotation;

        // --------- Center of Mass term (agent vs reference) ----------
        Vector3 agentComWorld = ComputeAgentCenterOfMassWorld();          
        Vector3 comDiff = refComWorld - agentComWorld;                   
        comSq = comDiff.sqrMagnitude;

        // --------- Joint rotation + velocity tracking ----------
        for (int i = 0; i < count; i++)
        {
            var bp = jd.bodyPartsList[i];
            var f = refFeatures[i];

            // Local joint rotations relative to the root
            Quaternion agentLocalRot = Quaternion.Inverse(rootRot) * bp.rb.rotation;
            Quaternion refLocalRot = f.localRot;

            // Quaternion difference -> rotation angle in radians
            Quaternion delta = refLocalRot * Quaternion.Inverse(agentLocalRot);
            delta.ToAngleAxis(out float angleDeg, out Vector3 axis);
            if (!float.IsNaN(axis.x))
            {
                float angleRad = angleDeg * Mathf.Deg2Rad;
                sumRotSq += angleRad * angleRad;
            }

            // Angular velocity difference in root space
            Vector3 agentAngVelLocal = root.InverseTransformDirection(bp.rb.angularVelocity);
            Vector3 velDiff = f.angVel - agentAngVelLocal;
            sumVelSq += velDiff.sqrMagnitude;

            // End-effector positional tracking (hands & feet)
            if (IsEndEffector(bp.rb.transform))
            {
                Vector3 refWorldPos = f.worldPos;
                Vector3 diff = refWorldPos - bp.rb.position;
                sumEndSq += diff.sqrMagnitude;
            }
        }

        // --------- Individual rewards as defined in the DeepMimic paper ---------

        // r_p = exp( -2 * Σ_j ||Δq_j||^2 )
        float rPose = Mathf.Exp(-poseExponent * sumRotSq);

        // r_v = exp( -0.1 * Σ_j ||Δω_j||^2 )
        float rVel = Mathf.Exp(-velocityExponent * sumVelSq);

        // r_e = exp( -40 * Σ_e ||Δp_e||^2 )
        float rEnd = Mathf.Exp(-endEffectorExponent * sumEndSq);

        // r_c = exp( -10 * ||Δp_c||^2 )
        float rCom = Mathf.Exp(-comExponent * comSq);

        // --------- Weighted combination of all imitation terms ---------
        const float w_p = 0.65f; // pose
        const float w_v = 0.10f; // velocity
        const float w_e = 0.15f; // end-effector
        const float w_c = 0.10f; // center of mass

        float imitationReward =
            w_p * rPose +
            w_v * rVel +
            w_e * rEnd +
            w_c * rCom;


        if (Time.frameCount % 10 == 0)  // nur alle 10 Frames loggen
        {
            Debug.Log(
                $"Reward Breakdown:\n" +
                $"sumRotSq: {sumRotSq:F4}, rPose: {rPose:F4}\n" +
                $"sumVelSq: {sumVelSq:F4}, rVel: {rVel:F4}\n" +
                $"sumEndSq: {sumEndSq:F4}, rEnd: {rEnd:F4}\n" +
                $"comSq: {comSq:F4}, rCom: {rCom:F4}\n" +
                $"TOTAL imitationReward: {imitationReward:F4}"
            );
        }



        return imitationReward;
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
    private bool IsEndEffector(Transform t)
    {
        return t == footL || t == footR || t == handL || t == handR;
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
    private Vector3 ComputeAgentCenterOfMassWorld()
    {
        float totalMass = 0f;
        Vector3 accum = Vector3.zero;

        for (int i = 0; i < jd.bodyPartsList.Count; i++)
        {
            var bp = jd.bodyPartsList[i];
            if (bp.rb == null) continue;

            float m = bp.rb.mass;
            totalMass += m;
            accum += bp.rb.transform.position * m;                        
        }

        if (totalMass <= 0f)
            return hips.position; 

        return accum / totalMass;                                        
    }


    //--------------------------------------------------------------------------------------------------------------
    // Delta Time for Decision Intervals
    //--------------------------------------------------------------------------------------------------------------
    private float GetDecisionDeltaTime()
    {
        int period = 1;

        if (decisionRequester != null && decisionRequester.DecisionPeriod > 0)
        {
            period = decisionRequester.DecisionPeriod;
        }

        return Time.fixedDeltaTime * period;
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


        int count = Mathf.Min(refFeatures.Count, jd.bodyPartsList.Count);

        for (int i = 0; i < count; i++)
        {
            var bp = jd.bodyPartsList[i];
            if (!IsEndEffector(bp.rb.transform))
                continue;

            var f = refFeatures[i];
            // ALT:
            // Vector3 refWorldPos = root.TransformPoint(f.localPos);

            Vector3 refWorldPos = f.worldPos;                           

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
    private void UpdateCenterOfMassDebug(Vector3 refComWorld)
    {
        if (comTarget == null || comRagdoll == null)
        {
            return;
        }

        comTarget.position = refComWorld;                            

        Vector3 ragdollComWorld = ComputeAgentCenterOfMassWorld();      
        comRagdoll.position = ragdollComWorld;                        
    }


}
