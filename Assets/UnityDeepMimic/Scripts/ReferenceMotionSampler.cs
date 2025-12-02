using UnityEngine;
using System.Collections.Generic;

public class ReferenceMotionSampler : MonoBehaviour
{

    public AnimationClip clip;
    public Animator animator;
    public Transform rootBone;
    public List<Transform> bones = new List<Transform>();
    public float[] boneMasses;

    public float sampleRateHz = 30f;
    public float ClipLength => clip != null ? clip.length : 0f;
    public float SampleRateHz => sampleRateHz;
    public int TotalFrames => clip != null ? Mathf.RoundToInt(clip.length * sampleRateHz) : 0;

    private Vector3[] prevPositions;
    private Quaternion[] prevRotations;

    private Vector3 lastComWorld;


    public struct BoneFeatures
    {
        public Vector3 localPos;
        public Vector3 worldPos;
        public Quaternion localRot; 
        public Vector3 linVel;     
        public Vector3 angVel;  
    }

    public List<BoneFeatures> currentFeatures = new List<BoneFeatures>();


    private void Awake()
    {
        if (animator != null)
        {
            animator.speed = 0f;
        }
    }

    public List<BoneFeatures> SampleAndExtract(float phase, out Vector3 comWorld)
    {
        comWorld = Vector3.zero;

        // Validate inputs
        if (clip == null || animator == null || rootBone == null || bones.Count == 0 || sampleRateHz <= 0f)
            return currentFeatures;

        int totalFrames = TotalFrames;
        if (totalFrames <= 1)
            return currentFeatures;

        EnsureBuffers();


        float phiNow = Mathf.Repeat(phase, 1f); // phi in [0,1]

        float dt = 1f / sampleRateHz;
        float deltaPhi = dt / clip.length;

        float phiPrev = phiNow - deltaPhi;
        if (phiPrev < 0f) // cycle back
            phiPrev += 1f;

        SamplePoseAtPhase(phiPrev);
        for (int i = 0; i < bones.Count; i++)
        {
            Transform b = bones[i];
            prevPositions[i] = b.position;
            prevRotations[i] = b.rotation;
        }


        SamplePoseAtPhase(phiNow);

        Vector3 comWorldNow = ComputeCenterOfMassWorld();
        comWorld = comWorldNow;
        lastComWorld = comWorldNow;


        currentFeatures.Clear();

        for (int i = 0; i < bones.Count; i++)
        {
            Transform bone = bones[i];
            BoneFeatures f = new BoneFeatures();

            f.localPos = rootBone.InverseTransformPoint(bone.position);
            f.worldPos = bone.position;
            f.localRot = Quaternion.Inverse(rootBone.rotation) * bone.rotation;

            Vector3 prevPos = prevPositions[i];
            Quaternion prevRot = prevRotations[i];

            Vector3 worldLinVel = (bone.position - prevPos) / dt;
            Vector3 worldAngVel = ComputeAngularVelocity(prevRot, bone.rotation, dt);

            f.linVel = rootBone.InverseTransformDirection(worldLinVel);
            f.angVel = rootBone.InverseTransformDirection(worldAngVel);

            currentFeatures.Add(f);
        }


        return currentFeatures;
    }
    public List<BoneFeatures> SampleAndExtractPhases(float phaseNow, float phasePrev, float deltaTime, out Vector3 comWorld)
    {
        comWorld = Vector3.zero;

        if (clip == null || animator == null || rootBone == null || bones.Count == 0)
            return currentFeatures;

        EnsureBuffers();

        float phiPrev = Mathf.Repeat(phasePrev, 1f);
        float phiNow = Mathf.Repeat(phaseNow, 1f);

        // ------------------------------------------
        // 1) Sample the pose at the previous phase and store bone transforms
        // ------------------------------------------
        SamplePoseAtPhase(phiPrev);
        for (int i = 0; i < bones.Count; i++)
        {
            prevPositions[i] = bones[i].position;
            prevRotations[i] = bones[i].rotation;
        }

        // ------------------------------------------
        // 2) Sample the pose at the current phase
        // ------------------------------------------
        SamplePoseAtPhase(phiNow);

        Vector3 comWorldNow = ComputeCenterOfMassWorld();
        comWorld = comWorldNow;                                                 
        lastComWorld = comWorldNow;

        currentFeatures.Clear();

        bool dtZero = deltaTime == 0f;

        for (int i = 0; i < bones.Count; i++)
        {
            Transform bone = bones[i];
            BoneFeatures f = new BoneFeatures();

            // Local position and rotation relative to the root
            f.localPos = rootBone.InverseTransformPoint(bone.position);
            f.worldPos = bone.position;
            f.localRot = Quaternion.Inverse(rootBone.rotation) * bone.rotation;

            // Previous transforms
            Vector3 prevPos = prevPositions[i];
            Quaternion prevRot = prevRotations[i];

            // -----------------------------
            // 3) If dt == 0 -> pose is static, velocities must be zero
            // -----------------------------
            if (dtZero)
            {
                f.linVel = Vector3.zero;
                f.angVel = Vector3.zero;
            }
            else
            {
                // Compute world-space linear and angular velocities
                Vector3 worldLinVel = (bone.position - prevPos) / deltaTime;
                Vector3 worldAngVel = ComputeAngularVelocity(prevRot, bone.rotation, deltaTime);

                // Transform velocities into root space
                f.linVel = rootBone.InverseTransformDirection(worldLinVel);
                f.angVel = rootBone.InverseTransformDirection(worldAngVel);
            }

            currentFeatures.Add(f);
        }

        return currentFeatures;
    }


    private void EnsureBuffers()
    {
        if (prevPositions == null || prevPositions.Length != bones.Count)
        {
            prevPositions = new Vector3[bones.Count];
            prevRotations = new Quaternion[bones.Count];
        }
    }
    private void SamplePoseAtPhase(float phase)
    {
        if (clip == null || animator == null)
            return;

        animator.Play(clip.name, 0, phase);
        animator.Update(0f);
    }
    private Vector3 ComputeAngularVelocity(Quaternion qPrev, Quaternion qCurr, float dt)
    {
        Quaternion dq = qCurr * Quaternion.Inverse(qPrev);
        dq.ToAngleAxis(out float angleDeg, out Vector3 axis);

        if (float.IsNaN(axis.x))
            return Vector3.zero;

        float angleRad = angleDeg * Mathf.Deg2Rad;
        return axis * (angleRad / dt);
    }
    private Vector3 ComputeCenterOfMassWorld()
    {
        float totalMass = 0f;
        Vector3 accum = Vector3.zero;

        for (int i = 0; i < bones.Count; i++)
        {
            Transform b = bones[i];
            float m = 1f;

            if (boneMasses != null && i < boneMasses.Length && boneMasses[i] > 0f)
                m = boneMasses[i];

            totalMass += m;
            accum += b.position * m;
        }

        return totalMass > 0f ? accum / totalMass : rootBone.position;
    }


    private void OnDrawGizmos()
    {
        if (rootBone == null)
            return;

        Vector3 comWorld = lastComWorld;             

        Gizmos.color = Color.cyan;
        Gizmos.DrawSphere(comWorld, 0.03f);
    }

}
