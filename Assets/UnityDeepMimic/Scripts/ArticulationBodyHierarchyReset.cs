using System.Collections.Generic;
using UnityEngine;

public class ArticulationBodyHierarchyReset : MonoBehaviour
{
    [SerializeField]
    private ArticulationBody root;   // Root der Articulation-Hierarchie

    private readonly List<float> initialJointPositions = new();
    private readonly List<float> initialJointVelocities = new();

    private bool hasInitialBackup = false;

    private Vector3 initialRootPosition;
    private Quaternion initialRootRotation;

    private void Reset()
    {
        if (root == null)
            root = GetComponentInChildren<ArticulationBody>();
    }

    private void Awake()
    {
        if (root == null)
        {
            root = GetComponentInChildren<ArticulationBody>();
        }

        if (root == null)
        {
            Debug.LogError("[ArticulationBodyHierarchyReset] Kein ArticulationBody gefunden.");
            return;
        }

        // Root-Transform merken (global)
        initialRootPosition = root.transform.position;
        initialRootRotation = root.transform.rotation;

        // Initiale DOFs der ganzen Kette holen
        initialJointPositions.Clear();
        initialJointVelocities.Clear();

        root.GetJointPositions(initialJointPositions);
        root.GetJointVelocities(initialJointVelocities);

        hasInitialBackup = true;
    }


    public void ResetHierarchyToInitial()
    {
        if (!hasInitialBackup || root == null)
        {
            Debug.LogWarning("[ArticulationBodyHierarchyReset] Kein gültiges Initial-Backup vorhanden.");
            return;
        }

        root.TeleportRoot(initialRootPosition, initialRootRotation);

        root.linearVelocity = Vector3.zero;
        root.angularVelocity = Vector3.zero;

        var pos = new List<float>(initialJointPositions.Count);
        var vel = new List<float>(initialJointVelocities.Count);

        pos.AddRange(initialJointPositions);
        vel.AddRange(initialJointVelocities);

        root.SetJointPositions(pos);
        root.SetJointVelocities(vel);
    }

    public void CaptureCurrentAsNewInitial()
    {
        if (root == null)
            return;

        initialRootPosition = root.transform.position;
        initialRootRotation = root.transform.rotation;

        initialJointPositions.Clear();
        initialJointVelocities.Clear();

        root.GetJointPositions(initialJointPositions);
        root.GetJointVelocities(initialJointVelocities);

        hasInitialBackup = true;
    }
}
