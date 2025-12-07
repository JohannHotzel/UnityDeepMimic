using UnityEngine;
using System.Collections;

public class RandomShooter : MonoBehaviour
{
    [Header("Target")]
    public Transform targetHip;             // Assign the agent's hip transform here

    [Header("Shooting Timing")]
    public float minInterval = 1.0f;        // Minimum time between shots
    public float maxInterval = 3.0f;        // Maximum time between shots

    [Header("Spawn Settings")]
    public float spawnDistance = 10.0f;     // Distance from the hip where projectiles spawn

    [Tooltip("If true, bullets will never spawn from below the hip (only from same height or above).")]
    public bool avoidBelowHip = true;       // Prevent spawn directions that start below the hip

    [Header("Projectile")]
    public GameObject projectilePrefab;     // Prefab of the projectile to shoot
    public float projectileSpeed = 10f;     // Initial speed of the projectile
    public float projectileLifeTime = 5f;   // Auto-destroy time for the projectile

    private void Start()
    {
        StartCoroutine(ShootLoop());
    }

    private IEnumerator ShootLoop()
    {
        while (true)
        {
            // Random wait time before next shot
            float waitTime = Random.Range(minInterval, maxInterval);
            yield return new WaitForSeconds(waitTime);

            // Safety checks
            if (targetHip == null || projectilePrefab == null)
                continue;

            // Generate a random direction around the target
            // We want shots from all sides, but not from below the hip
            Vector3 dir = Random.onUnitSphere; // Random point on sphere surface

            if (avoidBelowHip)
            {
                // Flip directions that point downward so they stay above or level
                // This effectively turns the full sphere into a top hemisphere
                if (dir.y < 0f)
                    dir.y = -dir.y;

                dir.Normalize();
            }

            // Spawn position at fixed distance from the hip
            Vector3 spawnPos = targetHip.position + dir * spawnDistance;

            // (Optional safety) Ensure spawnPos is not below hip level if avoidBelowHip is true
            if (avoidBelowHip && spawnPos.y < targetHip.position.y)
            {
                spawnPos.y = targetHip.position.y;
            }

            // Direction from spawn point to the target hip
            Vector3 toTarget = (targetHip.position - spawnPos).normalized;

            // Random rotation for the projectile
            Quaternion rot = Random.rotation;

            // Instantiate projectile
            GameObject proj = Instantiate(projectilePrefab, spawnPos, rot);

            // Apply velocity if the projectile has a Rigidbody
            Rigidbody rb = proj.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.linearVelocity = toTarget * projectileSpeed;
            }

            // Destroy projectile after lifetime expires
            Destroy(proj, projectileLifeTime);
        }
    }

    private void OnDrawGizmos()
    {
        if (targetHip == null)
            return;

        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(targetHip.position, spawnDistance);
    }

}
