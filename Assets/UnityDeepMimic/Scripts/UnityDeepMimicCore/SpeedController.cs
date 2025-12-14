using UnityEngine;

public class SpeedController : MonoBehaviour
{
    [Header("Random Speed Multiplier Settings")]
    [Tooltip("Enable or disable random speed multiplier changes.")]
    public bool randomSpeed = true;

    [Tooltip("Minimum target speed multiplier (1 = base speed).")]
    public float minMultiplier = 1f;

    [Tooltip("Maximum target speed multiplier.")]
    public float maxMultiplier = 2f;

    [Tooltip("How often a new random target multiplier is picked (seconds).")]
    public float multiplierChangeInterval = 4.0f;

    [Tooltip("How fast to interpolate towards the new target multiplier (higher = faster).")]
    public float lerpRate = 3.0f;

    [Header("Runtime")]
    [SerializeField]
    private float currentMultiplier = 1.0f;

    private float targetMultiplier = 1.0f;
    private float timeToNextChange = 0.0f;

    public float SpeedMultiplier => currentMultiplier;

    private void Start()
    {
        PickNewTargetMultiplier(immediate: true);
    }

    private void Update()
    {
        if (!randomSpeed)
            return;

        // Exponential smoothing (frame-rate independent)
        float t = 1f - Mathf.Exp(-lerpRate * Time.deltaTime);
        currentMultiplier = Mathf.Lerp(currentMultiplier, targetMultiplier, t);

        // Countdown until next target change
        timeToNextChange -= Time.deltaTime;
        if (timeToNextChange <= 0f)
        {
            PickNewTargetMultiplier(immediate: false);
        }
    }

    private void PickNewTargetMultiplier(bool immediate)
    {
        targetMultiplier = Random.Range(minMultiplier, maxMultiplier);
        timeToNextChange = multiplierChangeInterval;

        if (immediate)
            currentMultiplier = targetMultiplier;
    }

}
