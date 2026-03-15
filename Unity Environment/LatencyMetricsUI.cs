using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class LatencyMetricsUI : MonoBehaviour
{
    [Header("UI References")]
    [SerializeField] private TextMeshProUGUI cameraLatencyText;
    [SerializeField] private TextMeshProUGUI handTrackingLatencyText;
    [SerializeField] private TextMeshProUGUI overallStatusText;
    [SerializeField] private TextMeshProUGUI detailsText;
    
    [Header("Settings")]
    [SerializeField] private float updateFrequency = 2f; // Updates per second
    [SerializeField] private bool showDetailedBreakdown = true;
    [SerializeField] private bool autoFindComponents = true;
    
    [Header("World Space Canvas Setup")]
    [SerializeField] private bool autoSetupWorldCanvas = true;
    [SerializeField] private Vector3 worldPosition = new Vector3(0f, 1.5f, 2f); // Default position in front of origin
    [SerializeField] private Vector3 worldRotation = Vector3.zero; // Euler angles
    [SerializeField] private float canvasScale = 0.01f; // Scale for world space canvas
    [SerializeField] private bool faceCamera = true; // Always face the main camera
    [SerializeField] private bool showPositionGizmo = true; // Show gizmo in scene view
    
    [Header("Text Formatting")]
    [SerializeField] private float mainTextSize = 36f; // Size for camera and hand tracking text
    [SerializeField] private float statusTextSize = 32f; // Size for overall status text
    [SerializeField] private float detailsTextSize = 24f; // Size for detailed breakdown text
    [SerializeField] private bool autoApplyTextSizes = true; // Apply sizes automatically
    
    [Header("Manual Component References (Optional)")]
    [SerializeField] private CameraImageSubscriberDebug cameraSubscriber;
    [SerializeField] private HandTipHUD handTracking;
    
    [Header("Color Coding")]
    [SerializeField] private Color excellentColor = Color.green;
    [SerializeField] private Color goodColor = Color.yellow;
    [SerializeField] private Color poorColor = Color.red;
    [SerializeField] private Color noDataColor = Color.gray;
    
    [Header("Latency Thresholds (ms)")]
    [SerializeField] private float excellentThreshold = 30f;
    [SerializeField] private float goodThreshold = 60f;
    
    // Private variables
    private float updateTimer = 0f;
    private int frameCount = 0;
    private bool componentsFound = false;
    private Canvas parentCanvas;
    private Camera mainCamera;
    
    // Cache for performance metrics
    private struct LatencyData
    {
        public float current;
        public float average;
        public float min;
        public float max;
        public bool valid;
    }
    
    private LatencyData cameraLatency;
    private LatencyData handLatency;

    void Start()
    {
        InitializeUI();
        
        if (autoSetupWorldCanvas)
        {
            SetupWorldSpaceCanvas();
        }
        
        if (autoFindComponents)
        {
            FindLatencyComponents();
        }
        
        if (cameraSubscriber == null && handTracking == null)
        {
            Debug.LogWarning("LatencyMetricsUI: No latency measurement components found. Assign manually or ensure autoFindComponents is enabled.");
        }
    }
    
    void InitializeUI()
    {
        // Set initial text if components are missing
        if (cameraLatencyText != null)
        {
            cameraLatencyText.text = "Camera: Searching...";
            cameraLatencyText.color = noDataColor;
        }
        
        if (handTrackingLatencyText != null)
        {
            handTrackingLatencyText.text = "Hand Tracking: Searching...";
            handTrackingLatencyText.color = noDataColor;
        }
        
        if (overallStatusText != null)
        {
            overallStatusText.text = "System Status: Initializing...";
            overallStatusText.color = noDataColor;
        }
        
        if (detailsText != null)
        {
            detailsText.text = "Detailed metrics will appear here...";
            detailsText.color = noDataColor;
        }
    }
    
    void FindLatencyComponents()
    {
        if (cameraSubscriber == null)
        {
            cameraSubscriber = FindObjectOfType<CameraImageSubscriberDebug>();
            if (cameraSubscriber != null)
            {
                Debug.Log("LatencyMetricsUI: Found CameraImageSubscriberDebug component");
            }
        }
        
        if (handTracking == null)
        {
            handTracking = FindObjectOfType<HandTipHUD>();
            if (handTracking != null)
            {
                Debug.Log("LatencyMetricsUI: Found HandTipHUD component");
            }
        }
        
        componentsFound = (cameraSubscriber != null || handTracking != null);
        
        if (componentsFound)
        {
            Debug.Log("LatencyMetricsUI: Components found successfully");
        }
    }
    
    void SetupWorldSpaceCanvas()
    {
        // Find or get the parent canvas
        parentCanvas = GetComponentInParent<Canvas>();
        
        if (parentCanvas == null)
        {
            Debug.LogWarning("LatencyMetricsUI: No parent Canvas found. Please attach this script to a GameObject under a Canvas.");
            return;
        }
        
        // Find main camera
        mainCamera = Camera.main;
        if (mainCamera == null)
        {
            mainCamera = FindObjectOfType<Camera>();
        }
        
        // Configure canvas for world space
        parentCanvas.renderMode = RenderMode.WorldSpace;
        
        // Set canvas transform
        Transform canvasTransform = parentCanvas.transform;
        canvasTransform.position = worldPosition;
        canvasTransform.rotation = Quaternion.Euler(worldRotation);
        canvasTransform.localScale = Vector3.one * canvasScale;
        
        // Configure CanvasScaler for better text rendering
        CanvasScaler scaler = parentCanvas.GetComponent<CanvasScaler>();
        if (scaler == null)
        {
            scaler = parentCanvas.gameObject.AddComponent<CanvasScaler>();
        }
        
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920, 1080);
        scaler.matchWidthOrHeight = 0.5f;
        
        Debug.Log($"LatencyMetricsUI: World space canvas configured at position {worldPosition} with scale {canvasScale}");
        
        // Position this UI element within the canvas if needed
        RectTransform rectTransform = GetComponent<RectTransform>();
        if (rectTransform == null)
        {
            rectTransform = gameObject.AddComponent<RectTransform>();
        }
        
        // Apply text sizes if enabled
        if (autoApplyTextSizes)
        {
            ApplyTextSizes();
        }
    }

    void Update()
    {
        updateTimer += Time.deltaTime;
        frameCount++;
        
        // Make canvas face camera if enabled
        if (faceCamera && parentCanvas != null && mainCamera != null)
        {
            Vector3 directionToCamera = mainCamera.transform.position - parentCanvas.transform.position;
            directionToCamera.y = 0f; // Keep it upright
            
            if (directionToCamera != Vector3.zero)
            {
                parentCanvas.transform.rotation = Quaternion.LookRotation(-directionToCamera);
            }
        }
        
        // Update at specified frequency
        if (updateTimer >= 1f / updateFrequency)
        {
            UpdateLatencyData();
            UpdateUI();
            updateTimer = 0f;
        }
    }
    
    void UpdateLatencyData()
    {
        // Update camera latency data using reflection to access private fields
        if (cameraSubscriber != null)
        {
            cameraLatency = GetCameraLatencyData();
        }
        else
        {
            cameraLatency = new LatencyData { valid = false };
        }
        
        // Update hand tracking latency data using reflection to access private fields
        if (handTracking != null)
        {
            handLatency = GetHandTrackingLatencyData();
        }
        else
        {
            handLatency = new LatencyData { valid = false };
        }
    }
    
    LatencyData GetCameraLatencyData()
    {
        try
        {
            var cameraType = cameraSubscriber.GetType();
            
            // Use reflection to access private fields
            var currentField = cameraType.GetField("currentLatencyMs", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var avgField = cameraType.GetField("avgLatencyMs", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var minField = cameraType.GetField("minLatencyMs", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var maxField = cameraType.GetField("maxLatencyMs", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var enabledField = cameraType.GetField("enableLatencyMeasurement", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            
            bool enabled = enabledField != null ? (bool)enabledField.GetValue(cameraSubscriber) : false;
            
            if (!enabled)
            {
                return new LatencyData { valid = false };
            }
            
            return new LatencyData
            {
                current = currentField != null ? (float)currentField.GetValue(cameraSubscriber) : 0f,
                average = avgField != null ? (float)avgField.GetValue(cameraSubscriber) : 0f,
                min = minField != null ? (float)minField.GetValue(cameraSubscriber) : 0f,
                max = maxField != null ? (float)maxField.GetValue(cameraSubscriber) : 0f,
                valid = enabled && currentField != null
            };
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"LatencyMetricsUI: Error accessing camera latency data: {e.Message}");
            return new LatencyData { valid = false };
        }
    }
    
    LatencyData GetHandTrackingLatencyData()
    {
        try
        {
            var handType = handTracking.GetType();
            
            // Use reflection to access private fields
            var currentField = handType.GetField("currentLatencyMs", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var avgField = handType.GetField("avgLatencyMs", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var minField = handType.GetField("minLatencyMs", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var maxField = handType.GetField("maxLatencyMs", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var enabledField = handType.GetField("enableLatencyMeasurement", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            
            bool enabled = enabledField != null ? (bool)enabledField.GetValue(handTracking) : false;
            
            if (!enabled)
            {
                return new LatencyData { valid = false };
            }
            
            return new LatencyData
            {
                current = currentField != null ? (float)currentField.GetValue(handTracking) : 0f,
                average = avgField != null ? (float)avgField.GetValue(handTracking) : 0f,
                min = minField != null ? (float)minField.GetValue(handTracking) : 0f,
                max = maxField != null ? (float)maxField.GetValue(handTracking) : 0f,
                valid = enabled && currentField != null
            };
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"LatencyMetricsUI: Error accessing hand tracking latency data: {e.Message}");
            return new LatencyData { valid = false };
        }
    }
    
    void UpdateUI()
    {
        UpdateCameraLatencyUI();
        UpdateHandTrackingLatencyUI();
        UpdateOverallStatusUI();
        UpdateDetailsUI();
    }
    
    void UpdateCameraLatencyUI()
    {
        if (cameraLatencyText == null) return;
        
        if (cameraLatency.valid && cameraLatency.current > 0)
        {
            cameraLatencyText.text = $"Camera: {cameraLatency.current:F1}ms";
            cameraLatencyText.color = GetLatencyColor(cameraLatency.current);
        }
        else if (cameraSubscriber != null)
        {
            cameraLatencyText.text = "Camera: No Data";
            cameraLatencyText.color = noDataColor;
        }
        else
        {
            cameraLatencyText.text = "Camera: Not Found";
            cameraLatencyText.color = noDataColor;
        }
    }
    
    void UpdateHandTrackingLatencyUI()
    {
        if (handTrackingLatencyText == null) return;
        
        if (handLatency.valid && handLatency.current > 0)
        {
            handTrackingLatencyText.text = $"Hand Tracking: {handLatency.current:F1}ms";
            handTrackingLatencyText.color = GetLatencyColor(handLatency.current);
        }
        else if (handTracking != null)
        {
            handTrackingLatencyText.text = "Hand Tracking: No Data";
            handTrackingLatencyText.color = noDataColor;
        }
        else
        {
            handTrackingLatencyText.text = "Hand Tracking: Not Found";
            handTrackingLatencyText.color = noDataColor;
        }
    }
    
    void UpdateOverallStatusUI()
    {
        if (overallStatusText == null) return;
        
        int activeComponents = 0;
        float totalLatency = 0f;
        Color statusColor = excellentColor;
        
        if (cameraLatency.valid && cameraLatency.current > 0)
        {
            activeComponents++;
            totalLatency += cameraLatency.current;
        }
        
        if (handLatency.valid && handLatency.current > 0)
        {
            activeComponents++;
            totalLatency += handLatency.current;
        }
        
        if (activeComponents > 0)
        {
            float avgLatency = totalLatency / activeComponents;
            statusColor = GetLatencyColor(avgLatency);
            
            string status = avgLatency <= excellentThreshold ? "Excellent" : 
                           avgLatency <= goodThreshold ? "Good" : "Poor";
            
            overallStatusText.text = $"System: {status} ({avgLatency:F1}ms avg)";
        }
        else
        {
            overallStatusText.text = "System: No Active Metrics";
            statusColor = noDataColor;
        }
        
        overallStatusText.color = statusColor;
    }
    
    void UpdateDetailsUI()
    {
        if (detailsText == null || !showDetailedBreakdown) return;
        
        System.Text.StringBuilder details = new System.Text.StringBuilder();
        
        // Camera details
        if (cameraLatency.valid)
        {
            details.AppendLine($"Camera Latency:");
            details.AppendLine($"  Current: {cameraLatency.current:F1}ms");
            details.AppendLine($"  Min/Avg/Max: {cameraLatency.min:F1}/{cameraLatency.average:F1}/{cameraLatency.max:F1}ms");
            details.AppendLine();
        }
        
        // Hand tracking details
        if (handLatency.valid)
        {
            details.AppendLine($"Hand Tracking Latency:");
            details.AppendLine($"  Current: {handLatency.current:F1}ms");
            details.AppendLine($"  Min/Avg/Max: {handLatency.min:F1}/{handLatency.average:F1}/{handLatency.max:F1}ms");
            details.AppendLine();
        }
        
        // System info
        details.AppendLine($"Update Rate: {updateFrequency:F1}Hz");
        details.AppendLine($"Frame: {frameCount}");
        
        if (details.Length == 0)
        {
            details.AppendLine("No detailed metrics available.");
        }
        
        detailsText.text = details.ToString().TrimEnd();
        
        // Color code details based on overall performance
        bool hasValidData = cameraLatency.valid || handLatency.valid;
        detailsText.color = hasValidData ? Color.white : noDataColor;
    }
    
    Color GetLatencyColor(float latencyMs)
    {
        if (latencyMs <= excellentThreshold)
            return excellentColor;
        else if (latencyMs <= goodThreshold)
            return goodColor;
        else
            return poorColor;
    }
    
    // Public methods for external access
    public bool IsCameraLatencyValid => cameraLatency.valid;
    public bool IsHandTrackingLatencyValid => handLatency.valid;
    public float GetCameraLatency() => cameraLatency.valid ? cameraLatency.current : -1f;
    public float GetHandTrackingLatency() => handLatency.valid ? handLatency.current : -1f;
    
    // Method to manually refresh component references
    [System.Obsolete("Use RefreshComponents() instead")]
    public void RefreshComponentReferences()
    {
        RefreshComponents();
    }
    
    public void RefreshComponents()
    {
        FindLatencyComponents();
    }
    
    // Public methods for runtime canvas control
    public void SetWorldPosition(Vector3 newPosition)
    {
        worldPosition = newPosition;
        if (parentCanvas != null)
        {
            parentCanvas.transform.position = worldPosition;
        }
    }
    
    public void SetWorldRotation(Vector3 newRotation)
    {
        worldRotation = newRotation;
        if (parentCanvas != null)
        {
            parentCanvas.transform.rotation = Quaternion.Euler(worldRotation);
        }
    }
    
    public void SetCanvasScale(float newScale)
    {
        canvasScale = newScale;
        if (parentCanvas != null)
        {
            parentCanvas.transform.localScale = Vector3.one * canvasScale;
        }
    }
    
    public void ToggleFaceCamera(bool enabled)
    {
        faceCamera = enabled;
    }
    
    // Text size control methods
    public void ApplyTextSizes()
    {
        if (cameraLatencyText != null)
            cameraLatencyText.fontSize = mainTextSize;
            
        if (handTrackingLatencyText != null)
            handTrackingLatencyText.fontSize = mainTextSize;
            
        if (overallStatusText != null)
            overallStatusText.fontSize = statusTextSize;
            
        if (detailsText != null)
            detailsText.fontSize = detailsTextSize;
            
        Debug.Log($"LatencyMetricsUI: Applied text sizes - Main: {mainTextSize}, Status: {statusTextSize}, Details: {detailsTextSize}");
    }
    
    public void SetMainTextSize(float size)
    {
        mainTextSize = size;
        if (autoApplyTextSizes) ApplyTextSizes();
    }
    
    public void SetStatusTextSize(float size)
    {
        statusTextSize = size;
        if (autoApplyTextSizes) ApplyTextSizes();
    }
    
    public void SetDetailsTextSize(float size)
    {
        detailsTextSize = size;
        if (autoApplyTextSizes) ApplyTextSizes();
    }
    
    // Gizmo drawing for scene view
    void OnDrawGizmosSelected()
    {
        if (!showPositionGizmo) return;
        
        // Draw a wireframe cube at the world position
        Gizmos.color = Color.cyan;
        Gizmos.matrix = Matrix4x4.TRS(worldPosition, Quaternion.Euler(worldRotation), Vector3.one * canvasScale * 100f);
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(19.2f, 10.8f, 0.1f)); // Represents canvas size
        
        // Draw direction indicator
        Gizmos.color = Color.red;
        Gizmos.DrawRay(Vector3.zero, Vector3.forward * 2f);
        
        // Reset matrix
        Gizmos.matrix = Matrix4x4.identity;
        
        // Draw position label
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(worldPosition, 0.05f);
    }
}
