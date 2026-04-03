using UnityEngine;
using TMPro;
using UnityEngine.XR.Hands;
using UnityEngine.XR.Management;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

using UnityEngine.Events;

using UnityEngine.XR.Hands.Samples.GestureSample;

public class HandTipHUD : MonoBehaviour
{
    // Hand gesture for calibrating arm length
    [Header("Calibrate Arm Length Gesture")]
    [SerializeField] StaticHandGesture gesture;

    // Hand gestures for open/close status
    [Header("Open Left Hand Gesture")]
    [SerializeField] StaticHandGesture openHandGestureLeft;

    [Header("Close Left Hand Gesture")]
    [SerializeField] StaticHandGesture closeHandGestureLeft;

    [Header("Open Right Hand Gesture")]
    [SerializeField] StaticHandGesture openHandGestureRight; 

    [Header("Close Right Hand Gesture")]
    [SerializeField] StaticHandGesture closeHandGestureRight; 
    
    // If the gesture GameObject is inactive, try to activate it
    [Tooltip("If the gesture GameObject is inactive, try to activate it (only affects activeSelf; cannot override inactive parents).")]
    [SerializeField] bool autoActivateGestureGameObject = true;

    // Event for when the calibrate arm length gesture is performed
    [Tooltip("Invoked once when the gesture is detected (performed).")]
    [SerializeField] UnityEvent onHandShapeDetectedTrue;

    bool _gestureActive;

    // XR Origin
    [Header("Assign in Inspector")]
    public Transform xrOrigin;      
    public Transform xrCamera;      
    public TMP_Text textLeft;
    public TMP_Text textRight;

    // ROS stuff
    ROSConnection ros;
    public string topicName = "pos_rot"; // this will be combined with _left and _right for the two hands

    // Timing and calibration
    public float publishMessageFrequency = 0.1f;
    float timeElapsed = 0f;
    float timeSinceCalibrate = 0f;

    // Default arm length for the user. This will be changed with the calibration
    float userArmLength = 0.8f; // m

    // Hard coded arm length from the G1 humanoid
    float g1ArmLength = 0.51f;  // m

    // Open hand status
    bool _openHandActiveLeft = false;
    bool _openHandActiveRight = false;

    // Joint to show and decimal places for display
    [Header("Settings")]
    public XRHandJointID jointToShow = XRHandJointID.IndexTip;
    public int decimals = 3;

    // ========== LATENCY MEASUREMENT ==========
    [Header("Latency Measurement")]
    [SerializeField] private bool enableLatencyMeasurement = true;
    [SerializeField] private int latencySampleSize = 100;
    [SerializeField] private bool logLatencyToConsole = true;
    
    // Latency tracking variables
    private float currentLatencyMs = 0f;
    private float minLatencyMs = float.MaxValue;
    private float maxLatencyMs = 0f;
    private float avgLatencyMs = 0f;
    
    // For rolling average calculation
    private double latencySum = 0.0;
    private int latencyCount = 0;
    private int frameCount = 0;
    
    // Timing measurement points
    private double handPoseStartTime = 0.0;
    private double handProcessStartTime = 0.0;
    private double handProcessEndTime = 0.0;
    private double rosPublishStartTime = 0.0;
    private double rosPublishEndTime = 0.0;
    
    // Component latencies
    private float handProcessingLatencyMs = 0f;
    private float rosPublishingLatencyMs = 0f;
    
    // ROS Message History for when hand is not tracked
    private PoseStampedMsg prevLeftPoseStamped = new PoseStampedMsg();
    private PoseStampedMsg prevRightPoseStamped = new PoseStampedMsg();

    private string latencyDebugStatus = "Waiting for hand tracking data...";

    XRHandSubsystem _hands;

    void OnEnable()
    {
        Debug.Log("HTHUD: OnEnable");

        // Check if gesture is assigned
        if (gesture == null)
        {
            Debug.LogWarning($"[{nameof(HandTipHUD)}] No StaticHandGesture assigned. Hand-shape detection will not fire.");
            return;
        }

        // Log the initial state of the gesture GameObject and component
        Debug.Log($"HTHUD: gesture GO activeInHierarchy={gesture.gameObject.activeInHierarchy} activeSelf={gesture.gameObject.activeSelf} enabled={gesture.enabled}");

        // Activate the gesture GameObject
        if (autoActivateGestureGameObject && !gesture.gameObject.activeSelf)
        {
            gesture.gameObject.SetActive(true);
            Debug.Log($"HTHUD: gesture GO was inactive (activeSelf=false). SetActive(true) called. Now activeInHierarchy={gesture.gameObject.activeInHierarchy} activeSelf={gesture.gameObject.activeSelf}");
        }

        // Final check for activeInHierarchy
        if (!gesture.gameObject.activeInHierarchy)
        {
            Debug.LogError("HTHUD: StaticHandGesture is NOT active in the hierarchy (activeInHierarchy=false). " +
                           "It will never detect and will never fire events. Fix by enabling its parent GameObjects / prefab instance.");
        }

        // Subscribe to gesture events
        gesture.gesturePerformed.AddListener(OnGesturePerformed);
        gesture.gestureEnded.AddListener(OnGestureEnded);
        
        // Subscribe to open/close hand gesture events
        if (openHandGestureLeft != null)
        {
            openHandGestureLeft.gesturePerformed.AddListener(OnOpenLeftHandGesturePerformed);
        }

        if (closeHandGestureLeft != null)
        {
            closeHandGestureLeft.gesturePerformed.AddListener(OnCloseLeftHandGesturePerformed);
        }

        if (openHandGestureRight != null)
        {
            openHandGestureRight.gesturePerformed.AddListener(OnOpenRightHandGesturePerformed);
        }
        
        if (closeHandGestureRight != null)
        {
            closeHandGestureRight.gesturePerformed.AddListener(OnCloseRightHandGesturePerformed);
        }
    }

    // Unsubscribe from events when disabled to prevent memory leaks
    void OnDisable()
    {
        // Unsubscribe from gesture events
        if (gesture != null)
        {
            gesture.gesturePerformed.RemoveListener(OnGesturePerformed);
            gesture.gestureEnded.RemoveListener(OnGestureEnded);
        }

        if (openHandGestureLeft != null)
        {
            openHandGestureLeft.gesturePerformed.RemoveListener(OnOpenLeftHandGesturePerformed);
        }

        if (closeHandGestureLeft != null)
        {
            closeHandGestureLeft.gesturePerformed.RemoveListener(OnCloseLeftHandGesturePerformed);
        }
        
        if (openHandGestureRight != null)
        {
            openHandGestureRight.gesturePerformed.RemoveListener(OnOpenRightHandGesturePerformed);
        }
        
        if (closeHandGestureRight != null)
        {
            closeHandGestureRight.gesturePerformed.RemoveListener(OnCloseRightHandGesturePerformed);
        }
    }

    // When calibrate arm length gesture is performed
    void OnGesturePerformed()
    {
        // Prevent changes if the gesture is already active
        if (_gestureActive) return;
        _gestureActive = true;

        // Debug
        Debug.Log("HTHUD: OnGesturePerformed fired");
        OnHandShapeBecameTrue();
        onHandShapeDetectedTrue?.Invoke();
    }

    // When calibrate arm length gesture ends set active to false to allow future detections
    void OnGestureEnded()
    {
        _gestureActive = false;
        Debug.Log("HTHUD: OnGestureEnded fired");
    }

    void Start()
    {
        // Get the XRHandSubsystem from the active XR loader
        var loader = XRGeneralSettings.Instance?.Manager?.activeLoader;
        _hands = loader?.GetLoadedSubsystem<XRHandSubsystem>();

        // Initialize ROS publishers and debug text
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName + "_left");
        ros.RegisterPublisher<PoseStampedMsg>(topicName + "_right");
        ros.RegisterPublisher<BoolMsg>(topicName + "_left_status");
        ros.RegisterPublisher<BoolMsg>(topicName + "_right_status");
        Debug.Log($"ROS Publisher initialized on topics: {topicName}_left, {topicName}_right, {topicName}_left_status, and {topicName}_right_status");

        // Display initial status in debug text
        string status = _hands != null ? "Hands subsystem: OK" : "Hands subsystem: NOT FOUND";
        if (textLeft != null) textLeft.text = status;
        if (textRight != null) textRight.text = status;

        // Latency measurement initialization
        if (enableLatencyMeasurement)
        {
            Debug.Log("HTHUD: Latency measurement enabled");
            latencyDebugStatus = "Latency tracking initialized";
        }

        // Final debug check for gesture assignment
        Debug.Log($"HTHUD: gesture assigned? {(gesture != null)}");
    }

    // Run loop
    void Update()
    {
        // Update timers
        timeElapsed += Time.deltaTime;
        timeSinceCalibrate += Time.deltaTime;

        // Only publish at the specified frequency
        if (timeElapsed < publishMessageFrequency) return;

        // Ensure that all necessary components are assigned before proceeding
        if (_hands == null || xrOrigin == null || xrCamera == null) return;
        if (textLeft == null || textRight == null) return;

        // Start latency measurement
        if (enableLatencyMeasurement)
        {
            handPoseStartTime = Time.realtimeSinceStartupAsDouble;
            handProcessStartTime = handPoseStartTime;
        }

        // Get joint positions and orientations for both hands
        (Vector3 leftJoints, Quaternion leftRot) = FormatJointFromHeadset(_hands.leftHand);
        (Vector3 rightJoints, Quaternion rightRot) = FormatJointFromHeadset(_hands.rightHand);

        // Rotate z-axis orientation by 90 degrees counterclockwise
        Quaternion zRotation = Quaternion.AngleAxis(-90f, Vector3.forward);
        leftRot = zRotation * leftRot;
        rightRot = zRotation * rightRot;

        // Allow calibration if it's been more than 5 seconds since the last calibration
        if (_gestureActive && timeSinceCalibrate > 5f)
        {
            // Reset timer and update user arm length
            timeSinceCalibrate = 0f;
            userArmLength = leftJoints.z; // Update arm length
            if (userArmLength < 0.2f) userArmLength = 0.2f;
            Debug.Log("HTHUD: Hand shape detected (gesture active)");
        }

        // Project user arm length to match G1 arm length
        leftJoints *= g1ArmLength / userArmLength;
        rightJoints *= g1ArmLength / userArmLength;
        
        // Mark end of hand processing
        if (enableLatencyMeasurement)
        {
            handProcessEndTime = Time.realtimeSinceStartupAsDouble;
            handProcessingLatencyMs = (float)((handProcessEndTime - handProcessStartTime) * 1000.0);
        }

        // Format joint data for display within Unity and terminal
        string leftJointsStr = $"{leftJoints.x.ToString($"F{decimals}")}, " +
                               $"{leftJoints.z.ToString($"F{decimals}")}, " +
                               $"{leftJoints.y.ToString($"F{decimals}")}";
        string rightJointsStr = $"{rightJoints.x.ToString($"F{decimals}")}, " +
                                $"{rightJoints.z.ToString($"F{decimals}")}, " +
                                $"{rightJoints.y.ToString($"F{decimals}")}";

        // Text for display in Unity
        textLeft.text = $"L tracked: {_hands.leftHand.isTracked}\n{leftJointsStr}";
        textRight.text = $"R tracked: {_hands.rightHand.isTracked}\n{rightJointsStr}";

        // Terminal logging
        Debug.Log($"[{Time.time:F1}s] L: {leftJointsStr} R: {rightJointsStr}");
        Debug.Log($"g1ArmLength={g1ArmLength} userArmLength={userArmLength}");

        // Ensure ROS connection is established
        if (ros == null || !ros.HasConnectionThread)
        {
            Debug.LogError($"[{Time.time:F1}s] ROS connection not established! Check ROS Settings (Robotics > ROS Settings).");
            return;
        }
        
        // Start ROS publishing timing
        if (enableLatencyMeasurement)
        {
            rosPublishStartTime = Time.realtimeSinceStartupAsDouble;
        }

        var currentTime = Time.time;

        // Create header with current time for ROS messages
        var header = new HeaderMsg
        {
            stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
            {
                sec = (int)currentTime,
                nanosec = (uint)((currentTime - (int)currentTime) * 1e9)
            },
            frame_id = "camera"
        };

        // Publish left hand pose
        var leftPoseStamped = new PoseStampedMsg
        {
            header = header,
            pose = new PoseMsg
            {
                position = new PointMsg(leftJoints.x, leftJoints.z, -leftJoints.y),
                orientation = new QuaternionMsg(leftRot.x, leftRot.z, -leftRot.y, -leftRot.w)
            }
        };

        // If hand is not tracked, use the last measurement
        if (_hands.leftHand.isTracked)
        {
            prevLeftPoseStamped = leftPoseStamped; // Update history only if currently tracked
        }
        else
        {
            leftPoseStamped.pose = prevLeftPoseStamped.pose; // Use last known pose if not currently tracked
        }

        // Publish left hand pose and status
        ros.Publish(topicName + "_left", leftPoseStamped);
        var leftHandStatus = new BoolMsg { data = _openHandActiveLeft };
        ros.Publish(topicName + "_left_status", leftHandStatus);

        // Publish right hand pose
        var rightPoseStamped = new PoseStampedMsg
        {
            header = header,
            pose = new PoseMsg
            {
                position = new PointMsg(rightJoints.x, rightJoints.z, -rightJoints.y),
                orientation = new QuaternionMsg(rightRot.x, rightRot.z, -rightRot.y, -rightRot.w)
            }
        };

        // If hand is not tracked, use the last measurement
        if (_hands.rightHand.isTracked)
        {
            prevRightPoseStamped = rightPoseStamped; // Update history only if currently tracked
        }
        else
        {
            rightPoseStamped.pose = prevRightPoseStamped.pose; // Use last known pose if not currently tracked
        }

        // Publish right hand pose and status
        ros.Publish(topicName + "_right", rightPoseStamped);
        var rightHandStatus = new BoolMsg { data = _openHandActiveRight };
        ros.Publish(topicName + "_right_status", rightHandStatus);
        
        // Complete latency measurement
        if (enableLatencyMeasurement)
        {
            rosPublishEndTime = Time.realtimeSinceStartupAsDouble;
            rosPublishingLatencyMs = (float)((rosPublishEndTime - rosPublishStartTime) * 1000.0);
            currentLatencyMs = (float)((rosPublishEndTime - handPoseStartTime) * 1000.0);
            
            UpdateLatencyStats();
            
            frameCount++;
            if (logLatencyToConsole && frameCount % 30 == 0) // Log every 30 frames
            {
                LogLatencyStats();
            }
        }

        timeElapsed = 0;
    }

    // Event handlers for hand shape and open/close gestures
    void OnHandShapeBecameTrue()
    {
        Debug.Log("HTHUD: Hand shape/pose detected (StaticHandGesture performed).");
        // Put your action here.
    }
    
    void OnOpenRightHandGesturePerformed()
    {
        _openHandActiveRight = true;
        Debug.Log("HTHUD: Open right hand gesture performed - _openHandActiveRight set to true");
    }

    void OnOpenLeftHandGesturePerformed()
    {
        _openHandActiveLeft = true;
        Debug.Log("HTHUD: Open left hand gesture performed - _openHandActiveLeft set to true");
    }

    void OnCloseLeftHandGesturePerformed()
    {
        _openHandActiveLeft = false;
        Debug.Log("HTHUD: Close left hand gesture performed - _openHandActiveLeft set to false");
    }
    
    void OnCloseRightHandGesturePerformed()
    {
        _openHandActiveRight = false;
        Debug.Log("HTHUD: Close right hand gesture performed - _openHandActiveRight set to false");
    }

    // Get joint position and orientation relative to headset, or return zeros if not tracked
    (Vector3, Quaternion) FormatJointFromHeadset(XRHand hand)
    {
        if (!hand.isTracked) return (Vector3.zero, Quaternion.identity);

        var joint = hand.GetJoint(jointToShow);
        if (!joint.TryGetPose(out Pose pose)) return (Vector3.zero, Quaternion.identity);

        // Transform joint position from world space to headset-relative space
        Vector3 jointWorld = xrOrigin.TransformPoint(pose.position);
        Quaternion rot = pose.rotation;
        Vector3 fromHead = xrCamera.InverseTransformPoint(jointWorld);

        return (fromHead, rot);
    }
    
    // Update latency statistics with the current frame's latency measurement
    void UpdateLatencyStats()
    {
        if (currentLatencyMs > 0 && currentLatencyMs < 10000) // Sanity check
        {
            latencySum += currentLatencyMs;
            latencyCount++;
            
            // Update min and max latency
            if (currentLatencyMs < minLatencyMs)
                minLatencyMs = currentLatencyMs;
            if (currentLatencyMs > maxLatencyMs)
                maxLatencyMs = currentLatencyMs;
                
            // Keep rolling average within sample size
            if (latencyCount > latencySampleSize)
            {
                latencySum -= avgLatencyMs;
                latencyCount--;
            }
            
            avgLatencyMs = latencyCount > 0 ? (float)(latencySum / latencyCount) : 0f;
            
            // Print to terminal
            latencyDebugStatus = $"Total: {currentLatencyMs:F1}ms | Avg: {avgLatencyMs:F1}ms | Min: {minLatencyMs:F1}ms | Max: {maxLatencyMs:F1}ms";
        }
    }

    // Log latency statistics to the console with component breakdown and hand tracking status
    void LogLatencyStats()
    {
        Debug.Log($"[HAND TRACKING LATENCY] Frame {frameCount}: {latencyDebugStatus}");
        Debug.Log($"[HAND TRACKING BREAKDOWN] Processing: {handProcessingLatencyMs:F1}ms | ROS Publish: {rosPublishingLatencyMs:F1}ms");
        
        bool leftTracked = _hands.leftHand.isTracked;
        bool rightTracked = _hands.rightHand.isTracked;
        Debug.Log($"[HAND TRACKING STATUS] Left: {leftTracked} | Right: {rightTracked} | Publish Rate: {(1000f / publishMessageFrequency):F0}Hz");
    }
}
