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
    [Header("Calibrate Arm Length Gesture")]
    [SerializeField] StaticHandGesture gesture; // Assign the StaticHandGesture component that detects your hand pose/shape

    [Header("Open Left Hand Gesture")]
    [SerializeField] StaticHandGesture openHandGestureLeft;

    [Header("Close Left Hand Gesture")]
    [SerializeField] StaticHandGesture closeHandGestureLeft;

    [Header("Open Right Hand Gesture")]
    [SerializeField] StaticHandGesture openHandGestureRight; 

    [Header("Close Right Hand Gesture")]
    [SerializeField] StaticHandGesture closeHandGestureRight; 
    
    [Tooltip("If the gesture GameObject is inactive, try to activate it (only affects activeSelf; cannot override inactive parents).")]
    [SerializeField] bool autoActivateGestureGameObject = true;

    [Tooltip("Invoked once when the gesture is detected (performed).")]
    [SerializeField] UnityEvent onHandShapeDetectedTrue;

    bool _gestureActive;

    [Header("Assign in Inspector")]
    public Transform xrOrigin;      // XR Origin (tracking-to-world)
    public Transform xrCamera;      // XR Origin/Camera Offset/Main Camera
    public TMP_Text textLeft;
    public TMP_Text textRight;

    // ROS stuff
    ROSConnection ros;
    public string topicName = "pos_rot";
    public float publishMessageFrequency = 0.1f;
    float timeElapsed = 0f;
    float timeSinceCalibrate = 0f;

    float userArmLength = 0.8f; // meters
    float g1ArmLength = 0.45f;  // meters

    bool _openHandActiveLeft = false;
    bool _openHandActiveRight = false;

    [Header("Settings")]
    public XRHandJointID jointToShow = XRHandJointID.IndexTip;
    public int decimals = 3;

    XRHandSubsystem _hands;

    void OnEnable()
    {
        Debug.Log("HTHUD: OnEnable");

        if (gesture == null)
        {
            Debug.LogWarning($"[{nameof(HandTipHUD)}] No StaticHandGesture assigned. Hand-shape detection will not fire.");
            return;
        }

        // Key diagnostic: if this prints activeInHierarchy=False, the gesture will NOT run and will never fire events.
        Debug.Log($"HTHUD: gesture GO activeInHierarchy={gesture.gameObject.activeInHierarchy} activeSelf={gesture.gameObject.activeSelf} enabled={gesture.enabled}");

        // Try to activate the gesture GameObject if it's just disabled at the object level.
        if (autoActivateGestureGameObject && !gesture.gameObject.activeSelf)
        {
            gesture.gameObject.SetActive(true);
            Debug.Log($"HTHUD: gesture GO was inactive (activeSelf=false). SetActive(true) called. Now activeInHierarchy={gesture.gameObject.activeInHierarchy} activeSelf={gesture.gameObject.activeSelf}");
        }

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

    void OnDisable()
    {
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

    void OnGesturePerformed()
    {
        if (_gestureActive) return;
        _gestureActive = true;

        Debug.Log("HTHUD: OnGesturePerformed fired");
        OnHandShapeBecameTrue();
        onHandShapeDetectedTrue?.Invoke();
    }

    void OnGestureEnded()
    {
        _gestureActive = false;
        Debug.Log("HTHUD: OnGestureEnded fired");
    }

    void Start()
    {
        var loader = XRGeneralSettings.Instance?.Manager?.activeLoader;
        _hands = loader?.GetLoadedSubsystem<XRHandSubsystem>();

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName + "_left");
        ros.RegisterPublisher<PoseStampedMsg>(topicName + "_right");
        ros.RegisterPublisher<BoolMsg>(topicName + "_left_status");
        ros.RegisterPublisher<BoolMsg>(topicName + "_right_status");
        Debug.Log($"ROS Publisher initialized on topics: {topicName}_left, {topicName}_right, {topicName}_left_status, and {topicName}_right_status");

        string status = _hands != null ? "Hands subsystem: OK" : "Hands subsystem: NOT FOUND";
        if (textLeft != null) textLeft.text = status;
        if (textRight != null) textRight.text = status;

        Debug.Log($"HTHUD: gesture assigned? {(gesture != null)}");
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        timeSinceCalibrate += Time.deltaTime;
        if (timeElapsed < publishMessageFrequency) return;

        if (_hands == null || xrOrigin == null || xrCamera == null) return;
        if (textLeft == null || textRight == null) return;

        (Vector3 leftJoints, Quaternion leftRot) = FormatJointFromHeadset(_hands.leftHand);
        (Vector3 rightJoints, Quaternion rightRot) = FormatJointFromHeadset(_hands.rightHand);

        // Rotate z-axis orientation by 90 degrees counterclockwise
        Quaternion zRotation = Quaternion.AngleAxis(-90f, Vector3.forward);
        leftRot = zRotation * leftRot;
        rightRot = zRotation * rightRot;

        if (_gestureActive && timeSinceCalibrate > 5f)
        {
            timeSinceCalibrate = 0f;
            userArmLength = leftJoints.z;
            //leftJoints.z = 0;
            Debug.Log("HTHUD: Hand shape detected (gesture active)");
        }

        leftJoints *= g1ArmLength / userArmLength;
        rightJoints *= g1ArmLength / userArmLength;

        string leftJointsStr = $"{leftJoints.x.ToString($"F{decimals}")}, " +
                               $"{leftJoints.z.ToString($"F{decimals}")}, " +
                               $"{leftJoints.y.ToString($"F{decimals}")}";
        string rightJointsStr = $"{rightJoints.x.ToString($"F{decimals}")}, " +
                                $"{rightJoints.z.ToString($"F{decimals}")}, " +
                                $"{rightJoints.y.ToString($"F{decimals}")}";

        textLeft.text = $"L tracked: {_hands.leftHand.isTracked}\n{leftJointsStr}";
        textRight.text = $"R tracked: {_hands.rightHand.isTracked}\n{rightJointsStr}";

        Debug.Log($"[{Time.time:F1}s] L: {leftJointsStr} R: {rightJointsStr}");
        Debug.Log($"g1ArmLength={g1ArmLength} userArmLength={userArmLength}");

        if (ros == null || !ros.HasConnectionThread)
        {
            Debug.LogError($"[{Time.time:F1}s] ROS connection not established! Check ROS Settings (Robotics > ROS Settings).");
            return;
        }

        var currentTime = Time.time;
        var header = new HeaderMsg
        {
            stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
            {
                sec = (int)currentTime,
                nanosec = (uint)((currentTime - (int)currentTime) * 1e9)
            },
            frame_id = "camera"
        };

        var leftPoseStamped = new PoseStampedMsg
        {
            header = header,
            pose = new PoseMsg
            {
                position = new PointMsg(leftJoints.x, leftJoints.z, -leftJoints.y),
                orientation = new QuaternionMsg(leftRot.x, leftRot.z, -leftRot.y, -leftRot.w)
            }
        };
        ros.Publish(topicName + "_left", leftPoseStamped);

        var leftHandStatus = new BoolMsg { data = _openHandActiveLeft };
        ros.Publish(topicName + "_left_status", leftHandStatus);

        var rightPoseStamped = new PoseStampedMsg
        {
            header = header,
            pose = new PoseMsg
            {
                position = new PointMsg(rightJoints.x, rightJoints.z, -rightJoints.y),
                orientation = new QuaternionMsg(rightRot.x, rightRot.z, -rightRot.y, -rightRot.w)
            }
        };
        ros.Publish(topicName + "_right", rightPoseStamped);

        var rightHandStatus = new BoolMsg { data = _openHandActiveRight };
        ros.Publish(topicName + "_right_status", rightHandStatus);

        timeElapsed = 0;
    }

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

    (Vector3, Quaternion) FormatJointFromHeadset(XRHand hand)
    {
        if (!hand.isTracked) return (Vector3.zero, Quaternion.identity);

        var joint = hand.GetJoint(jointToShow);
        if (!joint.TryGetPose(out Pose pose)) return (Vector3.zero, Quaternion.identity);

        Vector3 jointWorld = xrOrigin.TransformPoint(pose.position);
        Quaternion rot = pose.rotation;
        Vector3 fromHead = xrCamera.InverseTransformPoint(jointWorld);

        return (fromHead, rot);
    }
}
