using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class CameraImageSubscriberDebug : MonoBehaviour
{
    public string imageTopic = "/camera/color/image_raw";
    public Renderer displayRenderer;

    private Texture2D cameraTexture;
    private int messageCount = 0;
    private bool testTextureApplied = false;

    private float lastImageTime = -1f;
    private float smoothedDeltaTime = -1f;

    [SerializeField] private int minTargetFps = 10;
    [SerializeField] private int maxTargetFps = 120;

    // ========== LATENCY MEASUREMENT ==========
    [Header("Latency Measurement")]
    [SerializeField] private bool enableLatencyMeasurement = true;
    [SerializeField] private int latencySampleSize = 100;

    private float currentLatencyMs = 0f;
    private float minLatencyMs = float.MaxValue;
    private float maxLatencyMs = 0f;
    private float avgLatencyMs = 0f;

    private double latencySum = 0.0;
    private int latencyCount = 0;

    private double rosToUnityTimeOffset = 0.0;
    private bool timeOffsetCalculated = false;

    private int calibrationSamples = 10;
    private List<double> calibrationOffsets = new List<double>();

    private double frameCaptureTime = 0.0;
    private double frameReceiveTime = 0.0;
    private double frameProcessTime = 0.0;
    private double frameDisplayTime = 0.0;

    private float networkLatencyMs = 0f;
    private float processingLatencyMs = 0f;
    private float renderLatencyMs = 0f;

    private string latencyDebugStatus = "Waiting for data...";
    private bool rosTimestampValid = false;
    private double firstRosTimestamp = -1.0;
    private bool usingMonotonicTime = false;

    // Optional drop policy
    [Header("Drop Policy")]
    [SerializeField] private bool dropStaleFrames = true;
    [SerializeField] private float maxAcceptableTotalLatencyMs = 300f;

    // Encoding conversion scratch
    private byte[] scratchRgb;

    // Cached GUI style
    private GUIStyle guiStyle;

    void Start()
    {
        Debug.Log("=== Camera Image Subscriber DEBUG Starting ===");

        if (displayRenderer == null)
        {
            displayRenderer = GetComponent<Renderer>();

            if (displayRenderer == null)
            {
                Debug.LogError("No Renderer found!");
                enabled = false;
                return;
            }
        }

        Debug.Log($" Renderer found: {displayRenderer.gameObject.name}");

        // TEST texture (red)
        Debug.Log("TEST 1: Applying RED test texture...");

        Texture2D redTexture = new Texture2D(256, 256, TextureFormat.RGB24, false);
        Color[] redPixels = new Color[256 * 256];

        for (int i = 0; i < redPixels.Length; i++)
            redPixels[i] = Color.red;

        redTexture.SetPixels(redPixels);
        redTexture.Apply(false, false);

        Shader unlitShader = Shader.Find("Unlit/Texture");

        if (unlitShader == null)
        {
            Debug.LogWarning(" Unlit/Texture shader not found, using existing material");
            displayRenderer.material.mainTexture = redTexture;
        }
        else
        {
            displayRenderer.material = new Material(unlitShader);
            displayRenderer.material.mainTexture = redTexture;
        }

        testTextureApplied = true;

        // Flip vertically
        displayRenderer.material.mainTextureScale = new Vector2(1f, -1f);
        displayRenderer.material.mainTextureOffset = new Vector2(0f, 1f);

        guiStyle = new GUIStyle
        {
            fontSize = 28,
            fontStyle = FontStyle.Bold,
            normal = { textColor = Color.yellow }
        };

        ROSConnection ros = ROSConnection.GetOrCreateInstance();

        Debug.Log($" ROS Connection: {ros.RosIPAddress}:{ros.RosPort}");

        ros.Subscribe<ImageMsg>(imageTopic, UpdateCameraImage);

        Debug.Log($" Subscribed to: {imageTopic}");
        Debug.Log(" === Waiting for ROS images (Quad should be RED while waiting) ===");

        if (enableLatencyMeasurement)
        {
            Debug.Log("=== LATENCY MEASUREMENT ENABLED ===");
            Debug.Log($" Need {calibrationSamples} frames for clock calibration");

            latencyDebugStatus = "Waiting for first frame...";
        }
    }

    void UpdateCameraImage(ImageMsg imageMsg)
    {
        messageCount++;

        frameReceiveTime = GetCurrentTimeSeconds();

        if (enableLatencyMeasurement)
        {
            double rosTimestampSec = imageMsg.header.stamp.sec;
            double rosTimestampNsec = imageMsg.header.stamp.nanosec;
            double rosTimestamp = rosTimestampSec + (rosTimestampNsec / 1e9);

            frameCaptureTime = rosTimestamp;

            if (messageCount == 1)
            {
                Debug.Log($"=== FIRST FRAME META ===");
                Debug.Log($" w={imageMsg.width} h={imageMsg.height} enc={imageMsg.encoding} step={imageMsg.step} len={imageMsg.data.Length}");
                Debug.Log($" ROS stamp: {rosTimestampSec}.{rosTimestampNsec:D9} (combined {rosTimestamp:F6})");
                Debug.Log($" Unity receive time: {frameReceiveTime:F6}");

                if (rosTimestamp <= 0)
                {
                    Debug.LogError(" ❌ ROS TIMESTAMP INVALID (<=0). Latency stats will be disabled.");
                    rosTimestampValid = false;
                    latencyDebugStatus = "ERROR: ROS timestamp invalid";
                }
                else
                {
                    firstRosTimestamp = rosTimestamp;
                    rosTimestampValid = true;

                    usingMonotonicTime = rosTimestamp < 1000000000;

                    calibrationOffsets.Clear();
                    timeOffsetCalculated = false;

                    latencyDebugStatus = $"Calibrating (0/{calibrationSamples})...";
                }
            }

            if (rosTimestampValid && !timeOffsetCalculated && calibrationOffsets.Count < calibrationSamples)
            {
                double offset = frameReceiveTime - frameCaptureTime;

                calibrationOffsets.Add(offset);

                latencyDebugStatus = $"Calibrating ({calibrationOffsets.Count}/{calibrationSamples})...";

                if (calibrationOffsets.Count == calibrationSamples)
                {
                    calibrationOffsets.Sort();

                    rosToUnityTimeOffset = calibrationOffsets[calibrationSamples / 2];

                    timeOffsetCalculated = true;
                    latencyDebugStatus = "✅ Measuring latency";

                    Debug.Log($"✅ TIME CALIBRATION COMPLETE. Offset={rosToUnityTimeOffset * 1000:F2} ms");
                }
            }

            if (timeOffsetCalculated && rosTimestampValid)
            {
                double adjustedDiff = frameReceiveTime - frameCaptureTime - rosToUnityTimeOffset;
                networkLatencyMs = (float)(adjustedDiff * 1000.0);
            }
        }

        float now = Time.realtimeSinceStartup;

        if (lastImageTime > 0f)
        {
            float delta = now - lastImageTime;

            if (delta > 0f)
            {
                smoothedDeltaTime = smoothedDeltaTime < 0f
                    ? delta
                    : Mathf.Lerp(smoothedDeltaTime, delta, 0.1f);

                float fps = 1f / smoothedDeltaTime;

                int target = Mathf.Clamp(Mathf.RoundToInt(fps), minTargetFps, maxTargetFps);

                QualitySettings.vSyncCount = 0;
                Application.targetFrameRate = target;
            }
        }

        lastImageTime = now;

        try
        {
            double processStartTime = GetCurrentTimeSeconds();

            int width = (int)imageMsg.width;
            int height = (int)imageMsg.height;

            if (cameraTexture == null || cameraTexture.width != width || cameraTexture.height != height)
            {
                cameraTexture = new Texture2D(width, height, TextureFormat.RGB24, false);

                cameraTexture.wrapMode = TextureWrapMode.Clamp;
                cameraTexture.filterMode = FilterMode.Bilinear;

                displayRenderer.material.mainTexture = cameraTexture;

                Debug.Log($"✓ Texture created: {width}x{height}, format: RGB24");
            }

            string enc = (imageMsg.encoding ?? "").ToLowerInvariant();

            if (enc == "" || enc == "rgb8" || enc == "rgb24")
            {
                cameraTexture.LoadRawTextureData(imageMsg.data);
            }
            else if (enc == "bgr8")
            {
                EnsureScratch(width * height * 3);
                BgrToRgb(imageMsg.data, scratchRgb);
                cameraTexture.LoadRawTextureData(scratchRgb);
            }
            else if (enc == "rgba8")
            {
                EnsureScratch(width * height * 3);
                RgbaToRgb(imageMsg.data, scratchRgb);
                cameraTexture.LoadRawTextureData(scratchRgb);
            }
            else if (enc == "bgra8")
            {
                EnsureScratch(width * height * 3);
                BgraToRgb(imageMsg.data, scratchRgb);
                cameraTexture.LoadRawTextureData(scratchRgb);
            }
            else
            {
                if (messageCount == 1)
                    Debug.LogWarning($"⚠️ Unknown encoding '{imageMsg.encoding}'. Trying raw upload.");

                cameraTexture.LoadRawTextureData(imageMsg.data);
            }

            cameraTexture.Apply(false, false);

            frameProcessTime = GetCurrentTimeSeconds();
            processingLatencyMs = (float)((frameProcessTime - processStartTime) * 1000.0);
        }
        catch (System.Exception e)
        {
            Debug.LogError($" EXCEPTION: {e.Message}");
            Debug.LogError($" Stack trace: {e.StackTrace}");
        }
    }

    void LateUpdate()
    {
        if (!enableLatencyMeasurement || messageCount == 0)
            return;

        frameDisplayTime = GetCurrentTimeSeconds();

        if (timeOffsetCalculated && rosTimestampValid)
        {
            double totalLatency = frameDisplayTime - frameCaptureTime - rosToUnityTimeOffset;

            currentLatencyMs = (float)(totalLatency * 1000.0);

            renderLatencyMs = (float)((frameDisplayTime - frameProcessTime) * 1000.0);

            if (currentLatencyMs > 0 && currentLatencyMs < 60000)
            {
                minLatencyMs = Mathf.Min(minLatencyMs, currentLatencyMs);
                maxLatencyMs = Mathf.Max(maxLatencyMs, currentLatencyMs);

                latencySum += currentLatencyMs;
                latencyCount++;

                if (latencyCount > latencySampleSize)
                {
                    latencySum *= 0.5;
                    latencyCount = latencySampleSize / 2;
                }

                avgLatencyMs = (float)(latencySum / latencyCount);
            }
        }
    }

    private void EnsureScratch(int n)
    {
        if (scratchRgb == null || scratchRgb.Length != n)
            scratchRgb = new byte[n];
    }

    static void BgrToRgb(byte[] src, byte[] dst)
    {
        for (int i = 0, j = 0; j < dst.Length; i += 3, j += 3)
        {
            dst[j] = src[i + 2];
            dst[j + 1] = src[i + 1];
            dst[j + 2] = src[i];
        }
    }

    static void RgbaToRgb(byte[] src, byte[] dst)
    {
        for (int i = 0, j = 0; j < dst.Length; i += 4, j += 3)
        {
            dst[j] = src[i];
            dst[j + 1] = src[i + 1];
            dst[j + 2] = src[i + 2];
        }
    }

    static void BgraToRgb(byte[] src, byte[] dst)
    {
        for (int i = 0, j = 0; j < dst.Length; i += 4, j += 3)
        {
            dst[j] = src[i + 2];
            dst[j + 1] = src[i + 1];
            dst[j + 2] = src[i];
        }
    }

    private double GetCurrentTimeSeconds()
    {
        return Time.realtimeSinceStartupAsDouble;
    }

    void OnGUI()
    {
        if (guiStyle == null) return;

        int yOffset = 10;
        int lineHeight = 40;

        guiStyle.fontSize = 28;
        guiStyle.normal.textColor = Color.yellow;

        GUI.Label(new Rect(10, yOffset, 1200, lineHeight), $"Topic: {imageTopic}", guiStyle);
        yOffset += lineHeight;

        GUI.Label(new Rect(10, yOffset, 1200, lineHeight), $"Messages: {messageCount}", guiStyle);
        yOffset += lineHeight;

        if (testTextureApplied && messageCount == 0)
        {
            guiStyle.normal.textColor = Color.red;
            guiStyle.fontSize = 32;

            GUI.Label(new Rect(10, yOffset, 1200, lineHeight), "QUAD SHOULD BE RED - IS IT?", guiStyle);
            return;
        }

        if (messageCount > 0)
        {
            guiStyle.normal.textColor = Color.green;
            guiStyle.fontSize = 24;

            GUI.Label(new Rect(10, yOffset, 1200, lineHeight), $"✓ Receiving images from ROS", guiStyle);
            yOffset += lineHeight;

            if (cameraTexture != null)
            {
                GUI.Label(
                    new Rect(10, yOffset, 1200, lineHeight),
                    $"Texture: {cameraTexture.width}x{cameraTexture.height} ({cameraTexture.format})",
                    guiStyle
                );

                yOffset += lineHeight;
            }
        }
    }
}
