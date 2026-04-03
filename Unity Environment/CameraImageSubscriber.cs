using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class CameraImageSubscriberDebug : MonoBehaviour
{
    // Image topic
    public string imageTopic = "/color/image_raw/compressed";
    public Renderer displayRenderer;

    private Texture2D cameraTexture;
    private int messageCount = 0;
    private bool testTextureApplied = false;

    private float lastImageTime = -1f;
    private float smoothedDeltaTime = -1f;

    [SerializeField] private int minTargetFps = 10;
    [SerializeField] private int maxTargetFps = 120;

    // Latency measurement variables
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
    private float decompressionLatencyMs = 0f;
    private float renderLatencyMs = 0f;

    // Ros timestamp validation
    private string latencyDebugStatus = "Waiting for data...";
    private bool rosTimestampValid = false;
    private double firstRosTimestamp = -1.0;
    private bool usingMonotonicTime = false;

    // Compression stats
    private long totalCompressedBytes = 0;
    private long totalUncompressedBytes = 0;
    private int compressionStatsCount = 0;

    // Drop policy
    [Header("Drop Policy")]
    [SerializeField] private bool dropStaleFrames = true;
    [SerializeField] private float maxAcceptableTotalLatencyMs = 300f;

    // Cached GUI style
    private GUIStyle guiStyle;
    
    // Latest message handling
    private CompressedImageMsg latestImageMsg = null;
    private bool hasNewImage = false;
    private readonly object imageLock = new object();

    void Start()
    {
        Debug.Log("=== Compressed Camera Image Subscriber Starting ===");
        
        // Reset message count to ensure proper calibration on restart
        messageCount = 0;
        rosTimestampValid = false;
        timeOffsetCalculated = false;
        calibrationOffsets.Clear();
        
        // Reset latency stats
        minLatencyMs = float.MaxValue;
        maxLatencyMs = 0f;
        avgLatencyMs = 0f;
        latencySum = 0.0;
        latencyCount = 0;

        // Find Renderer if not assigned
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

        Debug.Log($"  Renderer found: {displayRenderer.gameObject.name}");

        // Debug texture to confirm setup before images arrive
        // Set image to be all red
        Debug.Log("TEST: Applying RED test texture...");
        Texture2D redTexture = new Texture2D(256, 256, TextureFormat.RGB24, false);
        Color[] redPixels = new Color[256 * 256];

        for (int i = 0; i < redPixels.Length; i++)
            redPixels[i] = Color.red;

        redTexture.SetPixels(redPixels);
        redTexture.Apply(false, false);

        Shader unlitShader = Shader.Find("Unlit/Texture");

        // Warning if shader not found, but still apply texture to see if it works
        if (unlitShader == null)
        {
            Debug.LogWarning("  Unlit/Texture shader not found, using existing material");
            displayRenderer.material.mainTexture = redTexture;
        }
        else
        {
            displayRenderer.material = new Material(unlitShader);
            displayRenderer.material.mainTexture = redTexture;
        }

        testTextureApplied = true;

        // Vertical flip
        displayRenderer.material.mainTextureScale = new Vector2(1f, 1f);
        displayRenderer.material.mainTextureOffset = new Vector2(0f, 0f);

        guiStyle = new GUIStyle
        {
            fontSize = 28,
            fontStyle = FontStyle.Bold,
            normal = { textColor = Color.yellow }
        };

        // Initialize ROS connection and subscribe to compressed image topic
        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        Debug.Log($"  ROS Connection: {ros.RosIPAddress}:{ros.RosPort}");

        // Subscribe to CompressedImageMsg
        ros.Subscribe<CompressedImageMsg>(imageTopic, UpdateCameraImage);
        Debug.Log($"  Subscribed to COMPRESSED topic: {imageTopic}");
        Debug.Log("  === Waiting for ROS images (Quad should be RED while waiting) ===");

        // Latency measurement setup
        if (enableLatencyMeasurement)
        {
            Debug.Log("=== LATENCY MEASUREMENT ENABLED (Compressed) ===");
            Debug.Log($"  Need {calibrationSamples} frames for clock calibration");

            latencyDebugStatus = "Waiting for first frame...";
        }
    }

    void UpdateCameraImage(CompressedImageMsg compressedMsg)
    {
        // Store only the latest image - discard any previous unprocessed ones
        lock (imageLock)
        {
            latestImageMsg = compressedMsg;
            hasNewImage = true;
        }
    }
    
    void Update()
    {
        // Process the latest image if available
        CompressedImageMsg imageToProcess = null;
        
        // Access the latest image
        lock (imageLock)
        {
            if (hasNewImage && latestImageMsg != null)
            {
                imageToProcess = latestImageMsg;
                hasNewImage = false;
            }
        }
        
        if (imageToProcess != null)
        {
            ProcessCompressedImage(imageToProcess);
        }
    }
    
    // Process compressed images
    void ProcessCompressedImage(CompressedImageMsg compressedMsg)
    {
        messageCount++;
        Debug.Log($"FRAME RECEIVED: #{messageCount}, Size={compressedMsg.data.Length} bytes");

        frameReceiveTime = GetCurrentTimeSeconds();

        // Latency measurement
        if (enableLatencyMeasurement)
        {
            // Extract ROS timestamp
            Debug.Log($"LATENCY MEASUREMENT CHECK: enableLatencyMeasurement={enableLatencyMeasurement}");
            double rosTimestampSec = compressedMsg.header.stamp.sec;
            double rosTimestampNsec = compressedMsg.header.stamp.nanosec;
            double rosTimestamp = rosTimestampSec + (rosTimestampNsec / 1e9);

            Debug.Log($"TIMESTAMP DEBUG: Frame {messageCount}, ROS={rosTimestamp:F6}, Sec={rosTimestampSec}, Nsec={rosTimestampNsec}");

            frameCaptureTime = rosTimestamp;

                // Validate timestamp for any frame if not already validated
                if (!rosTimestampValid)
                {
                    // Check for valid timestamp
                    if (rosTimestamp <= 0)
                    {
                        Debug.LogError($"  ROS TIMESTAMP INVALID (<=0) at frame {messageCount}. Latency stats will be disabled.");
                        rosTimestampValid = false;
                        latencyDebugStatus = "ERROR: ROS timestamp invalid";
                    }
                    // Check for timestamps that look like monotonic time
                    else
                    {
                        firstRosTimestamp = rosTimestamp;
                        rosTimestampValid = true;

                        usingMonotonicTime = rosTimestamp < 1000000000;

                        calibrationOffsets.Clear();
                        timeOffsetCalculated = false;

                        latencyDebugStatus = $"Calibrating (0/{calibrationSamples})...";
                        
                        // Log the timestamp validation and calibration start
                        Debug.Log($"  ROS timestamp valid at frame {messageCount}. Starting calibration...");
                        Debug.Log($"  rosTimestampValid={rosTimestampValid}, timeOffsetCalculated={timeOffsetCalculated}, calibrationOffsets.Count={calibrationOffsets.Count}");
                    }
                }

                // Debug message on first frame with timestamp
                if (messageCount == 1)
                {
                    Debug.Log($"=== FIRST COMPRESSED FRAME ===");
                    Debug.Log($"  Format: {compressedMsg.format}");
                    Debug.Log($"  Compressed size: {compressedMsg.data.Length} bytes ({compressedMsg.data.Length / 1024f:F1} KB)");
                    Debug.Log($"  ROS stamp: {rosTimestampSec}.{rosTimestampNsec:D9} (combined {rosTimestamp:F6})");
                    Debug.Log($"  Unity receive time: {frameReceiveTime:F6}");
                }

                // Perform time offset calibration using the first few frames with valid timestamps
                if (rosTimestampValid && !timeOffsetCalculated && calibrationOffsets.Count < calibrationSamples)
                {
                    double offset = frameReceiveTime - frameCaptureTime;

                    calibrationOffsets.Add(offset);

                    latencyDebugStatus = $"Calibrating ({calibrationOffsets.Count}/{calibrationSamples})...";
                    Debug.Log($"CALIBRATING: Frame {messageCount}, Samples {calibrationOffsets.Count}/{calibrationSamples}, Offset={offset*1000:F2}ms");

                    if (calibrationOffsets.Count == calibrationSamples)
                    {
                        calibrationOffsets.Sort();

                        rosToUnityTimeOffset = calibrationOffsets[calibrationSamples / 2];

                        timeOffsetCalculated = true;
                        latencyDebugStatus = "Measuring latency";

                        Debug.Log($"╔════════════════════════════════════════╗");
                        Debug.Log($"║  TIME CALIBRATION COMPLETE          ║");
                        Debug.Log($"╚════════════════════════════════════════╝");
                        Debug.Log($"  Offset: {rosToUnityTimeOffset * 1000:F2} ms");
                    }
                }

            // Calculate network latency immediately upon receiving the frame
            if (timeOffsetCalculated && rosTimestampValid)
            {
                double adjustedDiff = frameReceiveTime - frameCaptureTime - rosToUnityTimeOffset;
                networkLatencyMs = (float)(adjustedDiff * 1000.0);
                
                // Drop stale frames if enabled and latency is too high
                if (dropStaleFrames && networkLatencyMs > maxAcceptableTotalLatencyMs)
                {
                    Debug.LogWarning($"Dropping stale frame: latency {networkLatencyMs:F1}ms > {maxAcceptableTotalLatencyMs}ms");
                    return;
                }
            }
        }

        // Update frame timing
        float now = Time.realtimeSinceStartup;

        // Dynamic frame rate adjustment based on image arrival rate
        if (lastImageTime > 0f)
        {
            float delta = now - lastImageTime;

            // Smooth the delta time
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

            // Create texture if needed
            if (cameraTexture == null)
            {
                cameraTexture = new Texture2D(2, 2, TextureFormat.RGB24, false);
                cameraTexture.wrapMode = TextureWrapMode.Clamp;
                cameraTexture.filterMode = FilterMode.Bilinear;
            }

            // Decompress JPEG using Unity's ImageConversion
            bool success = ImageConversion.LoadImage(cameraTexture, compressedMsg.data);

            if (!success)
            {
                Debug.LogError($"Failed to decompress image at frame {messageCount}");
                return;
            }

            // Apply to renderer
            displayRenderer.material.mainTexture = cameraTexture;

            frameProcessTime = GetCurrentTimeSeconds();
            decompressionLatencyMs = (float)((frameProcessTime - processStartTime) * 1000.0);

            // Compression statistics
            compressionStatsCount++;
            totalCompressedBytes += compressedMsg.data.Length;
            totalUncompressedBytes += cameraTexture.width * cameraTexture.height * 3; // RGB24

            // Log details on the first frame
            if (messageCount == 1)
            {
                Debug.Log($"First image decompressed successfully:");
                Debug.Log($"   Compressed: {compressedMsg.data.Length} bytes ({compressedMsg.data.Length / 1024f:F1} KB)");
                Debug.Log($"   Uncompressed: {cameraTexture.width}x{cameraTexture.height} = {cameraTexture.width * cameraTexture.height * 3 / 1024f:F1} KB");
                Debug.Log($"   Format: {cameraTexture.format}");
                Debug.Log($"   Decompression time: {decompressionLatencyMs:F2} ms");
                float ratio = (cameraTexture.width * cameraTexture.height * 3) / (float)compressedMsg.data.Length;
                Debug.Log($"   Compression ratio: {ratio:F1}x");
            }

            // Print debug info for latency measurement
            Debug.Log($"EnableLatencyMeasurement={enableLatencyMeasurement}, timeOffsetCalculated={timeOffsetCalculated}, rosTimestampValid={rosTimestampValid}");

            // Calculate latency if we have a valid timestamp and time offset
            if (enableLatencyMeasurement && timeOffsetCalculated && rosTimestampValid)
            {
                // Calculate render latency just before displaying the frame
                frameDisplayTime = GetCurrentTimeSeconds();
                renderLatencyMs = (float)((frameDisplayTime - frameProcessTime) * 1000.0);
                
                // Total latency from capture to display, adjusted for clock offset
                double totalLatency = frameDisplayTime - frameCaptureTime - rosToUnityTimeOffset;
                currentLatencyMs = (float)(totalLatency * 1000.0);

                Debug.Log($"ROS TimestampValid: {rosTimestampValid}");

                // Update latency stats
                if (currentLatencyMs > 0 && currentLatencyMs < 60000)
                {
                    // Min and max latency
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

                    // Log latency every frame for debugging
                    Debug.Log($"LATENCY: Total={currentLatencyMs:F1}ms | Net={networkLatencyMs:F1}ms | Decomp={decompressionLatencyMs:F1}ms | Render={renderLatencyMs:F1}ms | Frame={messageCount}");

                    // Print latency stats every 60 frames
                    if (messageCount % 60 == 0)
                    {
                        Debug.Log($"╔════════════════════════════════════════╗");
                        Debug.Log($"║  LATENCY BREAKDOWN (Compressed)        ║");
                        Debug.Log($"╚════════════════════════════════════════╝");
                        Debug.Log($"  Total Latency:         {currentLatencyMs:F2} ms");
                        Debug.Log($"  Network Latency:       {networkLatencyMs:F2} ms");
                        Debug.Log($"  Decompression Latency: {decompressionLatencyMs:F2} ms");
                        Debug.Log($"  Render Latency:        {renderLatencyMs:F2} ms");
                        Debug.Log($"  ────────────────────────────────────────");
                        Debug.Log($"  Min: {minLatencyMs:F2} ms");
                        Debug.Log($"  Avg: {avgLatencyMs:F2} ms");
                        Debug.Log($"  Max: {maxLatencyMs:F2} ms");
                    }
                }
            }

            // Print compression stats every 60 frames
            if (messageCount % 60 == 0)
            {
                float avgCompressedKB = (totalCompressedBytes / (float)compressionStatsCount) / 1024f;
                float avgUncompressedKB = (totalUncompressedBytes / (float)compressionStatsCount) / 1024f;
                float compressionRatio = totalUncompressedBytes / (float)totalCompressedBytes;
                float bandwidthSaved = (1 - 1 / compressionRatio) * 100f;

                Debug.Log($"╔════════════════════════════════════════╗");
                Debug.Log($"║  COMPRESSION STATS ({compressionStatsCount} frames)        ║");
                Debug.Log($"╚════════════════════════════════════════╝");
                Debug.Log($"  Avg compressed: {avgCompressedKB:F1} KB/frame");
                Debug.Log($"  Avg uncompressed: {avgUncompressedKB:F1} KB/frame");
                Debug.Log($"  Compression ratio: {compressionRatio:F1}x");
                Debug.Log($"  Bandwidth saved: {bandwidthSaved:F1}%");
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"EXCEPTION: {e.Message}");
            Debug.LogError($"  Stack trace: {e.StackTrace}");
        }
    }

    private double GetCurrentTimeSeconds()
    {
        // Use Unix epoch time to match ROS timestamps
        return (System.DateTime.UtcNow - new System.DateTime(1970, 1, 1)).TotalSeconds;
    }

    void OnGUI()
    {
        if (guiStyle == null) return;

        int yOffset = 10;
        int lineHeight = 40;

        guiStyle.fontSize = 28;
        guiStyle.normal.textColor = Color.yellow;

        // Display topic and message count
        GUI.Label(new Rect(10, yOffset, 1400, lineHeight), $"Topic: {imageTopic} (COMPRESSED)", guiStyle);
        yOffset += lineHeight;

        // Display message count
        GUI.Label(new Rect(10, yOffset, 1200, lineHeight), $"Frames: {messageCount}", guiStyle);
        yOffset += lineHeight;

        // If no messages received yet, show waiting message
        if (testTextureApplied && messageCount == 0)
        {
            guiStyle.normal.textColor = Color.red;
            guiStyle.fontSize = 32;

            yOffset += lineHeight;
            
            guiStyle.fontSize = 24;
            guiStyle.normal.textColor = Color.yellow;
            GUI.Label(new Rect(10, yOffset, 1400, lineHeight), "Waiting for compressed images...", guiStyle);
            return;
        }

        // If messages are being received, show stats
        if (messageCount > 0)
        {
            guiStyle.normal.textColor = Color.green;
            guiStyle.fontSize = 24;

            GUI.Label(new Rect(10, yOffset, 1400, lineHeight), $"Receiving COMPRESSED images", guiStyle);
            yOffset += lineHeight;

            if (cameraTexture != null)
            {
                GUI.Label(
                    new Rect(10, yOffset, 1200, lineHeight),
                    $"Resolution: {cameraTexture.width}x{cameraTexture.height}",
                    guiStyle
                );

                yOffset += lineHeight;
            }

            // Compression stats
            if (compressionStatsCount > 0)
            {
                float avgCompressedKB = (totalCompressedBytes / (float)compressionStatsCount) / 1024f;
                float compressionRatio = totalUncompressedBytes / (float)totalCompressedBytes;

                // Color code based on compression ratio
                guiStyle.normal.textColor = Color.cyan;
                guiStyle.fontSize = 22;
                GUI.Label(
                    new Rect(10, yOffset, 1200, lineHeight),
                    $"Avg: {avgCompressedKB:F1} KB/frame | Ratio: {compressionRatio:F1}x",
                    guiStyle
                );
                yOffset += lineHeight;
            }

            // Latency display
            if (enableLatencyMeasurement)
            {
                yOffset += 10;

                if (timeOffsetCalculated && rosTimestampValid)
                {
                    // Color code based on total latency
                    if (currentLatencyMs < 50f)
                        guiStyle.normal.textColor = Color.green;
                    else if (currentLatencyMs < 100f)
                        guiStyle.normal.textColor = Color.yellow;
                    else
                        guiStyle.normal.textColor = Color.red;

                    guiStyle.fontSize = 36;
                    GUI.Label(
                        new Rect(10, yOffset, 1200, lineHeight + 10),
                        $"TOTAL: {currentLatencyMs:F1} ms",
                        guiStyle
                    );
                    yOffset += lineHeight + 10;

                    guiStyle.fontSize = 24;
                    guiStyle.normal.textColor = Color.cyan;
                    GUI.Label(
                        new Rect(10, yOffset, 1200, lineHeight),
                        $"Min: {minLatencyMs:F1} | Avg: {avgLatencyMs:F1} | Max: {maxLatencyMs:F1}",
                        guiStyle
                    );
                    yOffset += lineHeight;

                    // Breakdown
                    guiStyle.fontSize = 22;
                    guiStyle.normal.textColor = Color.white;
                    GUI.Label(
                        new Rect(10, yOffset, 1400, lineHeight),
                        $"Net: {networkLatencyMs:F1}ms | Decomp: {decompressionLatencyMs:F1}ms | Render: {renderLatencyMs:F1}ms",
                        guiStyle
                    );
                }

                // If timestamp is invalid, show warning
                else
                {
                    guiStyle.normal.textColor = Color.yellow;
                    guiStyle.fontSize = 24;
                    GUI.Label(
                        new Rect(10, yOffset, 1400, lineHeight),
                        $"{latencyDebugStatus}",
                        guiStyle
                    );
                }
            }
        }
    }
}
