using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RosUnifiedLowLatencyViewer : MonoBehaviour
{
    [Header("ROS Topics")]
    public string depthCompressedTopic = "/depth/image_raw/compressed"; // UPDATE THIS!
    public string colorCompressedTopic = "/color/image_raw/compressed";
    public string cameraInfoTopic = "/depth/camera_info";

    [Header("Depth Mesh Settings")]
    public float depthScale = 0.001f;
    [Range(1, 8)] public int pixelStride = 4;
    public float minDepthMeters = 0.3f;
    public float maxDepthMeters = 2.5f;
    public float maxEdgeDeltaMeters = 0.05f;

    [Header("Color Settings")]
    public bool useColorTexture = true;
    
    [Header("Latency & Drop Policy")]
    [SerializeField] private bool enableLatencyMeasurement = true;
    [SerializeField] private bool dropStaleFrames = true;
    [SerializeField] private float maxAcceptableTotalLatencyMs = 300f;
    [SerializeField] private int calibrationSamples = 10;

    private ROSConnection ros;
    private Mesh mesh;
    private MeshRenderer meshRenderer;
    
    // Textures
    private Texture2D colorTexture;
    private Texture2D depthTexture; // New texture for decoding depth PNG

    // Messages and Locks
    private CompressedImageMsg latestDepthMsg;
    private CompressedImageMsg latestColorMsg;
    private readonly object depthLock = new object();
    private readonly object colorLock = new object();
    private bool newDepthAvailable = false;
    private bool newColorAvailable = false;
    private bool cameraInfoReady = false;

    // Intrinsics
    private float fx, fy, cx, cy;

    // Pre-allocated Mesh Data
    private Vector3[] vertices;
    private Vector2[] uvs;
    private Color32[] colors;
    private int[] triangles;
    private bool[] validVertex;
    private int gridWidth = 0, gridHeight = 0;
    private int lastDepthWidth = -1, lastDepthHeight = -1;

    // Latency Measurement Variables
    private double rosToUnityTimeOffset = 0.0;
    private bool timeOffsetCalculated = false;
    private List<double> calibrationOffsets = new List<double>();
    private int frameCount = 0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        // NOW SUBSCRIBING TO COMPRESSED IMAGE FOR DEPTH
        ros.Subscribe<CompressedImageMsg>(depthCompressedTopic, DepthCallback);
        ros.Subscribe<CompressedImageMsg>(colorCompressedTopic, ColorCallback);
        ros.Subscribe<CameraInfoMsg>(cameraInfoTopic, CameraInfoCallback);

        mesh = new Mesh();
        mesh.name = "ROS Unified Low Latency Mesh";
        mesh.indexFormat = IndexFormat.UInt32;
        mesh.MarkDynamic();
        GetComponent<MeshFilter>().mesh = mesh;

        meshRenderer = GetComponent<MeshRenderer>();
        calibrationOffsets.Clear();
        timeOffsetCalculated = false;
    }

    void DepthCallback(CompressedImageMsg msg)
    {
        lock (depthLock)
        {
            latestDepthMsg = msg;
            newDepthAvailable = true;
        }
    }

    void ColorCallback(CompressedImageMsg msg)
    {
        lock (colorLock)
        {
            latestColorMsg = msg;
            newColorAvailable = true;
        }
    }

    void CameraInfoCallback(CameraInfoMsg msg)
    {
        if (msg.k == null || msg.k.Length < 9) return;
        fx = (float)msg.k[0];
        fy = (float)msg.k[4];
        cx = (float)msg.k[2];
        cy = (float)msg.k[5];
        cameraInfoReady = true;
    }

    void Update()
    {
        if (!cameraInfoReady) return;

        ProcessColorStream();
        ProcessDepthStream();
    }

    void ProcessColorStream()
    {
        if (!useColorTexture || !newColorAvailable) return;

        CompressedImageMsg colorMsg = null;
        lock (colorLock)
        {
            colorMsg = latestColorMsg;
            newColorAvailable = false;
        }

        if (colorMsg == null) return;

        if (enableLatencyMeasurement)
        {
            double rosTime = colorMsg.header.stamp.sec + (colorMsg.header.stamp.nanosec / 1e9);
            if (!CalibrateClockOffset(rosTime)) return; 

            double networkLatencyMs = (GetCurrentTimeSeconds() - rosTime - rosToUnityTimeOffset) * 1000.0;
            
            if (dropStaleFrames && networkLatencyMs > maxAcceptableTotalLatencyMs) return;
        }

        if (colorTexture == null)
        {
            colorTexture = new Texture2D(2, 2, TextureFormat.RGB24, false);
            colorTexture.wrapMode = TextureWrapMode.Clamp;
            colorTexture.filterMode = FilterMode.Bilinear; // Smooth interpolation for color is fine
        }

        if (ImageConversion.LoadImage(colorTexture, colorMsg.data))
        {
            meshRenderer.material.mainTexture = colorTexture;
        }
    }

    void ProcessDepthStream()
    {
        if (!newDepthAvailable) return;

        CompressedImageMsg depthMsg = null;
        lock (depthLock)
        {
            depthMsg = latestDepthMsg;
            newDepthAvailable = false;
        }

        if (depthMsg == null || depthMsg.data.Length == 0) return;

        if (enableLatencyMeasurement)
        {
            double rosTime = depthMsg.header.stamp.sec + (depthMsg.header.stamp.nanosec / 1e9);
            if (!CalibrateClockOffset(rosTime)) return; 

            double networkLatencyMs = (GetCurrentTimeSeconds() - rosTime - rosToUnityTimeOffset) * 1000.0;

            if (dropStaleFrames && networkLatencyMs > maxAcceptableTotalLatencyMs)
            {
                Debug.LogWarning($"[Depth] Dropped stale frame. Latency: {networkLatencyMs:F1}ms");
                return;
            }
        }

        double processStartTime = GetCurrentTimeSeconds();

        // 1. Initialize Depth Texture
        if (depthTexture == null)
        {
            depthTexture = new Texture2D(2, 2, TextureFormat.RGB24, false);
            // CRITICAL: Point filter mode prevents Unity from blurring the raw data bits!
            depthTexture.filterMode = FilterMode.Point; 
            depthTexture.wrapMode = TextureWrapMode.Clamp;
        }

        // 2. Load the PNG
        if (ImageConversion.LoadImage(depthTexture, depthMsg.data))
        {
            // 3. Extract the pixel array
            Color32[] depthPixels = depthTexture.GetPixels32();
            
            // 4. Build the mesh
            BuildDepthMeshFast(depthPixels, depthTexture.width, depthTexture.height);
            frameCount++;

            if (enableLatencyMeasurement && frameCount % 60 == 0)
            {
                double processTimeMs = (GetCurrentTimeSeconds() - processStartTime) * 1000.0;
                Debug.Log($"[Depth Processing] PNG Decomp & Mesh Build Latency: {processTimeMs:F1}ms");
            }
        }
    }

    void BuildDepthMeshFast(Color32[] depthPixels, int width, int height)
    {
        if (width <= 1 || height <= 1) return;

        int newGridWidth = Mathf.CeilToInt(width / (float)pixelStride);
        int newGridHeight = Mathf.CeilToInt(height / (float)pixelStride);
        int vertexCount = newGridWidth * newGridHeight;

        if (width != lastDepthWidth || height != lastDepthHeight || newGridWidth != gridWidth || newGridHeight != gridHeight)
        {
            gridWidth = newGridWidth;
            gridHeight = newGridHeight;
            vertices = new Vector3[vertexCount];
            uvs = new Vector2[vertexCount];
            colors = new Color32[vertexCount];
            validVertex = new bool[vertexCount];
            triangles = new int[(gridWidth - 1) * (gridHeight - 1) * 6];
            
            lastDepthWidth = width;
            lastDepthHeight = height;
        }

        for (int gy = 0; gy < gridHeight; gy++)
        {
            int v = Mathf.Min(gy * pixelStride, height - 1);
            for (int gx = 0; gx < gridWidth; gx++)
            {
                int u = Mathf.Min(gx * pixelStride, width - 1);
                int index = gy * gridWidth + gx;

                // Unity's GetPixels32 reads textures from bottom to top. 
                // We must invert the V coordinate to read the correct pixel.
                int texV = height - 1 - v;
                int pixelIndex = texV * width + u;

                validVertex[index] = false;

                // DECODE THE RAW 16-BIT DEPTH FROM THE PNG
                // Red Channel = Lower 8 bits
                // Green Channel = Upper 8 bits
                Color32 p = depthPixels[pixelIndex];
                ushort depthRaw = (ushort)(p.r | (p.g << 8));

                if (depthRaw == 0) continue;

                float z = depthRaw * depthScale;
                if (z < minDepthMeters || z > maxDepthMeters) continue;

                float x = (u - cx) * z / fx;
                float y = (v - cy) * z / fy;

                vertices[index] = new Vector3(x, -y, z);
                validVertex[index] = true;

                float uvX = (float)u / (width - 1);
                float uvY = 1f - ((float)v / (height - 1));
                uvs[index] = new Vector2(uvX, uvY);

                if (!useColorTexture)
                {
                    float t = Mathf.Clamp01(Mathf.InverseLerp(minDepthMeters, maxDepthMeters, z));
                    t = Mathf.Sqrt(1f - t);
                    byte gray = (byte)Mathf.RoundToInt(t * 220f + 35f);
                    colors[index] = new Color32(gray, gray, gray, 255);
                }
            }
        }

        int triCount = 0;
        for (int gy = 0; gy < gridHeight - 1; gy++)
        {
            for (int gx = 0; gx < gridWidth - 1; gx++)
            {
                int i00 = gy * gridWidth + gx;
                int i10 = gy * gridWidth + (gx + 1);
                int i01 = (gy + 1) * gridWidth + gx;
                int i11 = (gy + 1) * gridWidth + (gx + 1);

                TryAddTriangleFast(i00, i10, i01, ref triCount);
                TryAddTriangleFast(i10, i11, i01, ref triCount);
            }
        }

        mesh.Clear(false);
        mesh.SetVertices(vertices, 0, vertexCount);
        mesh.SetUVs(0, uvs, 0, vertexCount);
        if (!useColorTexture) mesh.SetColors(colors, 0, vertexCount);
        mesh.SetIndices(triangles, 0, triCount, MeshTopology.Triangles, 0, false);
        mesh.RecalculateBounds();
    }

    void TryAddTriangleFast(int a, int b, int c, ref int triCount)
    {
        if (!validVertex[a] || !validVertex[b] || !validVertex[c]) return;

        float za = vertices[a].z;
        float zb = vertices[b].z;
        float zc = vertices[c].z;

        if (Mathf.Abs(za - zb) > maxEdgeDeltaMeters) return;
        if (Mathf.Abs(za - zc) > maxEdgeDeltaMeters) return;
        if (Mathf.Abs(zb - zc) > maxEdgeDeltaMeters) return;

        triangles[triCount++] = a;
        triangles[triCount++] = b;
        triangles[triCount++] = c;
    }

    private bool CalibrateClockOffset(double rosTime)
    {
        if (timeOffsetCalculated) return true;

        double unityTime = GetCurrentTimeSeconds();
        calibrationOffsets.Add(unityTime - rosTime);

        if (calibrationOffsets.Count >= calibrationSamples)
        {
            calibrationOffsets.Sort();
            rosToUnityTimeOffset = calibrationOffsets[calibrationSamples / 2];
            timeOffsetCalculated = true;
            Debug.Log($"[Clock Sync] Calibration complete. Offset: {rosToUnityTimeOffset * 1000:F2} ms");
            return true;
        }
        return false;
    }

    private double GetCurrentTimeSeconds()
    {
        return (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
    }
}
