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
    public string depthCompressedTopic = "/depth/image_raw/compressed"; // topics required
    public string colourCompressedTopic = "/colour/image_raw/compressed";
    public string cameraInfoTopic = "/depth/camera_info";

    [Header("Depth Mesh Settings")]
    // alter these in unity to get better/worse mesh quality and performance.
    public float depthScale = 0.001f; // converting from mm to meters
    [Range(1, 8)] public int pixelStride = 4; // how many pixels to skip when building the mesh
    public float minDepthMeters = 0.3f;
    public float maxDepthMeters = 2.5f;
    public float maxEdgeDeltaMeters = 0.05f; // depth discontinuity threshold for triangle creation

    [Header("colour Settings")]
    // set to false to turn off colour.
    public bool usecolourTexture = true;
    
    [Header("Latency & Drop Policy")]
    [SerializeField] private bool enableLatencyMeasurement = true;
    [SerializeField] private bool dropStaleFrames = true;
    [SerializeField] private float maxAcceptableTotalLatencyMs = 300f; // threshold for dropping frames
    [SerializeField] private int calibrationSamples = 10; // for clock sync calibration

    private ROSConnection ros;
    private Mesh mesh;
    private MeshRenderer meshRenderer;
    
    // Textures
    private Texture2D colourTexture;
    private Texture2D depthTexture; // New texture for decoding depth PNG

    // Messages and Locks
    private CompressedImageMsg latestDepthMsg;
    private CompressedImageMsg latestcolourMsg;
    private readonly object depthLock = new object();
    private readonly object colourLock = new object();
    private bool newDepthAvailable = false;
    private bool newcolourAvailable = false;
    private bool cameraInfoReady = false;

    // Intrinsics
    private float fx, fy, cx, cy; // fx and fy are focal lengths, cx and cy are principal points (center of image)

    // Pre-allocated Mesh Data
    private Vector3[] vertices;
    private Vector2[] uvs;
    private colour32[] colours;
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
        ros.Subscribe<CompressedImageMsg>(colourCompressedTopic, colourCallback);
        ros.Subscribe<CameraInfoMsg>(cameraInfoTopic, CameraInfoCallback);
        // initilze the mesh
        mesh = new Mesh();
        mesh.name = "ROS Unified Low Latency Mesh";
        mesh.indexFormat = IndexFormat.UInt32;
        mesh.MarkDynamic();
        GetComponent<MeshFilter>().mesh = mesh;

        meshRenderer = GetComponent<MeshRenderer>();
        calibrationOffsets.Clear();
        timeOffsetCalculated = false;
    }

    // callbacks execute everytime new data comes
    // is depth ready?
    void DepthCallback(CompressedImageMsg msg)
    {
        lock (depthLock)
        {
            latestDepthMsg = msg;
            newDepthAvailable = true;
        }
    }
    // is colour ready?
    void colourCallback(CompressedImageMsg msg)
    {
        lock (colourLock)
        {
            latestcolourMsg = msg;
            newcolourAvailable = true;
        }
    }

    // pull data out of intrinsic matrix 
    void CameraInfoCallback(CameraInfoMsg msg)
    {
        if (msg.k == null || msg.k.Length < 9) return;
        fx = (float)msg.k[0];
        fy = (float)msg.k[4];
        cx = (float)msg.k[2];
        cy = (float)msg.k[5];
        cameraInfoReady = true;
    }

    // Update is called once per frame (its a unity thing)
    void Update()
    {
        if (!cameraInfoReady) return;

        ProcesscolourStream();
        ProcessDepthStream();
    }

    void ProcesscolourStream()
    {   // checks if colour is enabled and if new colour data is here
        if (!usecolourTexture || !newcolourAvailable) return;

        CompressedImageMsg colourMsg = null;
        lock (colourLock)
        {
            colourMsg = latestcolourMsg;
            newcolourAvailable = false;
        }

        if (colourMsg == null) return;

        if (enableLatencyMeasurement)
        {   // converts ros time to unity time
            double rosTime = colourMsg.header.stamp.sec + (colourMsg.header.stamp.nanosec / 1e9);
            if (!CalibrateClockOffset(rosTime)) return; 

            double networkLatencyMs = (GetCurrentTimeSeconds() - rosTime - rosToUnityTimeOffset) * 1000.0;
            // exit if frame is deemed too stale
            if (dropStaleFrames && networkLatencyMs > maxAcceptableTotalLatencyMs) return;
        }
        // first time? create the texture
        if (colourTexture == null)
        {
            colourTexture = new Texture2D(2, 2, TextureFormat.RGB24, false);
            colourTexture.wrapMode = TextureWrapMode.Clamp;
            colourTexture.filterMode = FilterMode.Bilinear; // smooth pixels together
        }

        if (ImageConversion.LoadImage(colourTexture, colourMsg.data)) // write the compressed data into the texture
        {
            meshRenderer.material.mainTexture = colourTexture; // next render, the mesh will use this updated texture
        }
    }

    void ProcessDepthStream()
    {
        if (!newDepthAvailable) return; // no info? return

        CompressedImageMsg depthMsg = null;
        lock (depthLock)
        {
            depthMsg = latestDepthMsg;
            newDepthAvailable = false;
        }
        // ensure depth data was actually received
        if (depthMsg == null || depthMsg.data.Length == 0) return;

        if (enableLatencyMeasurement)
        {   // same as colour, convert ros time to unity time and check if frame is too stale to process
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

        // make depth texture if it doesn't exist yet
        if (depthTexture == null)
        {
            depthTexture = new Texture2D(2, 2, TextureFormat.RGB24, false);
            // pretty similar to colour, but keeps the pixels alone (no bilinear filtering)
            depthTexture.filterMode = FilterMode.Point; 
            depthTexture.wrapMode = TextureWrapMode.Clamp;
        }

        // load image data into texture
        if (ImageConversion.LoadImage(depthTexture, depthMsg.data))
        {
            // get the pixel array into a 1D array for CPU math. 
            colour32[] depthPixels = depthTexture.GetPixels32();
            
            // build the 3D mesh (see below)
            BuildDepthMeshFast(depthPixels, depthTexture.width, depthTexture.height);
            frameCount++;

            // log latency every 60 frames
            if (enableLatencyMeasurement && frameCount % 60 == 0)
            {
                double processTimeMs = (GetCurrentTimeSeconds() - processStartTime) * 1000.0;
                Debug.Log($"[Depth Processing] PNG Decomp & Mesh Build Latency: {processTimeMs:F1}ms");
            }
        }
    }

    void BuildDepthMeshFast(colour32[] depthPixels, int width, int height)
    {   // make sure a mesh can be made
        if (width <= 1 || height <= 1) return;
        // to improve performance the user can alter pixel stride, which will skip pixels
        int newGridWidth = Mathf.CeilToInt(width / (float)pixelStride);
        int newGridHeight = Mathf.CeilToInt(height / (float)pixelStride);
        int vertexCount = newGridWidth * newGridHeight;
        // makes sure we don't recreate arrays every frame. only recreate if resolution changes
        if (width != lastDepthWidth || height != lastDepthHeight || newGridWidth != gridWidth || newGridHeight != gridHeight)
        {   // if needed, resize arrays to fit new resolution
            gridWidth = newGridWidth;
            gridHeight = newGridHeight;
            vertices = new Vector3[vertexCount];
            uvs = new Vector2[vertexCount];
            colours = new colour32[vertexCount];
            validVertex = new bool[vertexCount];
            triangles = new int[(gridWidth - 1) * (gridHeight - 1) * 6];
            
            lastDepthWidth = width;
            lastDepthHeight = height;
        }
        // loop thru the depth pixels and decode the depth (maps x,y to u,v)
        for (int gy = 0; gy < gridHeight; gy++)
        {
            int v = Mathf.Min(gy * pixelStride, height - 1); // make sure we don't go out of bounds on the last pixel
            for (int gx = 0; gx < gridWidth; gx++)
            {
                int u = Mathf.Min(gx * pixelStride, width - 1); // make sure we don't go out of bounds on the last pixel
                int index = gy * gridWidth + gx;

                // unity reads pixels bottom to top :( so we need to invert v
                // the unity coordinate system is evil.
                int texV = height - 1 - v;
                int pixelIndex = texV * width + u;

                validVertex[index] = false;

                // take two 8 bit channels and combine them to get depth
                // Red Channel = Lower 8 bits
                // Green Channel = Upper 8 bits
                colour32 p = depthPixels[pixelIndex];
                ushort depthRaw = (ushort)(p.r | (p.g << 8));
                // if depth is zero, we have no data, so skip this vertex
                if (depthRaw == 0) continue;

                float z = depthRaw * depthScale; // convert to meters
                if (z < minDepthMeters || z > maxDepthMeters) continue; // range check, great for a predefined workspace

                float x = (u - cx) * z / fx; // project pixel into 3D space using camera info
                float y = (v - cy) * z / fy;

                vertices[index] = new Vector3(x, -y, z); // invert y again to make unity happy
                validVertex[index] = true;
                // store the UVs for texturing and later alterations
                float uvX = (float)u / (width - 1);
                float uvY = 1f - ((float)v / (height - 1)); // unity moment
                uvs[index] = new Vector2(uvX, uvY);

                if (!usecolourTexture)
                {
                    float t = Mathf.Clamp01(Mathf.InverseLerp(minDepthMeters, maxDepthMeters, z)); // close is white, far is black
                    t = Mathf.Sqrt(1f - t); // non-linear transform to keep things brighter longer
                    byte gray = (byte)Mathf.RoundToInt(t * 220f + 35f); // makes furthest item dark grey not pitch black
                    colours[index] = new colour32(gray, gray, gray, 255);
                }
            }
        }
        // now we have a big list of vertices and uvs, but we need to connect them with squares to make a mesh.
        int triCount = 0;
        for (int gy = 0; gy < gridHeight - 1; gy++)
        {
            for (int gx = 0; gx < gridWidth - 1; gx++)
            {   // look at the four corners of the square we want to make, and get their vertex indices
                int i00 = gy * gridWidth + gx;  // top left
                int i10 = gy * gridWidth + (gx + 1); // top right
                int i01 = (gy + 1) * gridWidth + gx; //  bottom left
                int i11 = (gy + 1) * gridWidth + (gx + 1); // bottom right

                // make two triangles for each square
                TryAddTriangleFast(i00, i10, i01, ref triCount);
                TryAddTriangleFast(i10, i11, i01, ref triCount);
            }
        }

        mesh.Clear(false); // wipe previous mesh
        mesh.SetVertices(vertices, 0, vertexCount); // set vertices
        mesh.SetUVs(0, uvs, 0, vertexCount); // set uvs for texturing
        if (!usecolourTexture) mesh.Setcolours(colours, 0, vertexCount);
        mesh.SetIndices(triangles, 0, triCount, MeshTopology.Triangles, 0, false); // set triangles
        mesh.RecalculateBounds();
    }

    void TryAddTriangleFast(int a, int b, int c, ref int triCount)
    {   // checks each corner of the triangle to make sure its a valid vertex
        if (!validVertex[a] || !validVertex[b] || !validVertex[c]) return;

        float za = vertices[a].z;
        float zb = vertices[b].z;
        float zc = vertices[c].z;
        // if the depth difference between any two corners is too large don't make a triangle
        if (Mathf.Abs(za - zb) > maxEdgeDeltaMeters) return; 
        if (Mathf.Abs(za - zc) > maxEdgeDeltaMeters) return;
        if (Mathf.Abs(zb - zc) > maxEdgeDeltaMeters) return;

        triangles[triCount++] = a;
        triangles[triCount++] = b;
        triangles[triCount++] = c;
    }

    private bool CalibrateClockOffset(double rosTime) // map ros time to unity time
    {
        if (timeOffsetCalculated) return true; // only calibrate once

        double unityTime = GetCurrentTimeSeconds();
        calibrationOffsets.Add(unityTime - rosTime);
        // check whether there are enough samples.
        if (calibrationOffsets.Count >= calibrationSamples)
        {
            calibrationOffsets.Sort(); // sort the offsets and take the median
            rosToUnityTimeOffset = calibrationOffsets[calibrationSamples / 2]; // grab the middle value (the median)
            timeOffsetCalculated = true;
            Debug.Log($"[Clock Sync] Calibration complete. Offset: {rosToUnityTimeOffset * 1000:F2} ms");
            return true;
        }
        return false;
    }

    private double GetCurrentTimeSeconds()
    {
        return (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds; // use UTC time cause idk what the G1 thinks of local time
    }
}
