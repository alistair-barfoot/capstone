using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RosDepthMeshViewer : MonoBehaviour
{
    [Header("ROS Topics")]
    public string depthTopic = "/depth/image_rect_raw";
    public string cameraInfoTopic = "/depth/camera_info";

    [Header("Depth Settings")]
    [Tooltip("D435 depth is usually 16UC1 in millimeters, so 0.001 converts to meters.")]
    public float depthScale = 0.001f;

    [Tooltip("Skip pixels for speed. 1 = all pixels, 2 = every other pixel, etc.")]
    [Range(1, 8)]
    public int pixelStride = 2;

    [Tooltip("Ignore depth values closer than this.")]
    public float minDepthMeters = 0.3f;

    [Tooltip("Ignore depth values farther than this.")]
    public float maxDepthMeters = 2.5f;

    [Header("Mesh Connectivity")]
    [Tooltip("Do not connect triangles across depth jumps larger than this.")]
    public float maxEdgeDeltaMeters = 0.05f;

    [Header("Performance")]
    [Tooltip("Limit how often the mesh is rebuilt. 0 = every new depth frame.")]
    public float updateInterval = 0.1f;

    [Header("Smoothing")]
    [Range(0.01f, 1f)]
    [Tooltip("Lower = more smoothing, higher = more responsive.")]
    public float positionLerp = 0.25f;

    [Header("Display")]
    public bool rebuildMesh = true;

    private ROSConnection ros;

    private ImageMsg latestDepthMsg;
    private readonly object depthLock = new object();

    private bool newDepthAvailable = false;
    private bool cameraInfoReady = false;

    private float fx, fy, cx, cy;

    private Mesh mesh;

    private Vector3[] vertices;
    private Color32[] colors;
    private int[] triangles;

    private Vector3[] smoothedVertices;
    private bool[] validVertex;

    private int gridWidth = 0;
    private int gridHeight = 0;
    private int lastDepthWidth = -1;
    private int lastDepthHeight = -1;

    private float lastBuildTime = -999f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<ImageMsg>(depthTopic, DepthCallback);
        ros.Subscribe<CameraInfoMsg>(cameraInfoTopic, CameraInfoCallback);

        mesh = new Mesh();
        mesh.name = "ROS Depth Mesh";
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        GetComponent<MeshFilter>().mesh = mesh;
    }

    void DepthCallback(ImageMsg msg)
    {
        lock (depthLock)
        {
            latestDepthMsg = msg;
            newDepthAvailable = true;
        }
    }

    void CameraInfoCallback(CameraInfoMsg msg)
    {
        if (msg.k == null || msg.k.Length < 9)
            return;

        // K = [fx  0 cx
        //      0 fy cy
        //      0  0  1]
        fx = (float)msg.k[0];
        fy = (float)msg.k[4];
        cx = (float)msg.k[2];
        cy = (float)msg.k[5];

        cameraInfoReady = true;
    }

    void Update()
    {
        if (!rebuildMesh || !cameraInfoReady)
            return;

        if (updateInterval > 0f && Time.time - lastBuildTime < updateInterval)
            return;

        ImageMsg depthMsg = null;

        lock (depthLock)
        {
            if (newDepthAvailable)
            {
                depthMsg = latestDepthMsg;
                newDepthAvailable = false;
            }
        }

        if (depthMsg == null)
            return;

        BuildDepthMesh(depthMsg);
        lastBuildTime = Time.time;
    }

    void BuildDepthMesh(ImageMsg depthMsg)
    {
        if (depthMsg.data == null || depthMsg.data.Length == 0)
            return;

        int width = (int)depthMsg.width;
        int height = (int)depthMsg.height;

        if (width <= 1 || height <= 1)
            return;

        if (depthMsg.encoding != "16UC1" && depthMsg.encoding != "mono16")
        {
            Debug.LogWarning($"Unexpected depth encoding: {depthMsg.encoding}. Expected 16UC1 or mono16.");
        }

        int newGridWidth = Mathf.CeilToInt(width / (float)pixelStride);
        int newGridHeight = Mathf.CeilToInt(height / (float)pixelStride);
        int vertexCount = newGridWidth * newGridHeight;

        bool dimensionsChanged =
            width != lastDepthWidth ||
            height != lastDepthHeight ||
            newGridWidth != gridWidth ||
            newGridHeight != gridHeight;

        if (dimensionsChanged || vertices == null || vertices.Length != vertexCount)
        {
            gridWidth = newGridWidth;
            gridHeight = newGridHeight;

            vertices = new Vector3[vertexCount];
            colors = new Color32[vertexCount];
            validVertex = new bool[vertexCount];
            smoothedVertices = new Vector3[vertexCount];

            lastDepthWidth = width;
            lastDepthHeight = height;
        }

        byte[] depthData = depthMsg.data;

        for (int gy = 0; gy < gridHeight; gy++)
        {
            int v = gy * pixelStride;
            if (v >= height)
                v = height - 1;

            for (int gx = 0; gx < gridWidth; gx++)
            {
                int u = gx * pixelStride;
                if (u >= width)
                    u = width - 1;

                int index = gy * gridWidth + gx;
                int byteIndex = (v * width + u) * 2;

                validVertex[index] = false;

                if (byteIndex + 1 >= depthData.Length)
                    continue;

                ushort depthRaw = (ushort)(depthData[byteIndex] | (depthData[byteIndex + 1] << 8));
                if (depthRaw == 0)
                    continue;

                float z = depthRaw * depthScale;
                if (z < minDepthMeters || z > maxDepthMeters)
                    continue;

                float x = (u - cx) * z / fx;
                float y = (v - cy) * z / fy;

                Vector3 newVertex = new Vector3(x, -y, z);

                if (smoothedVertices[index] == Vector3.zero)
                    smoothedVertices[index] = newVertex;
                else
                    smoothedVertices[index] = Vector3.Lerp(smoothedVertices[index], newVertex, positionLerp);

                vertices[index] = smoothedVertices[index];
                validVertex[index] = true;

                float t = Mathf.InverseLerp(minDepthMeters, maxDepthMeters, z);
                t = Mathf.Clamp01(t);
                t = Mathf.Sqrt(1f - t);
                byte gray = (byte)Mathf.RoundToInt(t * 220f + 35f);
                colors[index] = new Color32(gray, gray, gray, 255);
            }
        }

        int maxTriangleIndices = (gridWidth - 1) * (gridHeight - 1) * 6;
        if (triangles == null || triangles.Length < maxTriangleIndices)
        {
            triangles = new int[maxTriangleIndices];
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

                TryAddTriangle(i00, i01, i10, ref triCount);
                TryAddTriangle(i10, i01, i11, ref triCount);
            }
        }

        if (triCount == 0)
            return;

        int[] finalTriangles = new int[triCount];
        Array.Copy(triangles, finalTriangles, triCount);

        mesh.Clear(false);
        mesh.vertices = vertices;
        mesh.colors32 = colors;
        mesh.triangles = finalTriangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    void TryAddTriangle(int a, int b, int c, ref int triCount)
    {
        if (!validVertex[a] || !validVertex[b] || !validVertex[c])
            return;

        float za = vertices[a].z;
        float zb = vertices[b].z;
        float zc = vertices[c].z;

        if (Mathf.Abs(za - zb) > maxEdgeDeltaMeters)
            return;
        if (Mathf.Abs(za - zc) > maxEdgeDeltaMeters)
            return;
        if (Mathf.Abs(zb - zc) > maxEdgeDeltaMeters)
            return;

        triangles[triCount++] = a;
        triangles[triCount++] = b;
        triangles[triCount++] = c;
    }
}
