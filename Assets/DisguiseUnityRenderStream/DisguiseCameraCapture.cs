#if UNITY_STANDALONE_WIN 
#define PLUGIN_AVAILABLE
#endif

using System;
using System.Net;
using System.Net.Sockets;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

using Microsoft.Win32;

using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Serialization;
using System.Threading;
using System.Runtime.Remoting;
using Disguise.RenderStream;

[RequireComponent(typeof(Camera))]
public class DisguiseCameraCapture : MonoBehaviour
{
    public enum CaptureFormat
    {
        RGB,
        RGB_ALPHA,
        YUV,
        YUV_ALPHA
    }

    public enum CameraMode
    {
        Controlled,
        Fixed
    }

    // inspector fields
    [FormerlySerializedAs("Width")]
    [SerializeField]
    int m_width = 1920;
    [FormerlySerializedAs("Height")]
    [SerializeField]
    int m_height = 1080;
    [FormerlySerializedAs("Format")]
    [SerializeField]
    CaptureFormat m_fmt = CaptureFormat.RGB_ALPHA;
    [SerializeField]
    CameraMode m_cameraMode = CameraMode.Controlled;

    Disguise.RenderStream.SenderPixelFormat ToSenderFormat(CaptureFormat fmt)
    {
        switch(fmt)
        {
            case CaptureFormat.RGB: return Disguise.RenderStream.SenderPixelFormat.FMT_RGBX;
            case CaptureFormat.RGB_ALPHA: return Disguise.RenderStream.SenderPixelFormat.FMT_RGBA;
            case CaptureFormat.YUV: return Disguise.RenderStream.SenderPixelFormat.FMT_UYVY_422;
            case CaptureFormat.YUV_ALPHA: return Disguise.RenderStream.SenderPixelFormat.FMT_NDI_UYVY_422_A;
            default: return Disguise.RenderStream.SenderPixelFormat.FMT_RGBA;
        }
    }

    bool Validate()
    {
        bool valid = true;
        if (m_width <= 0 || m_height <= 0)
        {
            Debug.LogError("DisguiseCameraCapture: Width/Height cannot be 0 or negative.");
            valid = false;
        }
        return valid;
    }

    // Start is called before the first frame update
    public IEnumerator Start()
    {
        if (Validate() == false)
        {
            Debug.LogError("DisguiseCameraCapture: Validation errors detected, capture cannot start.");
            enabled = false;
            yield break;
        }

        if (PluginEntry.instance.IsAvailable == false)
        {
            Debug.LogError("DisguiseCameraCapture: RenderStream DLL not available, capture cannot start.");
            enabled = false;
            yield break;
        }

        m_frameData = new FrameData();
        m_cameraData = new CameraData();

        m_camera = GetComponent<Camera>();
        m_frameSender = new Disguise.RenderStream.FrameSender(PluginEntry.instance.projectAssetHandle, gameObject.name, m_camera, m_width, m_height, ToSenderFormat(m_fmt));

        if (Application.isPlaying == false)
            yield break;

        while (true)
        {
            yield return new WaitForEndOfFrame();
            if (enabled)
            {
                m_newFrameData |= m_frameSender.AwaitFrameData(500, ref m_frameData, ref m_cameraData);
            }
        }
    }

    // Update is called once per frame
    public void Update()
    {
        // set tracking
        if (m_newFrameData && m_cameraMode == CameraMode.Controlled)
        {
            m_camera.usePhysicalProperties = true;
            m_camera.sensorSize = new Vector2(m_cameraData.sensorX, m_cameraData.sensorY);
            m_camera.focalLength = m_cameraData.focalLength;
            m_camera.lensShift = new Vector2(-m_cameraData.cx / m_cameraData.sensorX, m_cameraData.cy / m_cameraData.sensorY) / 2.0f;

            transform.localPosition = new Vector3(m_cameraData.x, m_cameraData.y, m_cameraData.z);
            transform.localRotation = Quaternion.Euler(new Vector3(-m_cameraData.rx, m_cameraData.ry, -m_cameraData.rz));
        }

        m_frameSender.ProcessQueue();
    }

    public void LateUpdate()
    {
        if (m_newFrameData)
        {
            m_frameSender.EnqueueFrame(m_frameData, m_cameraData);
            m_newFrameData = false;
        }
    }

    public void OnDestroy()
    {
        if (m_frameSender != null)
        {
            m_frameSender.CleanupMaterial();
        }
    }

    public void OnDisable()
    {
        if (m_frameSender != null)
        {
            m_frameSender.ReleaseTexture();
            m_frameSender.DestroyStream();
        }
    }

    Camera m_camera;
    Disguise.RenderStream.FrameSender m_frameSender;

    FrameData m_frameData;
    CameraData m_cameraData;
    bool m_newFrameData = false;
}

namespace Disguise.RenderStream
{
    // d3renderstream/d3renderstream.h
    using StreamHandle = UInt64;
    using AssetHandle = UInt64;
    using CameraHandle = UInt64;

    public enum SenderPixelFormat : UInt32
    {
        // No sampling with alpha
        FMT_BGRA = 0x00000000,
        FMT_RGBA,

        // No sampling with padding
        FMT_BGRX,
        FMT_RGBX,

        // sampling
        FMT_UYVY_422,

        // special
        FMT_NDI_UYVY_422_A, // NDI specific yuv422 format with alpha lines appended to bottom of frame

        // rivermax
        FMT_HQ_YUV422_10BIT,
        FMT_HQ_YUV422_12BIT,
        FMT_HQ_RGB_10BIT,
        FMT_HQ_RGB_12BIT,
        FMT_HQ_RGBA_10BIT,
        FMT_HQ_RGBA_12BIT,
    }

    public enum SenderFrameType : UInt32
    {
        RS_FRAMETYPE_HOST_MEMORY = 0x00000000,
        RS_FRAMETYPE_DX11_TEXTURE,
        RS_FRAMETYPE_DX12_TEXTURE
    };

    public enum RS_ERROR : UInt32
    {
        RS_ERROR_SUCCESS = 0,

        // Core is not initialised
        RS_NOT_INITIALISED,

        // Core is already initialised
        RS_ERROR_ALREADYINITIALISED,

        // Given handle is invalid
        RS_ERROR_INVALIDHANDLE,

        // Maximum number of frame senders have been created
        RS_MAXSENDERSREACHED,

        RS_ERROR_BADSTREAMTYPE,

        RS_ERROR_NOTFOUND,

        RS_ERROR_UNSPECIFIED
    };

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct CameraData
    {
        public StreamHandle streamHandle;
        public CameraHandle cameraHandle;
        public float x, y, z;
        public float rx, ry, rz;
        public float sensorX, sensorY;
        public float focalLength;
        public float cx, cy;
        public float nearZ, farZ;
    };

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct FrameData
    {
        public double tTracked;
        public double localTime;
        double localTimeDelta;
        public UInt32 frameRateNumerator;
        public UInt32 frameRateDenominator;
        public UInt32 flags;
        public UInt32 scene;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct CameraResponseData
    {
        public double tTracked;
        public CameraData camera;
    };

    delegate void logger_t(string message);

    [Serializable]
    public sealed class PluginEntry
    {
        private class Nested
        {
            // Explicit static constructor to tell C# compiler
            // not to mark type as beforefieldinit
            static Nested() { }

            internal static readonly PluginEntry instance = new PluginEntry();
        }

        public static PluginEntry instance { get { return Nested.instance; } }

        unsafe delegate void pGetVersion(ref int major, ref int minor);

        unsafe delegate RS_ERROR pInit();
        unsafe delegate RS_ERROR pShutdown();

        unsafe delegate void pRegisterLoggingFunc(logger_t logger);
        unsafe delegate void pRegisterErrorLoggingFunc(logger_t logger);
        unsafe delegate void pRegisterVerboseLoggingFunc(logger_t logger);

        unsafe delegate void pUnregisterLoggingFunc();
        unsafe delegate void pUnregisterErrorLoggingFunc();
        unsafe delegate void pUnregisterVerboseLoggingFunc();

        unsafe delegate RS_ERROR pCreateAsset(string assetName, ref AssetHandle assetHandle);
        unsafe delegate RS_ERROR pDestroyAsset(ref AssetHandle assetHandle);
        unsafe delegate RS_ERROR pCreateStream(AssetHandle assetHandle, string name, ref StreamHandle streamHandle);
        unsafe delegate RS_ERROR pDestroyStream(AssetHandle assetHandle, ref StreamHandle streamHandle);
        unsafe delegate RS_ERROR pSendFrame(AssetHandle assetHandle, StreamHandle streamHandle, SenderFrameType frameType, IntPtr data, int width, int height, SenderPixelFormat senderFormat, IntPtr sendData);
        unsafe delegate RS_ERROR pAwaitFrameData(ref AssetHandle assetHandle, int timeoutMs, IntPtr data);
        unsafe delegate RS_ERROR pGetFrameCamera(AssetHandle assetHandle, StreamHandle streamHandle, IntPtr outCameraData);

        pGetVersion m_getVersion = null;

        pRegisterLoggingFunc m_registerLoggingFunc = null;
        pRegisterErrorLoggingFunc m_registerErrorLoggingFunc = null;
        pRegisterVerboseLoggingFunc m_registerVerboseLoggingFunc = null;

        pUnregisterLoggingFunc m_unregisterLoggingFunc = null;
        pUnregisterErrorLoggingFunc m_unregisterErrorLoggingFunc = null;
        pUnregisterVerboseLoggingFunc m_unregisterVerboseLoggingFunc = null;

        pInit m_init = null;
        pShutdown m_shutdown = null;
        pCreateAsset m_createAsset = null;
        pDestroyAsset m_destroyAsset = null;
        pCreateStream m_createStream = null;
        pDestroyStream m_destroyStream = null;
        pSendFrame m_sendFrame = null;
        pAwaitFrameData m_awaitFrameData = null;
        pGetFrameCamera m_getFrameCamera = null;

        void logInfo(string message)
        {
            Debug.Log(message);
        }

        void logError(string message)
        {
            Debug.LogError(message);
        }

        public void getVersion(ref int major, ref int minor)
        {
            int vMajor = 0;
            int vMinor = 0;
            if (m_getVersion != null)
            {
                m_getVersion(ref vMajor, ref vMinor);
                major = vMajor;
                minor = vMinor;
            }
        }

        public RS_ERROR createAsset(string name, ref AssetHandle assetHandle)
        {
            if (m_createAsset == null)
                return RS_ERROR.RS_NOT_INITIALISED;
            return m_createAsset(name, ref assetHandle);
        }

        public RS_ERROR destroyAsset(ref AssetHandle assetHandle)
        {
            if (m_destroyAsset == null)
                return RS_ERROR.RS_NOT_INITIALISED;
            return m_destroyAsset(ref assetHandle);
        }

        public RS_ERROR createStream(AssetHandle assetHandle, string name, ref StreamHandle streamHandle)
        {
            if (m_createStream == null)
                return RS_ERROR.RS_NOT_INITIALISED;
            return m_createStream(assetHandle, name, ref streamHandle);
        }

        public RS_ERROR destroyStream(AssetHandle assetHandle, ref StreamHandle streamHandle)
        {
            if (m_destroyStream == null)
                return RS_ERROR.RS_NOT_INITIALISED;
            return m_destroyStream(assetHandle, ref streamHandle);
        }

        public RS_ERROR sendFrame(AssetHandle assetHandle, StreamHandle streamHandle, SenderFrameType frameType, IntPtr data, int width, int height, SenderPixelFormat fmt, CameraResponseData sendData)
        {
            if (m_sendFrame == null)
                return RS_ERROR.RS_NOT_INITIALISED;

            if (handleReference.IsAllocated)
                handleReference.Free();
            handleReference = GCHandle.Alloc(sendData, GCHandleType.Pinned);
            try
            {
                RS_ERROR error = m_sendFrame(assetHandle, streamHandle, frameType, data, width, height, fmt, handleReference.AddrOfPinnedObject());
                return error;
            }
            finally
            {
                if (handleReference.IsAllocated)
                    handleReference.Free();
            }
            //return RS_ERROR.RS_ERROR_UNSPECIFIED;
        }

        public RS_ERROR awaitFrameData(ref AssetHandle assetHandle, int timeoutMs, ref FrameData data)
        {
            if (m_awaitFrameData == null)
                return RS_ERROR.RS_NOT_INITIALISED;

            if (handleReference.IsAllocated)
                handleReference.Free();
            handleReference = GCHandle.Alloc(data, GCHandleType.Pinned);
            try
            {
                RS_ERROR error = m_awaitFrameData(ref assetHandle, timeoutMs, handleReference.AddrOfPinnedObject());
                if (error == RS_ERROR.RS_ERROR_SUCCESS)
                {
                    data = (FrameData)Marshal.PtrToStructure(handleReference.AddrOfPinnedObject(), typeof(FrameData));
                }
                return error;
            }
            finally
            {
                if (handleReference.IsAllocated)
                    handleReference.Free();
            }
            //return RS_ERROR.RS_ERROR_UNSPECIFIED;
        }

        public RS_ERROR getFrameCamera(AssetHandle assetHandle, StreamHandle streamHandle, ref CameraData outCameraData)
        {
            if (m_getFrameCamera == null)
                return RS_ERROR.RS_NOT_INITIALISED;

            if (handleReference.IsAllocated)
                handleReference.Free();
            handleReference = GCHandle.Alloc(outCameraData, GCHandleType.Pinned);
            try
            {
                RS_ERROR error = m_getFrameCamera(assetHandle, streamHandle, handleReference.AddrOfPinnedObject());
                if (error == RS_ERROR.RS_ERROR_SUCCESS)
                {
                    outCameraData = (CameraData)Marshal.PtrToStructure(handleReference.AddrOfPinnedObject(), typeof(CameraData));
                }
                return error;
            }
            finally
            {
                if (handleReference.IsAllocated)
                    handleReference.Free();
            }
            //return RS_ERROR.RS_ERROR_UNSPECIFIED;
        }

#if PLUGIN_AVAILABLE

        [DllImport("kernel32.dll")]
        static extern IntPtr LoadLibraryEx(string lpLibFileName, IntPtr fileHandle, int flags);

        [DllImport("kernel32.dll")]
        static extern IntPtr GetProcAddress(IntPtr hModule, string lpProcName);

        [DllImport("kernel32.dll", CharSet = CharSet.Ansi)]
        static extern bool FreeLibrary(IntPtr hModule);

        private void free()
        {
            RS_ERROR error;
            if (assetHandle != 0)
            {
                error = destroyAsset(ref assetHandle);
                if (error != RS_ERROR.RS_ERROR_SUCCESS)
                    Debug.LogError(string.Format("Failed to destroy asset {0}: {1}", name, error));
                assetHandle = 0;
            }

            error = m_shutdown();
            if (error != RS_ERROR.RS_ERROR_SUCCESS)
                Debug.LogError(string.Format("Failed to shutdown: {0}", error));

            if (d3RenderStreamDLL != IntPtr.Zero)
                FreeLibrary(d3RenderStreamDLL);
        }

        public bool IsAvailable
        {
            get
            {
                UnityEngine.Rendering.GraphicsDeviceType gapi = UnityEngine.SystemInfo.graphicsDeviceType;
                return functionsLoaded && (gapi == UnityEngine.Rendering.GraphicsDeviceType.Direct3D11 ||
                       gapi == UnityEngine.Rendering.GraphicsDeviceType.Metal);
            }
        }
#else
        private void free() {}
        public bool IsAvailable { get { return false; } }
#endif

        const int LOAD_IGNORE_CODE_AUTHZ_LEVEL = 0x00000010;
        const int LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR = 0x00000100;
        const int LOAD_LIBRARY_SEARCH_APPLICATION_DIR = 0x00000200;
        const int LOAD_LIBRARY_SEARCH_USER_DIRS = 0x00000400;
        const int LOAD_LIBRARY_SEARCH_SYSTEM32 = 0x00000800;

        const string _dllName = "d3renderstream";

        const int RENDER_STREAM_VERSION_MAJOR = 1;
        const int RENDER_STREAM_VERSION_MINOR = 7;

        bool functionsLoaded = false;
        IntPtr d3RenderStreamDLL = IntPtr.Zero;
        GCHandle handleReference; // Everything is run under coroutines with odd lifetimes, so store a reference to GCHandle

        string name;
        AssetHandle assetHandle = 0;

        // https://answers.unity.com/questions/16804/retrieving-project-name.html?childToView=478633#answer-478633
        public string GetProjectName()
        {
            string[] s = Application.dataPath.Split('/');
            if (s.Length >= 2)
            {
                string projectName = s[s.Length - 2];
                return projectName;
            }
            return "UNKNOWN UNITY PROJECT";
        }

        public AssetHandle projectAssetHandle { get { return assetHandle; } }

        private PluginEntry()
        {
#if PLUGIN_AVAILABLE
            RegistryKey d3Key = Registry.CurrentUser.OpenSubKey("Software")?.OpenSubKey("d3 Technologies")?.OpenSubKey("d3 Production Suite");
            if (d3Key == null)
            {
                Debug.LogError(string.Format("Failed to find path to {0}.dll. d3 Not installed?", _dllName));
                return;
            }

            string d3ExePath = d3Key.GetValue("exe path").ToString();
            int endSeparator = d3ExePath.LastIndexOf(Path.DirectorySeparatorChar);
            if (endSeparator != d3ExePath.Length - 1)
                d3ExePath = d3ExePath.Substring(0, endSeparator + 1);

            string libPath = d3ExePath + _dllName + ".dll";
            d3RenderStreamDLL = LoadWin32Library(libPath);
            if (d3RenderStreamDLL == IntPtr.Zero)
            {
                Debug.LogError(string.Format("Failed to load {0}.dll from {1}", _dllName, d3ExePath));
                return;
            }

            m_getVersion = DelegateBuilder<pGetVersion>(d3RenderStreamDLL, "rs_getVersion");

            m_registerLoggingFunc = DelegateBuilder<pRegisterLoggingFunc>(d3RenderStreamDLL, "rs_registerLoggingFunc");
            m_registerErrorLoggingFunc = DelegateBuilder<pRegisterErrorLoggingFunc>(d3RenderStreamDLL, "rs_registerErrorLoggingFunc");
            m_registerVerboseLoggingFunc = DelegateBuilder<pRegisterVerboseLoggingFunc>(d3RenderStreamDLL, "rs_registerVerboseLoggingFunc");

            m_unregisterLoggingFunc = DelegateBuilder<pUnregisterLoggingFunc>(d3RenderStreamDLL, "rs_unregisterLoggingFunc");
            m_unregisterErrorLoggingFunc = DelegateBuilder<pUnregisterErrorLoggingFunc>(d3RenderStreamDLL, "rs_unregisterErrorLoggingFunc");
            m_unregisterVerboseLoggingFunc = DelegateBuilder<pUnregisterVerboseLoggingFunc>(d3RenderStreamDLL, "rs_unregisterVerboseLoggingFunc");

            m_init = DelegateBuilder<pInit>(d3RenderStreamDLL, "rs_init");
            m_shutdown = DelegateBuilder<pShutdown>(d3RenderStreamDLL, "rs_shutdown");
            m_createAsset = DelegateBuilder<pCreateAsset>(d3RenderStreamDLL, "rs_createAsset");
            m_destroyAsset = DelegateBuilder<pDestroyAsset>(d3RenderStreamDLL, "rs_destroyAsset");
            m_createStream = DelegateBuilder<pCreateStream>(d3RenderStreamDLL, "rs_createStream");
            m_destroyStream = DelegateBuilder<pDestroyStream>(d3RenderStreamDLL, "rs_destroyStream");
            m_sendFrame = DelegateBuilder<pSendFrame>(d3RenderStreamDLL, "rs_sendFrame");
            m_awaitFrameData = DelegateBuilder<pAwaitFrameData>(d3RenderStreamDLL, "rs_awaitFrameData");
            m_getFrameCamera = DelegateBuilder<pGetFrameCamera>(d3RenderStreamDLL, "rs_getFrameCamera");

            if (m_getVersion == null)
            {
                Debug.LogError(string.Format("getVersion functions failed load from {0}.dll", _dllName));
                return;
            }

            int major = 0;
            int minor = 0;
            getVersion(ref major, ref minor);
            Debug.Log(string.Format("{0}.dll v{1}.{2} successfully loaded.", _dllName, major, minor));

            if (major != RENDER_STREAM_VERSION_MAJOR || minor != RENDER_STREAM_VERSION_MINOR)
            {
                Debug.LogError(string.Format("Unsupported RenderStream library, expected version {0}.{1}", RENDER_STREAM_VERSION_MAJOR, RENDER_STREAM_VERSION_MINOR));
                return;
            }

            if (m_init == null || m_shutdown == null || m_createAsset == null || m_destroyAsset == null || m_createStream == null || m_destroyStream == null || m_sendFrame == null || m_awaitFrameData == null || m_getFrameCamera == null)
            {
                Debug.LogError(string.Format("One or more functions failed load from {0}.dll", _dllName));
                return;
            }

            functionsLoaded = true;

            if (m_registerLoggingFunc != null)
                m_registerLoggingFunc(logInfo);
            if (m_registerErrorLoggingFunc != null)
                m_registerErrorLoggingFunc(logError);

            RS_ERROR error = m_init();
            if (error != RS_ERROR.RS_ERROR_SUCCESS)
                Debug.LogError(string.Format("Failed to initialise: {0}", error));

            name = GetProjectName();
            error = createAsset(name, ref assetHandle);
            if (error != RS_ERROR.RS_ERROR_SUCCESS)
                Debug.LogError(string.Format("Failed to create asset {0}: {1}", name, error));

#else
            Debug.LogError(string.Format("{0}.dll is only available on Windows", _dllName));
#endif
        }

        ~PluginEntry()
        {
            if (handleReference.IsAllocated)
                handleReference.Free();
            free();
        }

        static IntPtr LoadWin32Library(string dllFilePath)
        {
            System.IntPtr moduleHandle = LoadLibraryEx(dllFilePath, IntPtr.Zero, LOAD_IGNORE_CODE_AUTHZ_LEVEL | LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_APPLICATION_DIR | LOAD_LIBRARY_SEARCH_SYSTEM32 | LOAD_LIBRARY_SEARCH_USER_DIRS);
            if (moduleHandle == IntPtr.Zero)
            {
                // I'm gettin last dll error
                int errorCode = Marshal.GetLastWin32Error();
                Debug.LogError(string.Format("There was an error during dll loading : {0}, error - {1}", dllFilePath, errorCode));
            }
            return moduleHandle;
        }

        static T DelegateBuilder<T>(IntPtr loadedDLL, string functionName) where T : Delegate
        {
            IntPtr pAddressOfFunctionToCall = GetProcAddress(loadedDLL, functionName);
            if (pAddressOfFunctionToCall == IntPtr.Zero)
            {
                return null;
            }
            T functionDelegate = Marshal.GetDelegateForFunctionPointer(pAddressOfFunctionToCall, typeof(T)) as T;
            return functionDelegate;
        }
    }

    public class FrameSender
    {
        struct Frame
        {
            public int width, height;
            public SenderPixelFormat fmt;
            public AsyncGPUReadbackRequest readback;
            public CameraResponseData responseData;
        }

        private FrameSender() { }
        public FrameSender(AssetHandle assetHandle, string name, Camera cam, int width, int height, SenderPixelFormat fmt)
        {
            m_assetHandle = assetHandle;
            m_name = name;
            Cam = cam;
            Width = width;
            Height = height;
            Format = fmt;

            m_lastFrameCount = -1;
            m_streamHandle = 0;

            if (m_streamHandle == 0)
            {
                RS_ERROR error = PluginEntry.instance.createStream(m_assetHandle, m_name, ref m_streamHandle);
                if (error != RS_ERROR.RS_ERROR_SUCCESS)
                    Debug.LogError(string.Format("Failed to create stream {0}: {1}", m_name, error));
            }

            RenderTextureDescriptor desc = new RenderTextureDescriptor(Width, Height, RenderTextureFormat.ARGB32, 24);
            m_sourceTex = new RenderTexture(desc)
            {
                name = m_name + " Texture"
            };
            Cam.targetTexture = m_sourceTex;
        }

        public bool AwaitFrameData(int timeoutMs, ref FrameData frameData, ref CameraData cameraData)
        {
            if (m_assetHandle == 0)
                return false;

            AssetHandle handle = 0;
            RS_ERROR error = PluginEntry.instance.awaitFrameData(ref handle, timeoutMs, ref frameData);
            if (error == RS_ERROR.RS_ERROR_SUCCESS && handle == m_assetHandle)
            {
                error = PluginEntry.instance.getFrameCamera(m_assetHandle, m_streamHandle, ref cameraData);
            }
            return (error == RS_ERROR.RS_ERROR_SUCCESS && handle == m_assetHandle);
        }

        public void ProcessQueue()
        {
            while (FrameQueueEmpty() == false)
            {
                Frame frame = m_frameQueue.Peek();

                if (Application.isPlaying == false)
                    frame.readback.WaitForCompletion();

                if (frame.readback.hasError)
                {
                    Debug.LogWarning("GPU readback error was detected");
                    m_frameQueue.Dequeue();
                    continue;
                }

                // wait until next frame if in playmode, don't block the application
                if (frame.readback.done == false)
                    break;

                unsafe
                {
                    RS_ERROR error = PluginEntry.instance.sendFrame(m_assetHandle, m_streamHandle, SenderFrameType.RS_FRAMETYPE_DX11_TEXTURE, (IntPtr)frame.readback.GetData<Byte>().GetUnsafeReadOnlyPtr(),
                                                                    frame.width, frame.height, frame.fmt, frame.responseData);
                    if (error != RS_ERROR.RS_ERROR_SUCCESS)
                        Debug.LogError(string.Format("Error sending frame: {0}", error));
                }

                m_frameQueue.Dequeue();
            }
        }

        public void EnqueueFrame(FrameData frameData, CameraData cameraData)
        {
            if (FrameQueueFull())
            {
                Debug.LogWarning("Too many GPU readback requests.");
                return;
            }

            if (m_lastFrameCount == Time.frameCount)
                return;

            m_lastFrameCount = Time.frameCount;

            ReleaseTexture();
            int width = (int)(m_sourceTex.width * FormatWidthMultiplier());
            int height = (int)(m_sourceTex.height * FormatHeightMultiplier());
            m_convertedTex = RenderTexture.GetTemporary(width, height, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);

            if (m_material == null)
            {
                m_material = new Material(Shader.Find("Hidden/DisguiseRenderStream/Sender"));
                m_material.hideFlags = HideFlags.DontSave;
            }

            Graphics.Blit(m_sourceTex, m_convertedTex, m_material, ShaderPass());

            CameraResponseData responseData = new CameraResponseData();
            responseData.tTracked = frameData.tTracked;
            responseData.camera = cameraData;

            m_frameQueue.Enqueue(new Frame
            {
                width = m_sourceTex.width,
                height = m_sourceTex.height,
                fmt = Format,
                readback = AsyncGPUReadback.Request(m_convertedTex),
                responseData = responseData
            });
        }

        public void ReleaseTexture()
        {
            if (m_convertedTex != null)
            {
                RenderTexture.ReleaseTemporary(m_convertedTex);
                m_convertedTex = null;
            }
        }

        public void DestroyStream()
        {
            if (m_streamHandle != 0)
            {
                RS_ERROR error = PluginEntry.instance.destroyStream(m_assetHandle, ref m_streamHandle);
                if (error != RS_ERROR.RS_ERROR_SUCCESS)
                    Debug.LogError(string.Format("Failed to destroy stream: {0}", error));
                m_streamHandle = 0;
            }
        }

        public void CleanupMaterial()
        {
            if (m_material != null)
            {
                if (Application.isPlaying)
                    UnityEngine.Object.Destroy(m_material);
                else
                    UnityEngine.Object.DestroyImmediate(m_material);
                m_material = null;
            }
        }

        float FormatHeightMultiplier()
        {
            switch (Format)
            {
                case SenderPixelFormat.FMT_NDI_UYVY_422_A: return 1.5f;
                default: return 1.0f;
            }
        }

        float FormatWidthMultiplier()
        {
            switch (Format)
            {
                case SenderPixelFormat.FMT_UYVY_422:
                case SenderPixelFormat.FMT_NDI_UYVY_422_A:
                    return 0.5f;
                default:
                    return 1.0f;
            }
        }

        bool FrameQueueEmpty()
        {
            return m_frameQueue.Count == 0;
        }

        bool FrameQueueFull()
        {
            return m_frameQueue.Count >= MaxQueueSize;
        }

        int ShaderPass()
        {
            switch (Format)
            {
                case SenderPixelFormat.FMT_UYVY_422: return 0;
                case SenderPixelFormat.FMT_NDI_UYVY_422_A: return 1;
                default: return 2;
            }
        }

        public int Width { get; set; }
        public int Height { get; set; }
        public SenderPixelFormat Format { get; set; }
        public Camera Cam { get; set; }

        private RenderTexture m_sourceTex;

        string m_name;
        Material m_material;
        RenderTexture m_convertedTex;
        int m_lastFrameCount;

        AssetHandle m_assetHandle;
        StreamHandle m_streamHandle;

        const int MaxQueueSize = 4;
        Queue<Frame> m_frameQueue = new Queue<Frame>(MaxQueueSize);
    }

}
