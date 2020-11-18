using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using com.rfilkov.kinect;


namespace com.rfilkov.components
{
    public class ForegroundBlendRenderer : MonoBehaviour
    {
        [Tooltip("Reference to background removal manager. If left to None, it looks up the first available BR-manager in the scene.")]
        public BackgroundRemovalManager backgroundRemovalManager = null;

        [Tooltip("Depth value in meters, used for invalid depth points.")]
        public float invalidDepthValue = 0f;

        [Tooltip("Whether to maximize the rendered object on the screen, or not.")]
        public bool maximizeOnScreen = true;

        [Tooltip("Whether to apply per-pixel lighting on the foreground, or not.")]
        public bool applyLighting = false;


        // references to KM and data
        private KinectManager kinectManager = null;
        private KinectInterop.SensorData sensorData = null;
        private DepthSensorBase sensorInt = null;
        private Material matRenderer = null;

        // depth image buffer
        private ComputeBuffer depthImageBuffer = null;

        // textures
        private Texture alphaTex = null;
        private Texture colorTex = null;

        // lighting structures
        private Light[] sceneLights = null;
        private Bounds lightBounds;

        private Vector4[] dirLightData = new Vector4[2];

        [SerializeField]
        public struct PointLight
        {
            public Vector4 color;
            public float range;
            public Vector3 pos;
        }

        private const int SIZE_POINT_LIGHT = 32;
        private const int MAX_POINT_LIGHTS = 8;

        [SerializeField]
        private PointLight[] pointLights = new PointLight[MAX_POINT_LIGHTS];
        private ComputeBuffer pointLightsBuffer = null;
        private int pointLightsNumber = 0;

        [SerializeField]
        public struct SpotLight
        {
            public Vector4 color;
            public Vector3 pos;
            public Vector4 dir;
            public Vector4 pars;
        }

        private const int SIZE_SPOT_LIGHT = 60;
        private const int MAX_SPOT_LIGHTS = 8;

        [SerializeField]
        public SpotLight[] spotLights = new SpotLight[MAX_SPOT_LIGHTS];
        private ComputeBuffer spotLightsBuffer = null;
        private int spotLightsNumber = 0;

        const int NUMBER_LIGHTS_MAX = MAX_POINT_LIGHTS / 2 + MAX_SPOT_LIGHTS / 2;


        void Start()
        {
            kinectManager = KinectManager.Instance;

            if (backgroundRemovalManager == null)
            {
                backgroundRemovalManager = FindObjectOfType<BackgroundRemovalManager>();
            }

            Renderer meshRenderer = GetComponent<Renderer>();
            if (meshRenderer && meshRenderer.material /**&& meshRenderer.material.mainTexture == null*/)
            {
                matRenderer = meshRenderer.material;
            }

            if (kinectManager && kinectManager.IsInitialized() && backgroundRemovalManager && backgroundRemovalManager.enabled)
            {
                // get sensor data
                sensorData = kinectManager.GetSensorData(backgroundRemovalManager.sensorIndex);
                sensorInt = sensorData != null ? (DepthSensorBase)sensorData.sensorInterface : null;
            }

            // find scene lights
            sceneLights = GameObject.FindObjectsOfType<Light>();
            lightBounds = new Bounds(transform.position, new Vector3(20f, 20f, 20f));

            //Debug.Log("sceneLights: " + sceneLights.Length);
            //for(int i = 0; i < sceneLights.Length; i++)
            //{
            //    Debug.Log(i.ToString() + " - " + sceneLights[i].name + " - " + sceneLights[i].type);
            //}
        }


        void OnDestroy()
        {
            if (sensorData != null && sensorData.colorDepthBuffer != null)
            {
                sensorData.colorDepthBuffer.Release();
                sensorData.colorDepthBuffer = null;
            }

            if (depthImageBuffer != null)
            {
                //depthImageCopy = null;

                depthImageBuffer.Release();
                depthImageBuffer = null;
            }

            if (pointLightsBuffer != null)
            {
                pointLightsBuffer.Release();
                pointLightsBuffer = null;
            }

            if (spotLightsBuffer != null)
            {
                spotLightsBuffer.Release();
                spotLightsBuffer = null;
            }
        }


        void Update()
        {
            if (matRenderer == null || sensorInt == null)
                return;

            if(alphaTex == null)
            {
                // alpha texture
                alphaTex = backgroundRemovalManager.GetAlphaTex();

                if(alphaTex != null)
                {
                    matRenderer.SetTexture("_AlphaTex", alphaTex);
                }
            }

            if(colorTex == null)
            {
                // color texture
                colorTex = !backgroundRemovalManager.computeAlphaMaskOnly ? backgroundRemovalManager.GetForegroundTex() : alphaTex;  // sensorInt.pointCloudColorTexture

                if (colorTex != null)
                {
                    matRenderer.SetInt("_TexResX", colorTex.width);
                    matRenderer.SetInt("_TexResY", colorTex.height);

                    matRenderer.SetTexture("_ColorTex", colorTex);
                }
            }

            if (colorTex == null || alphaTex == null /**|| foregroundCamera == null*/)
                return;

            if (depthImageBuffer != null && sensorData.depthImage != null)
            {
                //KinectInterop.CopyBytes(sensorData.depthImage, sizeof(ushort), depthImageCopy, sizeof(ushort));

                int depthBufferLength = sensorData.depthImageWidth * sensorData.depthImageHeight / 2;
                KinectInterop.SetComputeBufferData(depthImageBuffer, sensorData.depthImage, depthBufferLength, sizeof(uint));
            }

            if (sensorInt.pointCloudResolution == DepthSensorBase.PointCloudResolution.DepthCameraResolution)
            {
                if (depthImageBuffer == null)
                {
                    //int depthImageLength = sensorData.depthImageWidth * sensorData.depthImageHeight;
                    //depthImageCopy = new ushort[depthImageLength];

                    int depthBufferLength = sensorData.depthImageWidth * sensorData.depthImageHeight / 2;
                    depthImageBuffer = KinectInterop.CreateComputeBuffer(depthImageBuffer, depthBufferLength, sizeof(uint));

                    ScaleRendererTransform(colorTex);
                }

                matRenderer.SetBuffer("_DepthMap", depthImageBuffer);
                //Debug.Log("SceneMeshGpu DepthFrameTime: " + lastDepthFrameTime);
            }
            else
            {
                if (sensorData.colorDepthBuffer == null)
                {
                    int bufferLength = sensorData.colorImageWidth * sensorData.colorImageHeight / 2;
                    sensorData.colorDepthBuffer = new ComputeBuffer(bufferLength, sizeof(uint));

                    ScaleRendererTransform(colorTex);
                }

                matRenderer.SetBuffer("_DepthMap", sensorData.colorDepthBuffer);
                //Debug.Log("SceneMeshGpu ColorDepthBufferTime: " + sensorData.lastColorDepthBufferTime);
            }

            matRenderer.SetFloat("_InvDepthVal", invalidDepthValue);

            if(applyLighting)
            {
                matRenderer.SetInt("_ApplyLights", 1);
                matRenderer.SetInt("_ApplyShadows", 0);
                matRenderer.SetFloat("_Metallic", 0);

                ApplyLighting();
            }
            else
            {
                matRenderer.SetInt("_ApplyLights", 0);
            }
        }

        // scales the renderer's transform properly
        private void ScaleRendererTransform(Texture colorTex)
        {
            Vector3 localScale = transform.localScale;

            if (maximizeOnScreen)
            {
                Camera camera = Camera.main;
                float objectZ = transform.position.z;

                float screenW = Screen.width;
                float screenH = Screen.height;

                Vector3 vLeft = camera.ScreenToWorldPoint(new Vector3(0f, screenH / 2f, objectZ));
                Vector3 vRight = camera.ScreenToWorldPoint(new Vector3(screenW, screenH / 2f, objectZ));
                float distLeftRight = (vRight - vLeft).magnitude;

                Vector3 vBottom = camera.ScreenToWorldPoint(new Vector3(screenW / 2f, 0f, objectZ));
                Vector3 vTop = camera.ScreenToWorldPoint(new Vector3(screenW / 2f, screenH, objectZ));
                float distBottomTop = (vTop - vBottom).magnitude;

                localScale.x = distLeftRight / localScale.x;
                localScale.y = distBottomTop / localScale.y;
            }

            // scale according to color-tex resolution
            //localScale.y = localScale.x * colorTex.height / colorTex.width;

            // apply color image scale
            Vector3 colorImageScale = kinectManager.GetColorImageScale(backgroundRemovalManager.sensorIndex);
            if (colorImageScale.x < 0f)
                localScale.x = -localScale.x;
            if (colorImageScale.y < 0f)
                localScale.y = -localScale.y;

            transform.localScale = localScale;
        }

        // applies the current lights
        private void ApplyLighting()
        {
            const float interiorCone = 0.1f;  // interior cone of the spotlight
            dirLightData[1] = Vector4.zero;

            int pi = 0;
            int si = 0;

            foreach (Light light in sceneLights)
            {
                if (!light.gameObject.activeInHierarchy || !light.enabled)
                    continue;

                if (light.type == LightType.Directional || Vector3.Distance(lightBounds.center, light.transform.position) < (light.range + lightBounds.extents.x))
                {
                    if (light.type != LightType.Directional && light.shadows != LightShadows.None)
                    {
                        light.shadows = LightShadows.None;
                    }

                    if (light.type == LightType.Point)
                    {
                        if (pi < MAX_POINT_LIGHTS)
                        {
                            pointLights[pi].color = light.color * light.intensity;
                            pointLights[pi].pos = light.gameObject.transform.position;
                            pointLights[pi].range = light.range;

                            pi++;
                        }
                    }
                    else if (light.type == LightType.Spot)
                    {
                        if (si < MAX_SPOT_LIGHTS)
                        {
                            Vector3 vLightFwd = light.gameObject.transform.forward.normalized;

                            spotLights[si].color = light.color * light.intensity;
                            spotLights[si].pos = light.gameObject.transform.position;
                            spotLights[si].dir = new Vector4(vLightFwd.x, vLightFwd.y, vLightFwd.z, Mathf.Cos((light.spotAngle / 2.0f) * Mathf.Deg2Rad));
                            spotLights[si].pars = new Vector4(light.spotAngle, light.intensity, 1.0f / light.range, interiorCone);

                            si++;
                        }
                    }
                    else if (light.type == LightType.Directional)
                    {
                        Vector3 vLightFwd = light.gameObject.transform.forward.normalized;
                        dirLightData[0] = new Vector4(vLightFwd.x, vLightFwd.y, vLightFwd.z, 0);
                        dirLightData[1] = light.color * light.intensity;
                    }

                }
            }

            if (pointLightsBuffer == null)
            {
                pointLightsBuffer = new ComputeBuffer(MAX_POINT_LIGHTS, SIZE_POINT_LIGHT);
                pointLightsBuffer.SetData(pointLights);

                matRenderer.SetBuffer("_PointLights", pointLightsBuffer);
            }
            else
            {
                pointLightsBuffer.SetData(pointLights);
            }

            if (spotLightsBuffer == null)
            {
                spotLightsBuffer = new ComputeBuffer(MAX_SPOT_LIGHTS, SIZE_SPOT_LIGHT);
                spotLightsBuffer.SetData(spotLights);
                matRenderer.SetBuffer("_SpotLights", spotLightsBuffer);
            }
            else
            {
                spotLightsBuffer.SetData(spotLights);
            }

            pointLightsNumber = pi;
            spotLightsNumber = si;

            matRenderer.SetInt("_PointLightsNumber", pointLightsNumber);
            matRenderer.SetInt("_SpotLightsNumber", spotLightsNumber);

            matRenderer.SetVectorArray("_DirectionalLights", dirLightData);
        }

    }
}
