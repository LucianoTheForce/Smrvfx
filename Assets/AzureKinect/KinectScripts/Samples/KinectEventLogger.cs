using com.rfilkov.kinect;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace com.rfilkov.components
{
    /// <summary>
    /// KinectEventLogger is a simple class, containing log methods for testing KinectEventManager.
    /// </summary>
    public class KinectEventLogger : MonoBehaviour
    {

        public void OnDepthSensorsStarted()
        {
            Debug.Log("OnDepthSensorsStarted");
        }

        public void OnDepthSensorsStopped()
        {
            Debug.Log("OnDepthSensorsStopped");
        }

        public void OnSensorDisconnect(ulong lastFrameTime)
        {
            Debug.Log("OnSensorDisconnect at time: " + lastFrameTime);
        }

        public void OnNewColorImage(Texture colorTex, ulong lastFrameTime)
        {
            Debug.Log("OnNewColorImage at time: " + lastFrameTime);
        }

        public void OnNewDepthFrame(ushort[] depthFrame, ulong lastFrameTime)
        {
            Debug.Log("OnNewDepthFrame at time: " + lastFrameTime);
        }

        public void OnNewInfraredFrame(ushort[] infraredFrame, ulong lastFrameTime)
        {
            Debug.Log("OnNewInfraredFrame at time: " + lastFrameTime);
        }

        public void OnNewBodyFrame(KinectInterop.BodyData[] alBodies, uint bodyCount, ulong lastFrameTime)
        {
            Debug.Log("OnNewBodyFrame at time: " + lastFrameTime);
        }

        public void OnNewBodyIndexFrame(byte[] bodyIndexFrame, ulong lastFrameTime)
        {
            Debug.Log("OnNewBodyIndexFrame at time: " + lastFrameTime);
        }

        public void OnNewDepthImage(Texture colorTex, ulong lastFrameTime)
        {
            Debug.Log("OnNewDepthImage at time: " + lastFrameTime);
        }

        public void OnNewInfraredImage(Texture colorTex, ulong lastFrameTime)
        {
            Debug.Log("OnNewInfraredImage at time: " + lastFrameTime);
        }

        public void OnNewBodyIndexImage(Texture colorTex, ulong lastFrameTime)
        {
            Debug.Log("OnNewBodyIndexImage at time: " + lastFrameTime);
        }

        public void OnNewDepthCameraColorImage(Texture colorTex, ulong lastFrameTime)
        {
            Debug.Log("OnNewDepthCameraColorImage at time: " + lastFrameTime);
        }

        public void OnNewColorCameraDepthFrame(ushort[] depthFrame, ulong lastFrameTime)
        {
            Debug.Log("OnNewColorCameraDepthFrame at time: " + lastFrameTime);
        }

        public void OnNewColorCameraBodyIndexFrame(byte[] bodyIndexFrame, ulong lastFrameTime)
        {
            Debug.Log("OnNewColorCameraBodyIndexFrame at time: " + lastFrameTime);
        }

    }
}

