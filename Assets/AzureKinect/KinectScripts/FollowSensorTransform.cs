using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using com.rfilkov.kinect;


namespace com.rfilkov.components
{
    /// <summary>
    /// This component makes the game object follow the position and rotation of the sensor.
    /// </summary>
    public class FollowSensorTransform : MonoBehaviour
    {
        [Tooltip("Depth sensor index - 0 is the 1st one, 1 - the 2nd one, etc.")]
        public int sensorIndex = 0;

        [Tooltip("Smooth factor used for the game object movement and rotation.")]
        public float smoothFactor = 0f;

        [Tooltip("Whether to use the depth camera pose or the sensor transform position and rotation.")]
        public bool useDepthCameraPose = true;


        // reference to the KinectManager
        private KinectManager kinectManager = null;

        // sensor position and rotation
        Vector3 sensorWorldPos = Vector3.zero;
        Quaternion sensorWorldRot = Quaternion.identity;


        void Start()
        {
            // get reference to KinectManager
            kinectManager = KinectManager.Instance;
        }

        void Update()
        {
            if(kinectManager && kinectManager.IsInitialized())
            {
                if(useDepthCameraPose)
                {
                    Matrix4x4 sensorToWorldMat = kinectManager.GetSensorToWorldMatrix(sensorIndex);
                    sensorWorldPos = sensorToWorldMat.GetColumn(3);
                    sensorWorldRot = sensorToWorldMat.rotation;
                }
                else
                {
                    Transform sensorTrans = kinectManager.GetSensorTransform(sensorIndex);
                    sensorWorldPos = sensorTrans.position;
                    sensorWorldRot = sensorTrans.rotation;
                }

                if (smoothFactor != 0f)
                {
                    transform.position = Vector3.Lerp(transform.position, sensorWorldPos, smoothFactor * Time.deltaTime);
                    transform.rotation = Quaternion.Slerp(transform.rotation, sensorWorldRot, smoothFactor * Time.deltaTime);
                }
                else
                {
                    transform.position = sensorWorldPos;
                    transform.rotation = sensorWorldRot;
                }
            }
        }

    }
}
