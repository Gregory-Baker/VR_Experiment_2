using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR.InteractionSystem;

namespace Valve.VR.InteractionSystem
{
    public class TurnRobotToPlayer : MonoBehaviour
    {
        public SteamVR_Action_Boolean confirmTargetAction;
        public Hand hand;

        public GameObject player;

        public float angularSpeed = 10f;

        bool turning = false;

        Vector3 previousRobotPos;

        private void OnEnable()
        {
            if (hand == null)
                hand = this.GetComponent<Hand>();

            if (confirmTargetAction == null)
            {
                Debug.LogError("<b>[SteamVR Interaction]</b> No action assigned");
                return;
            }

            confirmTargetAction.AddOnChangeListener(OnConfirmActionChange, hand.handType);
            previousRobotPos = transform.position;
        }

        private void OnDisable()
        {
            if (confirmTargetAction != null)
                confirmTargetAction.RemoveOnChangeListener(OnConfirmActionChange, hand.handType);
        }

        private void OnConfirmActionChange(SteamVR_Action_Boolean actionIn, SteamVR_Input_Sources inputSource, bool newValue)
        {
            if (newValue)
            {
                if (IsRobotStationary())
                {
                    turning = true;
                }                
            }
        }

        private bool IsRobotStationary()
        {
            Vector3 robotPositionDiff = transform.position - previousRobotPos;
            print(robotPositionDiff);
            if (robotPositionDiff.magnitude < 3*Mathf.Epsilon)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (turning)
            {
                if (Mathf.Abs(AngleToPlayer()) < 0.1f)
                {
                    turning = false;
                }
                else if (AngleToPlayer() > 0)
                {
                    transform.Rotate(Vector3.up, angularSpeed * Time.deltaTime);
                    player.transform.Rotate(Vector3.up, -angularSpeed * Time.deltaTime);
                }
                else
                {
                    transform.Rotate(Vector3.up, -angularSpeed * Time.deltaTime);
                    player.transform.Rotate(Vector3.up, angularSpeed * Time.deltaTime);
                }
            }
            previousRobotPos = transform.position;
        }

        private float AngleToPlayer()
        {
            float robotAngle = transform.eulerAngles.y;
            float playerAngle = player.transform.eulerAngles.y;
            float angleDiff = playerAngle - robotAngle;
            if (angleDiff > 180)
            {
                angleDiff -= 360;
            }
            else if (angleDiff < -180)
            {
                angleDiff += 360;
            }

            return angleDiff;
        }
    }
}
