using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR.InteractionSystem;

namespace Valve.VR.InteractionSystem
{
    //---------------------------
    public class ConfirmTarget : MonoBehaviour
    {
        public SteamVR_Action_Boolean confirmTargetAction;
        public Hand hand;

        public GameObject confirmedTarget;
        public GameObject interimTarget;
        public GameObject selectedTarget;
        public GameObject robot;

        public float verticalOffset = 0.01f;

        [Header("Stop and Turn")]
        public bool stopAndTurnOn = true;
        public float robotRad;
        public float targetInFrontAngle = 45f;
        public float targetBehindAngle = 135f;
        public float robotTurnRadius = 6f;
        public float angularSpeed = 10f;
        public float veryCloseToTargetRad = 3.5f;
        public float targetCloseFrontAngle = 20f;


        Vector3 robotToTargetVector;
        float angleToTarget;
        float distanceToTarget;

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
                MoveTarget(confirmedTarget, robot.transform.position); //  + 0.1f * robot.transform.forward
                MoveTarget(interimTarget, robot.transform.position);

                Vector3 targetPosition = selectedTarget.transform.position;
                targetPosition.y += verticalOffset;
                
                if (TargetInStopAndTurnZone())
                {
                    MoveTarget(interimTarget, targetPosition);
                }
                else
                {
                    MoveTarget(interimTarget, targetPosition);
                    MoveTarget(confirmedTarget, targetPosition);
                }

                // print("Distance to target: " + distanceToTarget);
                // print("Angle to target: " + angleToTarget);
            }
        }

        public void MoveTarget(GameObject gameObject, Vector3 position)
        {
            gameObject.transform.position = position;
        }

        private bool TargetInStopAndTurnZone()
        {
            if (!stopAndTurnOn) return false;

            robotToTargetVector = selectedTarget.transform.position - robot.transform.position;

            robotToTargetVector.y = 0;

            angleToTarget = Vector3.SignedAngle(robot.transform.forward, robotToTargetVector, robot.transform.up);

            distanceToTarget = Vector3.Distance(selectedTarget.transform.position, robot.transform.position);

            if (Mathf.Abs(angleToTarget) < targetCloseFrontAngle)
            {
                return false;
            }
            else if (distanceToTarget < veryCloseToTargetRad)
            {
                return true;
            }
            else if (Mathf.Abs(angleToTarget) < targetInFrontAngle)
            {
                return false;
            }
            else if (distanceToTarget < robotTurnRadius)
            {
                return true;
            }
            else if (Mathf.Abs(angleToTarget) > targetBehindAngle)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        void Update()
        {
            float distance = Vector3.Distance(interimTarget.transform.position, confirmedTarget.transform.position);

            if (distance > 0.5f) // todo remove hard-coding of min dist
            {
                if (TargetInStopAndTurnZone())
                {
                    robotToTargetVector = selectedTarget.transform.position - robot.transform.position;
                    angleToTarget = Vector3.SignedAngle(robot.transform.forward, robotToTargetVector, robot.transform.up);
                    robot.transform.Rotate(Vector3.up, Mathf.Sign(angleToTarget) * angularSpeed * Time.deltaTime);
                }
                else
                {
                    MoveTarget(confirmedTarget, interimTarget.transform.position);
                }
            }
        }
    }
}