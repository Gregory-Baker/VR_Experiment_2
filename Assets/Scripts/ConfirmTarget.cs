using System;
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

        float communicationDelay;

        float verticalOffset = 0.01f; // used to raise confirmed target above selected target

        [Header("Stop and Turn")]
        public bool stopAndTurnOn = true;
        public float robotRad;
        public float targetInFrontAngle = 35f;
        public float targetBehindAngle = 100f;
        public float robotTurnRadius = 6f;
        public float angularSpeed = 12f;
        public float veryCloseToTargetRad = 4f;
        public float targetCloseFrontAngle = 20f;

        Vector3 targetPosition;

        public List<Vector3> waypoints;
        float distanceToNextWaypoint;
        public float waypointAccuracyRad = 2f;

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

            communicationDelay = robot.GetComponent<Status>().communicationDelay;
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
                MoveTarget(interimTarget, robot.transform.position);

                targetPosition = selectedTarget.transform.position;
                targetPosition.y += verticalOffset;

                waypoints.Add(targetPosition);

                if (waypoints.Count == 1)
                {
                    MoveToWaypoint(waypoints[0]);
                }
            }
        }

        private void MoveToWaypoint(Vector3 waypointPosition)
        {
            if (TargetInStopAndTurnZone())
            {
                TurnToTarget();
                MoveTarget(interimTarget, waypointPosition);
            }
            else
            {
                MoveTarget(interimTarget, waypointPosition);
                StartCoroutine(MoveTargetCoroutine(communicationDelay));
            }
        }

        public void MoveTarget(GameObject gameObject, Vector3 position)
        {
            gameObject.transform.position = position;
        }

        private void TurnToTarget()
        {
            StartCoroutine(TurnToTargetCoroutine(communicationDelay));
        }

        public void StopRobot()
        {
            float stoppingDistance = 0.3f;
            Vector3 robotStoppingVector = robot.transform.forward * stoppingDistance;
            MoveTarget(interimTarget, robot.transform.position);
            MoveTarget(confirmedTarget, robot.transform.position + robotStoppingVector);
        }

        private bool TargetInStopAndTurnZone()
        {
            //if (!stopAndTurnOn) return false;

            //robotToTargetVector = selectedTarget.transform.position - robot.transform.position;

            //robotToTargetVector.y = 0;

            //angleToTarget = Vector3.SignedAngle(robot.transform.forward, robotToTargetVector, robot.transform.up);

            //distanceToTarget = Vector3.Distance(selectedTarget.transform.position, robot.transform.position);

            //if (Mathf.Abs(angleToTarget) < targetCloseFrontAngle)
            //{
            //    return false;
            //}
            //else if (distanceToTarget < veryCloseToTargetRad)
            //{
            //    return true;
            //}
            //else if (Mathf.Abs(angleToTarget) < targetInFrontAngle)
            //{
            //    return false;
            //}
            //else if (distanceToTarget < robotTurnRadius)
            //{
            //    return true;
            //}
            //else if (Mathf.Abs(angleToTarget) > targetBehindAngle)
            //{
            //    return true;
            //}
            //else
            //{
            //    return false;
            //}

            return false;
        }

        void Update()
        {
            if (waypoints.Count > 0)
            {
                distanceToNextWaypoint = Vector3.Distance(waypoints[0], robot.transform.position);

                if (distanceToNextWaypoint < waypointAccuracyRad)
                {
                    if (waypoints.Count > 1)
                    {
                        MoveToWaypoint(waypoints[1]);
                        waypoints.RemoveAt(0);
                        //PlanPath()
                    }
                }
            }
        }

        IEnumerator MoveTargetCoroutine(float delayTime)
        {
            yield return new WaitForSeconds(delayTime);
            MoveTarget(confirmedTarget, targetPosition);
        }

        IEnumerator TurnToTargetCoroutine(float delayTime)
        {
            yield return new WaitForSeconds(delayTime);
            if (robot.GetComponent<Status>().isMoving)
            {
                StopRobot();
            }
            MoveTarget(interimTarget, targetPosition);

            while (TargetInStopAndTurnZone())
            {
                robotToTargetVector = selectedTarget.transform.position - robot.transform.position;
                angleToTarget = Vector3.SignedAngle(robot.transform.forward, robotToTargetVector, robot.transform.up);
                robot.transform.Rotate(Vector3.up, Mathf.Sign(angleToTarget) * angularSpeed * Time.deltaTime);
                yield return null;
            }
            MoveTarget(confirmedTarget, targetPosition);
        }

    }
}