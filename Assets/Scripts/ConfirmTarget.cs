using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR.InteractionSystem;
using UnityEngine.AI;
using System.Runtime.InteropServices;
using System.Linq;

namespace Valve.VR.InteractionSystem
{
    //---------------------------
    public class ConfirmTarget : MonoBehaviour
    {
        float pi = Mathf.PI;

        public SteamVR_Action_Boolean confirmTargetAction;
        public Hand hand;

        public GameObject confirmedTarget;
        public GameObject interimTarget;
        public GameObject selectedTarget;
        public GameObject robot;
        public NavMeshSurface navmesh;
        public LineRenderer line;

        float communicationDelay;

        float verticalOffset = 0.01f; // used to raise confirmed target above selected target

        [Header("Stop and Turn")]
        public bool stopAndTurnOn = true;
        public float robotRad;
        //public float targetInFrontAngle = 35f;
        //public float targetBehindAngle = 100f;
        public float linearSpeed = 0.5f;
        float angularSpeed;
        //public float veryCloseToTargetRad = 4f;
        //public float targetCloseFrontAngle = 20f;

        Vector3 targetPosition;

        public List<Vector2> waypoints = new List<Vector2>();
        //float distanceToNextWaypoint;
        public float waypointAccuracyRad = 1f;
        public float robotTurnRadius = 2f;
        public float chordLength = 0.4f;

        Vector3 robotToTargetVector;
        float angleToTarget;
        float distanceToTarget;
        Vector2[] deltaXY = new Vector2[3];

        bool newPath = false;
        bool newSegment = false;

        bool bbPathPlanning = false;
        bool hybridPathPlanning = true;

        public List<GameObject> cubes = new List<GameObject>();

        Coroutine lastRoutine = null;

        private void OnEnable()
        {
            if (hand == null)
                hand = this.GetComponent<Hand>();

            if (confirmTargetAction == null)
            {
                Debug.LogError("<b>[SteamVR Interaction]</b> No action assigned");
                return;
            }

            angularSpeed = (180 * linearSpeed) / (Mathf.PI * robotTurnRadius);

            confirmTargetAction.AddOnChangeListener(OnConfirmActionChange, hand.handType);

            communicationDelay = robot.GetComponent<Status>().communicationDelay;


            // Used in Hybrid a* path planning
            deltaXY[0].x = robotTurnRadius * Mathf.Sin(chordLength / robotTurnRadius);
            deltaXY[0].y = robotTurnRadius * (1 - Mathf.Cos(chordLength / robotTurnRadius));
            deltaXY[1].x = chordLength;
            deltaXY[2].x = robotTurnRadius * Mathf.Sin(chordLength / robotTurnRadius);
            deltaXY[2].y = -robotTurnRadius * (1 - Mathf.Cos(chordLength / robotTurnRadius));
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
                //MoveTarget(interimTarget, robot.transform.position);

                targetPosition = selectedTarget.transform.position;
                targetPosition.y += verticalOffset;

                CreateWaypointDisk(targetPosition);

                waypoints.Add(PathPlan.V3toV2(targetPosition));

                if (waypoints.Count >= 1)
                {
                    if (bbPathPlanning)
                    {
                        Vector2 x1 = PathPlan.V3toV2(robot.transform.position);
                        float heading = -Mathf.Deg2Rad * robot.transform.eulerAngles.y + pi / 2;

                        var pathInfo = PathPlan.BBInfo(x1, heading, waypoints, robotTurnRadius);

                        var pathPoints = PathPlan.BBPathPoints(heading, pathInfo, waypoints.Last(), robotTurnRadius, 0.1f);
                        DeletePathPoints();
                        ShowPathPoints(pathPoints);

                        if (PathPlan.CheckPathPoints(pathPoints))
                        {
                            var pathInstructions = PathPlan.BBInstructions(pathInfo);
                            print("new instructions");
                            PathPlan.PrintBBPathInstructions(pathInstructions);
                            var unpackedInstructions = PathPlan.UnpackPathInstructions(pathInstructions);
                            OnDrawGizmosSelected(line, pathPoints);
                            newPath = true;
                            StartCoroutine(MoveAlongBBPath(unpackedInstructions));
                        }
                        else { print("path not feasible"); }
                    }
                    else if (hybridPathPlanning)
                    {
                        float robotHeadingRad = Mathf.Deg2Rad * robot.transform.eulerAngles.y;
                        PathPlan.PathParams pathParams = new PathPlan.PathParams(robotTurnRadius, waypointAccuracyRad, chordLength, true);
                        StartCoroutine(PathPlan.HybridAStarPath(robot.transform.position, robotHeadingRad, targetPosition, pathParams));
                    }
                }
            }
        }

        public void ShowPathPoints(List<Vector3> pathPoints)
        {
            foreach (Vector3 point in pathPoints)
            {
                GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                cube.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                cube.transform.position = point;
                cube.GetComponent<Collider>().enabled = false;
                cubes.Add(cube);
            }
        }

        public void DeletePathPoints()
        {
            foreach (GameObject cube in cubes)
            {
                Destroy(cube);
            }
        }

        private void CreateWaypointDisk(Vector3 targetPosition)
        {
            GameObject disk = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            disk.transform.localScale = new Vector3(0.3f, 0.01f, 0.3f);
            disk.transform.position = targetPosition;
            disk.GetComponent<Collider>().enabled = false;
            disk.GetComponent<Renderer>().material.color = Color.green;
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
            return false;
        }

        void Update()
        {
            if (waypoints.Count > 0)
            {
                Vector3 waypoint_3d = PathPlan.V2toV3(waypoints[0]);
                float distanceToNextWaypoint = Vector3.Distance(waypoint_3d, robot.transform.position);

                if (distanceToNextWaypoint < waypointAccuracyRad)
                {
                    waypoints.RemoveAt(0);
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

        IEnumerator MoveAlongBBPathCoroutine(float delayTime, PathPlan.BBPathInstructions instructions)
        {
            yield return new WaitForSeconds(delayTime);

            for (int i = 0; i < instructions.Angles.Count; i++)
            {
                float distanceToTravel;

                float distanceMoved = 0;
                if (instructions.SatelliteTurns[i])
                {
                    distanceToTravel = Mathf.Sign(instructions.Angles[i]) * robotTurnRadius * pi / 3;

                    while (Mathf.Abs(distanceMoved) < Mathf.Abs(distanceToTravel))
                    {
                        distanceMoved += MoveRobotOneStep(robot, linearSpeed, Mathf.Sign(instructions.Angles[i]) * angularSpeed);
                        yield return null;
                    }
                }

                distanceToTravel = instructions.Angles[i] * robotTurnRadius;
                distanceMoved = 0;

                while (Mathf.Abs(distanceMoved) < Mathf.Abs(distanceToTravel))
                {
                    distanceMoved += MoveRobotOneStep(robot, linearSpeed, -Mathf.Sign(instructions.Angles[i]) * angularSpeed);
                    yield return null;
                }

                if (instructions.SatelliteTurns[i])
                {
                    distanceToTravel = Mathf.Sign(instructions.Angles[i]) * robotTurnRadius * pi / 3;
                    distanceMoved = 0;

                    while (Mathf.Abs(distanceMoved) < Mathf.Abs(distanceToTravel))
                    {
                        distanceMoved += MoveRobotOneStep(robot, linearSpeed, Mathf.Sign(instructions.Angles[i]) * angularSpeed);
                        yield return null;
                    }
                }

                distanceToTravel = instructions.Distances[i];
                distanceMoved = 0;

                while (Mathf.Abs(distanceMoved) < Mathf.Abs(distanceToTravel))
                {
                    distanceMoved += MoveRobotOneStep(robot, linearSpeed, 0f);
                    yield return null;
                }
            }
        }

        IEnumerator MoveAlongBBPathSegment(GameObject robot, float linearVelocity, float angularVelocity, float distanceToTravel)
        {
            float distanceMoved = 0;
            while (Mathf.Abs(distanceMoved) < Mathf.Abs(distanceToTravel) && !newPath)
            {
                distanceMoved += MoveRobotOneStep(robot, linearVelocity, angularVelocity);
                yield return null;
            }
        }

        IEnumerator MoveAlongBBPath(List<Vector2> instructions, float delayTime = 0)
        {
            yield return 0;
            newPath = false;
            foreach (Vector2 instruction in instructions)
            {
                float distanceToTravel;
                float angularVelocity = 0;
                if (instruction.x == 1)
                {
                    distanceToTravel = instruction.y * robotTurnRadius;
                    angularVelocity = Mathf.Sign(instruction.y) * angularSpeed;
                }
                else
                {
                    distanceToTravel = instruction.y;
                }

                float distanceMoved = 0;
                while (Mathf.Abs(distanceMoved) < Mathf.Abs(distanceToTravel) && !newPath)
                {
                    distanceMoved += MoveRobotOneStep(robot, linearSpeed, angularVelocity);
                    yield return null;
                }
                if (newPath) { break; }
            }
        }

        public void MoveAlongBBPath(float delayTime, PathPlan.BBPathInstructions instructions)
        {
            newPath = false;
            for (int i = 0; i < instructions.Angles.Count; i++)
            {
                float distanceToTravel;
                if (instructions.SatelliteTurns[i])
                {
                    distanceToTravel = Mathf.Sign(instructions.Angles[i]) * robotTurnRadius * pi / 3;
                    StartCoroutine(MoveAlongBBPathSegment(robot, linearSpeed, Mathf.Sign(instructions.Angles[i]) * angularSpeed, distanceToTravel));
                }
                if (newPath) { print("here");  break; }

                distanceToTravel = instructions.Angles[i] * robotTurnRadius;
                StartCoroutine(MoveAlongBBPathSegment(robot, linearSpeed, -Mathf.Sign(instructions.Angles[i]) * angularSpeed, distanceToTravel));
                if (newPath) { print("herea"); break; }

                if (instructions.SatelliteTurns[i])
                {
                    distanceToTravel = Mathf.Sign(instructions.Angles[i]) * robotTurnRadius * pi / 3;
                    StartCoroutine(MoveAlongBBPathSegment(robot, linearSpeed, Mathf.Sign(instructions.Angles[i]) * angularSpeed, distanceToTravel));
                }
                if (newPath) { print("here"); break; }

                distanceToTravel = instructions.Distances[i];
                StartCoroutine(MoveAlongBBPathSegment(robot, Mathf.Sign(instructions.Distances[i]) * linearSpeed, 0f, distanceToTravel));

                if (newPath) { print("heres"); break; }
            }
        }

        public static float MoveRobotOneStep(GameObject robot, float linearVelocity, float angularVelocity)
        {
            float distanceMoved = linearVelocity * Time.deltaTime;

            robot.transform.Translate(Vector3.forward * distanceMoved);
            robot.transform.Rotate(Vector3.up * angularVelocity * Time.deltaTime);

            return distanceMoved;
        }

        void OnDrawGizmosSelected(LineRenderer line, Vector3[] path)
        {
            line.positionCount = path.Length;

            int i = 0;

            foreach (Vector3 node in path)
            {
                line.SetPosition(i, node);
                i++;
            }
        }

        void OnDrawGizmosSelected(LineRenderer line, List<Vector3> path)
        {
            line.positionCount = path.Count;

            int i = 0;

            foreach (Vector3 node in path)
            {
                line.SetPosition(i, node);
                i++;
            }

        }
    }
}