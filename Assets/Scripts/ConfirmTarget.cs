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
        public float linearSpeed = 0.5f;
        float angularSpeed;

        Vector3 targetPosition;

        public List<Vector2> waypoints = new List<Vector2>();
        public float waypointAccuracyRad = 1f;
        public float robotTurnRadius = 2f;
        public float chordLength = 0.4f;

        Vector3 robotToTargetVector;
        float angleToTarget;
        float distanceToTarget;

        bool newPath = false;

        bool bbPathPlanning = false;
        bool hybridPathPlanning = true;
        bool showPathPoints = false;

        public List<GameObject> cubes = new List<GameObject>();
        public List<RSpoint> pathPoints = new List<RSpoint>();


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
                        PathParams pathParams = new PathParams(robotTurnRadius, waypointAccuracyRad, chordLength, showPathPoints);
                        StartCoroutine(HybridAStarPath(robot.transform.position, robotHeadingRad, targetPosition, pathParams));

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

        IEnumerator MoveAlongBBPath(List<Vector2> instructions, float delayTime = 0)
        {
            yield return new WaitForSeconds(delayTime);
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

        public static float MoveRobotOneStep(GameObject robot, float linearVelocity, float angularVelocity)
        {
            float distanceMoved = linearVelocity * Time.deltaTime;

            robot.transform.Translate(Vector3.forward * distanceMoved);
            robot.transform.Rotate(Vector3.up * angularVelocity * Time.deltaTime);

            return distanceMoved;
        }

        public void OnDrawGizmosSelected(LineRenderer line, Vector3[] path)
        {
            line.positionCount = path.Length;

            int i = 0;

            foreach (Vector3 node in path)
            {
                line.SetPosition(i, node);
                i++;
            }
        }

        public void OnDrawGizmosSelected(LineRenderer line, List<Vector3> path)
        {
            line.positionCount = path.Count;

            int i = 0;

            foreach (Vector3 node in path)
            {
                line.SetPosition(i, node);
                i++;
            }

        }

        public class RSpoint
        {
            public int N { get; set; }
            public Vector3 X { get; set; }
            public float Theta { get; set; }
            public float G { get; set; }
            public float H { get; set; }
            public float F { get; set; }
            public int N_p { get; set; }
            public int S { get; set; }
            public int Gear { get; set; }
            public bool GearChange { get; set; }
            public int T { get; set; }

            public RSpoint(int nodeIndex,
                Vector3 position,
                float heading,
                float costFromRoot,
                float estimatedCostToGoal,
                float estimatedTotalCost,
                int parentIndex,
                int segmentIndex,
                int gearValue = 1,
                bool gearChangeValue = false,
                int turnValue = 0)
            {
                N = nodeIndex;
                X = position;
                Theta = heading;
                G = costFromRoot;
                H = estimatedCostToGoal;
                F = estimatedTotalCost;
                N_p = parentIndex;
                S = segmentIndex;
                Gear = gearValue;
                GearChange = gearChangeValue;
                T = turnValue;

            }

            public RSpoint(RSpoint nodeToClone)
            {
                N = nodeToClone.N;
                X = nodeToClone.X;
                Theta = nodeToClone.Theta;
                G = nodeToClone.G;
                H = nodeToClone.H;
                F = nodeToClone.F;
                N_p = nodeToClone.N_p;
                S = nodeToClone.S;
                Gear = 1;
                GearChange = false;
                T = 0;
            }
        }


        public class PathParams
        {
            public float R_turn { get; set; } // agent turning radius
            public float R_max { get; set; } // waypoint accuracy radius
            public float L { get; set; } // chord length
            public bool ShowPoints { get; set; }
            public Vector2[] Delta_XY { get; set; } = new Vector2[6]; // path segment deltas

            public PathParams(float robotTurnRadius,
                float waypointAccuracyRad,
                float chordLength,
                bool showPoints = false)
            {
                R_turn = robotTurnRadius;
                R_max = waypointAccuracyRad;
                L = chordLength;
                ShowPoints = showPoints;

                Delta_XY[0].x = robotTurnRadius * Mathf.Sin(chordLength / robotTurnRadius);
                Delta_XY[0].y = robotTurnRadius * (1 - Mathf.Cos(chordLength / robotTurnRadius));
                Delta_XY[1].x = chordLength;
                Delta_XY[2].x = robotTurnRadius * Mathf.Sin(chordLength / robotTurnRadius);
                Delta_XY[2].y = -robotTurnRadius * (1 - Mathf.Cos(chordLength / robotTurnRadius));

                Delta_XY[3].x = -robotTurnRadius * Mathf.Sin(chordLength / robotTurnRadius);
                Delta_XY[3].y = -robotTurnRadius * (1 - Mathf.Cos(chordLength / robotTurnRadius));
                Delta_XY[4].x = -chordLength;
                Delta_XY[5].x = -robotTurnRadius * Mathf.Sin(chordLength / robotTurnRadius);
                Delta_XY[5].y = robotTurnRadius * (1 - Mathf.Cos(chordLength / robotTurnRadius));
            }
        }

        public IEnumerator HybridAStarPath(Vector3 startPos, float startHead, Vector3 goalPos1, PathParams pathParams)
        {
            List<RSpoint> openNodes = new List<RSpoint>();
            List<RSpoint> closedNodes = new List<RSpoint>();

            int nodeCounter = 0;

            List<Vector2> plannedPath = new List<Vector2>();

            float costToGoal = CostToGoal(startPos, goalPos1, pathParams.R_max);
            RSpoint startPoint = new RSpoint(nodeCounter, startPos, startHead, 0, costToGoal, costToGoal, 0, 1);

            openNodes.Add(startPoint);
            nodeCounter++;
            while (openNodes.Count > 0)
            {
                var chosenNode = openNodes.Find(x => x.F == openNodes.Min(y => y.F));

                openNodes.Remove(chosenNode);
                closedNodes.Add(chosenNode);

                if (chosenNode.H < pathParams.R_max)
                {
                    pathPoints = BuildPath(closedNodes, chosenNode.N);
                    var pathPointPositions = FindPathPositions(pathPoints);
                    OnDrawGizmosSelected(line, pathPointPositions);

                    StartCoroutine(MoveAlongAStarPath(pathPoints));
                    yield break;
                }
                else
                {
                    UpdateNeighbours(chosenNode, goalPos1, ref nodeCounter, pathParams, ref openNodes, ref closedNodes);
                }
                yield return null;
            }
            print("done");
            yield break;
        }

        public void UpdateNeighbours(RSpoint Node, Vector3 goalPos, ref int nodeCounter, PathParams pathParams, ref List<RSpoint> openNodes, ref List<RSpoint> closedNodes)
        {
            foreach (Vector2 delta in pathParams.Delta_XY)
            {
                RSpoint newPoint = new RSpoint(Node);
                newPoint.N_p = Node.N;
                newPoint.N = nodeCounter;
                newPoint.X += Vector3.forward * (delta.x * Mathf.Cos(Node.Theta) + delta.y * Mathf.Sin(Node.Theta));
                newPoint.X += Vector3.right * (delta.x * Mathf.Sin(Node.Theta) - delta.y * Mathf.Cos(Node.Theta));
                if (delta.x < 0)
                {
                    newPoint.Gear = -1;
                }
                if (Node.Gear != newPoint.Gear)
                {
                    newPoint.GearChange = true;
                }

                if (pathParams.ShowPoints)
                {
                    GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    cube.transform.localScale = new Vector3(0.03f, 0.03f, 0.03f);
                    cube.transform.position = newPoint.X;
                    cube.GetComponent<Collider>().enabled = false;
                }

                if (delta.y > 0.001)
                {
                    newPoint.T = -1;

                }
                else if (delta.y < -0.001)
                {
                    newPoint.T = 1;
                }

                newPoint.Theta = Node.Theta + newPoint.T * pathParams.L / pathParams.R_turn;
                newPoint.G = Node.G + pathParams.L;
                newPoint.H = CostToGoal(newPoint.X, goalPos, pathParams.R_max);

                float newTotalCost = newPoint.G + 1.05f * newPoint.H + 0.01f * Mathf.Abs(newPoint.T);
                newPoint.F = newTotalCost;
                NavMeshHit hit;
                if (NavMesh.SamplePosition(newPoint.X, out hit, 0.1f, NavMesh.AllAreas))
                {
                    openNodes.Add(newPoint);
                }
                else
                {
                    closedNodes.Add(newPoint);
                }
                nodeCounter++;
            }
        }


        public static List<Vector3> FindPathPositions(List<RSpoint> pathPoints)
        {
            List<Vector3> pathPointPositions = new List<Vector3>();

            foreach (RSpoint pathPoint in pathPoints)
            {
                pathPointPositions.Add(pathPoint.X);
            }

            return pathPointPositions;
        }

        public static List<RSpoint> BuildPath(List<RSpoint> closedNodes, int finalNodeIndex)
        {
            List<RSpoint> pathPoints = new List<RSpoint>();

            int currentNodeIndex = finalNodeIndex;

            while (currentNodeIndex != 0)
            {
                var currentNode = closedNodes.Find(x => x.N == currentNodeIndex);
                pathPoints.Add(currentNode);

                currentNodeIndex = closedNodes.Find(x => x.N == currentNodeIndex).N_p;
            }
            pathPoints.Add(closedNodes.Find(x => x.N == 0));

            pathPoints.Reverse();

            return pathPoints;
        }

        IEnumerator MoveAlongAStarPath(List<RSpoint> pathPoints, float delayTime = 0)
        {
            yield return new WaitForSeconds(delayTime);

            foreach (RSpoint point in pathPoints)
            {
                float distanceToTravel = chordLength;
                float distanceMoved = 0;
                if (point.N != 0)
                {
                    while (Mathf.Abs(distanceMoved) < Mathf.Abs(distanceToTravel))
                    {
                        distanceMoved += MoveRobotOneStep(robot, point.Gear * linearSpeed, point.T * angularSpeed);
                        yield return null;
                    }
                }
            }
        }

        public static float CostToGoal(Vector3 pos, Vector3 goalPos, float r_max)
        {
            float cost = Vector3.Distance(pos, goalPos) - r_max;
            return cost;
        }
    }
}