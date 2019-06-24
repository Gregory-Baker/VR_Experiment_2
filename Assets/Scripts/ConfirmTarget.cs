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
        public float angularSpeed = 12f;
        //public float veryCloseToTargetRad = 4f;
        //public float targetCloseFrontAngle = 20f;

        Vector3 targetPosition;

        public List<Vector3> waypoints = new List<Vector3>();
        float distanceToNextWaypoint;
        public float waypointAccuracyRad = 1f;
        public float robotTurnRadius = 2f;
        public float chordLength = 0.4f;
        List<RSpoint> openNodes = new List<RSpoint>();
        List<RSpoint> closedNodes = new List<RSpoint>();

        Vector3 robotToTargetVector;
        float angleToTarget;
        float distanceToTarget;
        Vector2[] deltaXY = new Vector2[3];

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

            public RSpoint(int nodeIndex,
                Vector3 position, 
                float heading, 
                float costFromRoot,
                float estimatedCostToGoal,
                float estimatedTotalCost,
                int parentIndex,
                int segmentIndex)
            {
                N = nodeIndex;
                X = position;
                Theta = heading;
                G = costFromRoot;
                H = estimatedCostToGoal;
                F = estimatedTotalCost;
                N_p = parentIndex;
                S = segmentIndex;
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
            }
        }

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

            deltaXY[0].x = robotTurnRadius * Mathf.Sin(chordLength / robotTurnRadius);
            deltaXY[0].y = robotTurnRadius * (1 - Mathf.Cos(chordLength / robotTurnRadius));


            deltaXY[1].x = chordLength;

            deltaXY[2].x = robotTurnRadius * Mathf.Sin(chordLength / robotTurnRadius);
            deltaXY[2].y = - robotTurnRadius * (1 - Mathf.Cos(chordLength / robotTurnRadius));

            Vector2 testTarget = new Vector2(-12.5f, 5.0f);

            //var pathInfo = BakerPath(testTarget);


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

                if (waypoints.Count >= 1)
                {
                    //float robotHeadingRad = Mathf.Deg2Rad * robot.transform.eulerAngles.y;
                    PlanPath(robot.transform.position, robot.transform.eulerAngles.y, waypoints[waypoints.Count-1]);
                    //MoveToWaypoint(waypoints[0]);
                }
            }
        }

        public void PlanPath(Vector3 startPos, float startHead, Vector3 targetPos)
        {
            var pathParams = BakerPath(startPos, startHead, targetPos);

            if (!pathParams.Item3)
            {
                StartCoroutine(HybridAStarPath(startPos, startHead, targetPos));
            }
        }

        private Tuple<float, float, bool> BakerPath(Vector3 startPos, float startHead, Vector3 targetPos)
        {
            float turn_angle = 0;
            float straight_distance = 0;
            Vector2 x_p = Vector2.zero;
            bool validPath = true;

            float r_t = robotTurnRadius;

            Vector2 x_r = new Vector2(startPos.x, startPos.z);
            Vector2 target = new Vector2(targetPos.x, targetPos.z);

            float theta = -Mathf.Deg2Rad * startHead + pi / 2;

            float cos_theta = Mathf.Cos(theta);
            float sin_theta = Mathf.Sin(theta);

            Vector2 x_rg = target - x_r;
            float d_rg = x_rg.magnitude;

            Vector2 x_rg_r = new Vector2(
                x_rg.x * cos_theta + x_rg.y * sin_theta,
                x_rg.y * cos_theta + x_rg.x * sin_theta);

            int turn = 0;

            Vector2 x_c = new Vector2();

            if (x_rg_r.y > 0)
            {
                print("Left hand turn");
                turn = 1;

                x_c.x = x_r.x - r_t * sin_theta;
                x_c.y = x_r.y + r_t*cos_theta;
            }
            else if (x_rg_r.y < 0)
            {
                print("Right hand turn");
                turn = -1;

                x_c.x = x_r.x + r_t * sin_theta;
                x_c.y = x_r.y - r_t * cos_theta;
            }
            else if (x_rg_r.x < 0)
            {
                print("It's behind you!");
                turn = 1;

                x_c.x = x_r.x - r_t * sin_theta;
                x_c.y = x_r.y + r_t * cos_theta;
            }
            else
            {
                print("Dead ahead");
                straight_distance = d_rg;
                validPath = false;
                var result = Tuple.Create(turn_angle, straight_distance, validPath);
                return result;
            }

            Vector2 x_cg = target - x_c;

            float d_cg = x_cg.magnitude;
            
            if (d_cg < r_t)
            {
                print("Target unreachable - inside turning circle");
            }
            else
            {
                float beta = Mathf.Acos(r_t / d_cg);
                float gamma = Mathf.Atan2(x_cg.y, x_cg.x);
                if (turn > 0)
                {
                    if (gamma < -pi/2)
                    {
                        gamma += 2 * pi;
                    }
                    turn_angle = gamma - beta - theta + pi / 2;
                    x_p.x = x_c.x + r_t * Mathf.Sin(theta + turn_angle);
                    x_p.y = x_c.y - r_t * Mathf.Cos(theta + turn_angle);
                }
                else
                {
                    if (gamma > pi / 2)
                    {
                        gamma -= 2 * pi;
                    }
                    turn_angle = beta + gamma - theta - pi / 2;
                    x_p.x = x_c.x - r_t * Mathf.Sin(theta + turn_angle);
                    x_p.y = x_c.y + r_t * Mathf.Cos(theta + turn_angle);
                }
                straight_distance = Mathf.Sqrt(Mathf.Pow(r_t, 2) + Mathf.Pow(d_cg, 2));
                print("Turn: " + turn_angle + " radians");
                print("Move forward: " + straight_distance + " metres");
            }
            int steps = (int)(straight_distance/chordLength + 1);
            var xCoords = LinSpace(x_p.x, target.x, steps);
            var yCoords = LinSpace(x_p.y, target.y, steps);
            Vector3[] linearSamplePoints = new Vector3[steps];
            int i = 0;
            foreach (var xCoord in xCoords)
            {
                linearSamplePoints[i].x = (float)xCoord;
                i++;
            }
            i = 0;
            foreach (var yCoord in yCoords)
            {
                linearSamplePoints[i].z = (float)yCoord;
                i++;
            }
            NavMeshHit hit;
            foreach (var samplePoint in linearSamplePoints)
            {
                if (!NavMesh.SamplePosition(samplePoint, out hit, 0.1f, NavMesh.AllAreas))
                {
                    validPath = false;
                }
            }
            print("Valid Path: " + validPath);
            if (validPath)
            {
                OnDrawGizmosSelected(line, linearSamplePoints);
            }
            return Tuple.Create(turn_angle, straight_distance, validPath);
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
                distanceToNextWaypoint = Vector3.Distance(waypoints[0], robot.transform.position);

                if (distanceToNextWaypoint < waypointAccuracyRad)
                {
                    if (waypoints.Count > 1)
                    {
                        MoveToWaypoint(waypoints[1]);
                        waypoints.RemoveAt(0);
                        float robotHeadingRad = Mathf.Deg2Rad * robot.transform.eulerAngles.y;
                        HybridAStarPath(robot.transform.position, robotHeadingRad, waypoints[0]);
                    }
                }
            }
        }

        IEnumerator HybridAStarPath(Vector3 startPos, float startHead, Vector3 goalPos1)
        {
            bool validPath = false;

            float robotHeadingRad = Mathf.Deg2Rad * robot.transform.eulerAngles.y;

            int nodeCounter = 0;

            List<Vector2> plannedPath = new List<Vector2>();

            float costToGoal = CostToGoal(startPos, goalPos1);
            RSpoint startPoint = new RSpoint(nodeCounter, startPos, startHead, 0, costToGoal, costToGoal, 0, 1);

            openNodes.Add(startPoint);
            nodeCounter++;
            while (openNodes.Count > 0)
            {
                var chosenNode = openNodes.Find(x => x.F == openNodes.Min(y => y.F));

                openNodes.Remove(chosenNode);
                closedNodes.Add(chosenNode);

                if (chosenNode.H < waypointAccuracyRad)
                {
                    validPath = true;
                    yield break;
                }
                else
                {
                    nodeCounter = UpdateNeighbours(chosenNode, goalPos1, nodeCounter);
                }
                yield return null;
            }
            yield break;
        }

        private int UpdateNeighbours(RSpoint Node, Vector3 goalPos, int nodeCounter)
        {
            foreach (Vector2 delta in deltaXY)
            {
                RSpoint newPoint = new RSpoint(Node);
                newPoint.N_p = Node.N;
                newPoint.N = nodeCounter;
                newPoint.X += Vector3.forward * (delta.x * Mathf.Cos(Node.Theta) + delta.y * Mathf.Sin(Node.Theta));
                newPoint.X += Vector3.right * (delta.x * Mathf.Sin(Node.Theta) - delta.y * Mathf.Cos(Node.Theta));
                GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                cube.transform.localScale = new Vector3(0.03f, 0.03f, 0.03f);
                cube.transform.position = newPoint.X;
                cube.GetComponent<Collider>().enabled = false;
                if (delta.y > 0.001)
                {
                    newPoint.Theta = Node.Theta - chordLength / robotTurnRadius;
                }
                else if (delta.y < -0.001)
                {
                    newPoint.Theta = Node.Theta + chordLength / robotTurnRadius;
                }
                newPoint.G = Node.G + chordLength;
                newPoint.H = CostToGoal(newPoint.X, goalPos);
                float newTotalCost = newPoint.G + 1.05f*newPoint.H;
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
            return nodeCounter;
        }

        private float CostToGoal(Vector3 pos, Vector3 goalPos)
        {
            float cost = Vector3.Distance(pos, goalPos) - waypointAccuracyRad;
            return cost;
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

        public static IEnumerable<double> Arange(double start, int count)
        {
            return Enumerable.Range((int)start, count).Select(v => (double)v);
        }

        public static IEnumerable<double> LinSpace(double start, double stop, int num, bool endpoint = true)
        {
            var result = new List<double>();
            if (num <= 0)
            {
                return result;
            }

            if (endpoint)
            {
                if (num == 1)
                {
                    return new List<double>() { start };
                }

                var step = (stop - start) / ((double)num - 1.0d);
                result = Arange(0, num).Select(v => (v * step) + start).ToList();
            }
            else
            {
                var step = (stop - start) / (double)num;
                result = Arange(0, num).Select(v => (v * step) + start).ToList();
            }

            return result;
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
    }
}