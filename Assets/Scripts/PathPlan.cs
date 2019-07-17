﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using System.Linq;

public class PathPlan : MonoBehaviour
{

    static float pi = Mathf.PI;

    public static Vector2 Rotate(Vector2 x, float angle)
    {
        // alpha: radians, counter-clockwise about orthogonal axis

        var c_theta = Mathf.Cos(angle);
        var s_theta = Mathf.Sin(angle);

        Vector2 x_rot = new Vector2(
            x.x * c_theta + x.y * s_theta,
            x.y * c_theta - x.x * s_theta);

        return x_rot;
    }

    public class BBPathInstructions
    {
        public List<float> Angles { get; set; }
        public List<float> Distances { get; set; }
        public List<bool> SatelliteTurns { get; set; }

        public BBPathInstructions(List<float> turn_angles,
            List<float> straight_distances,
            List<bool> satellite_turns)
        {
            Angles = turn_angles;
            Distances = straight_distances;
            SatelliteTurns = satellite_turns;
        }
    }


    public class TurningCircleInfo
    {
        public float A { get; set; }
        public float O { get; set; }
        public float D { get; set; }
        public Vector2 XC { get; set; }
        public Vector2 XP1 { get; set; }
        public Vector2 XP2 { get; set; }
        public bool ST { get; set; }
        public Vector2 XC1 { get; set; }
        public Vector2 XC2 { get; set; }

        public TurningCircleInfo(float alpha,
            float offset,
            float waypoint_distance,
            Vector2 turning_centre,
            Vector2 turn_start_pos,
            Vector2 turn_finish_pos,
            bool satellite_turn,
            Vector2 st_turn_centre_1,
            Vector2 st_turn_centre_2
            )

        {
            A = alpha;
            O = offset;
            D = waypoint_distance;
            XC = turning_centre;
            XP1 = turn_start_pos;
            XP2 = turn_finish_pos;
            ST = satellite_turn;
            XC1 = st_turn_centre_1;
            XC2 = st_turn_centre_2;
        }
    }

    public static TurningCircleInfo FindTurningCircle(Vector2 x1, Vector2 x2, Vector2 x3, float r_t, float r_max = 0)
    {

        var x12 = x2 - x1;
        var x13 = x3 - x1;
        var x23 = x3 - x2;

        var theta = Mathf.Atan2(x12.y, x12.x);
        var c_theta = Mathf.Cos(theta);
        var s_theta = Mathf.Sin(theta);

        var x2_1 = Rotate(x12, theta);

        var x3_1 = Rotate(x13, theta);

        var turn_lr = Mathf.Sign(x3_1.y);

        var alpha = Mathf.Atan2(x3_1.y, (x3_1.x - x2_1.x));

        var delta_x = r_t * Mathf.Tan(alpha / 2);
        var delta_x_abs = Mathf.Abs(delta_x);

        var xc_1 = new Vector2(
            x2_1.x - delta_x_abs,
            turn_lr * r_t);

        var xc = Rotate(xc_1, -theta) + x1;

        var x2c = xc - x2;

        var d2c = x2c.magnitude;
        var d12 = x12.magnitude;
        var d23 = x23.magnitude;

        bool satellite_turn;
        float offset;

        var xc1 = Vector2.zero;
        var xc2 = Vector2.zero;

        if (d23 < delta_x_abs || d12 < delta_x_abs || (r_max > 0 && d2c > (r_max + r_t)))
        {
            satellite_turn = true;
            offset = Mathf.Sqrt(3) * r_t;

            xc = new Vector2(x2.x, x2.y);
            alpha = turn_lr * (Mathf.Abs(alpha) + 2 * pi / 3);

        }
        else
        {
            satellite_turn = false;
            offset = delta_x_abs;
        }

        var xp1_1 = x2_1 - new Vector2(offset, 0);
        var xp1 = Rotate(xp1_1, -theta) + x1;

        var xp2 = x2 + (offset / d23) * x23;

        if (satellite_turn)
        {
            var x12_3d = new Vector3(x12.x, x12.y, 0f) / d12;
            var xp1_3d = new Vector3(xp1.x, xp1.y, 0f);
            var xc1_3d = xp1_3d + r_t * Vector3.Cross(x12_3d, Mathf.Sign(x3_1.y)*Vector3.forward);
            xc1 = new Vector2(xc1_3d.x, xc1_3d.y);

            var x23_3d = new Vector3(x23.x, x23.y, 0f) / d23;
            var xp2_3d = new Vector3(xp2.x, xp2.y, 0f);
            var xc2_3d = xp2_3d + r_t * Vector3.Cross(x23_3d, Mathf.Sign(x3_1.y) * Vector3.forward);
            xc2 = new Vector2(xc2_3d.x, xc2_3d.y);

            //var x23_3d = V2toV3(x23)/d23;
            //var xc2_3d = V2toV3(xp2) + r_t * Vector3.Cross(x23_3d, Vector3.down);
            //xc2 = V3toV2(xc2_3d);
        }


        TurningCircleInfo turningCircleInfo = new TurningCircleInfo(alpha, offset, d23, xc, xp1, xp2, satellite_turn, xc1, xc2);
        return turningCircleInfo;
    }

    public static TurningCircleInfo FindTurningCircle(Vector2 x1, Vector2 x2, float heading, float r_t)
    {
        var x12 = x2 - x1;
        var x2_1 = Rotate(x12, heading);

        int turn_lr;
        if (Mathf.Sign(x2_1.y) >= 0) { turn_lr = 1; }
        else { turn_lr = -1; }

        var xc_1 = new Vector2(0, turn_lr * r_t);
        var xc2_1 = x2_1 - xc_1;

        var dcg = xc2_1.magnitude;

        var dpg = Mathf.Sqrt(Mathf.Pow(dcg, 2) - Mathf.Pow(r_t, 2));

        var beta = Mathf.Acos(r_t / dcg);
        var gamma = Mathf.Atan2(xc2_1.y, xc2_1.x);

        if (Mathf.Sign(gamma) != turn_lr)
        {
            gamma += turn_lr * 2 *  pi;
        }

        var alpha = (gamma + turn_lr * (pi / 2 - beta));
        alpha = alpha % (2 * pi);

        var xc = Rotate(xc_1, -heading) + x1;

        var xp_1 = xc_1 + turn_lr * r_t * new Vector2(Mathf.Sin(alpha), -Mathf.Cos(alpha));

        var xp = Rotate(xp_1, -heading) + x1;

        TurningCircleInfo turningCircleInfo = new TurningCircleInfo(alpha, 0f, dpg, xc, x1, xp, false, Vector2.zero, Vector2.zero);
        return turningCircleInfo;
    }

    public static List<TurningCircleInfo> BBInfo(Vector2 pos, float heading, List<Vector2> waypoints, float r_t, float r_max = 0)
    {
        List<TurningCircleInfo> turningCircleInfos = new List<TurningCircleInfo>
        {
            FindTurningCircle(pos, waypoints[0], heading, r_t)
        };

        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            if (i == 0)
            {
                turningCircleInfos.Add(FindTurningCircle(turningCircleInfos[0].XP1, waypoints[i], waypoints[i + 1], r_t, r_max));
            }
            else
            {
                turningCircleInfos.Add(FindTurningCircle(waypoints[i - 1], waypoints[i], waypoints[i + 1], r_t, r_max));
            }
        }

        return turningCircleInfos;

    }

    public static BBPathInstructions BBInstructions(List<TurningCircleInfo> pathInfo)
    {
        List<float> turn_angles = new List<float>();
        List<float> straight_distances = new List<float>();
        List<bool> satellite_turns = new List<bool>();

        List<float> waypoint_distances = new List<float>();
        List<float> offsets = new List<float>();

        for (int i = 0; i < pathInfo.Count; i++)
        {
            if (i == 0)
            {
                turn_angles.Add(pathInfo[i].A);
                waypoint_distances.Add(pathInfo[i].D);
                offsets.Add(0f);
                satellite_turns.Add(false);
            }
            else
            {
                turn_angles.Add(pathInfo[i].A);
                offsets.Add(pathInfo[i].O);
                waypoint_distances.Add(pathInfo[i].D);
                straight_distances.Add(waypoint_distances[i - 1] - offsets[i - 1] - offsets[i]);
                satellite_turns.Add(pathInfo[i].ST);
            }
        }

        straight_distances.Add(waypoint_distances[waypoint_distances.Count - 1] - offsets[offsets.Count - 1]);

        BBPathInstructions path_instructions = new BBPathInstructions(turn_angles, straight_distances, satellite_turns);

        return path_instructions;

    }

    public static List<Vector2> UnpackPathInstructions(BBPathInstructions pathInstructions)
    {
        List<Vector2> instructions = new List<Vector2>();

        for (int i = 0; i < pathInstructions.Angles.Count; i++)
        {
            if (pathInstructions.SatelliteTurns[i])
            {
                instructions.Add(new Vector2(1, Mathf.Sign(pathInstructions.Angles[i]) * pi / 3));
                instructions.Add(new Vector2(1, -pathInstructions.Angles[i]));
                instructions.Add(new Vector2(1, Mathf.Sign(pathInstructions.Angles[i]) * pi / 3));
            }
            else
            {
                if (Mathf.Abs(pathInstructions.Angles[i]) > 0.00001)
                {
                instructions.Add(new Vector2(1, -pathInstructions.Angles[i]));
                }
            }
            instructions.Add(new Vector2(0, pathInstructions.Distances[i]));
        }

        return instructions;
    }

    public static void PrintBBPathInstructions(BBPathInstructions pathInstructions)
    {
        for (int i = 0; i < pathInstructions.Angles.Count; i++)
        {
            if (pathInstructions.SatelliteTurns[i])
            {
                print("Turn angle: " + (Mathf.Sign(pathInstructions.Angles[i]) * pi / 3));
                print("Turn angle: " + pathInstructions.Angles[i]);
                print("Turn angle: " + (Mathf.Sign(pathInstructions.Angles[i]) * pi / 3));
            }
            else
            {
                print("Turn angle: " + pathInstructions.Angles[i]);
            }
            print("Straight Distance: " + pathInstructions.Distances[i]);
        }
    }

    public static List<Vector3> BBPathPoints(float heading, List<TurningCircleInfo> pathInfo, Vector2 finalPoint, float r_t,  float height = 0)
    {
        List<Vector3> pathPoints = new List<Vector3>();

        float headingStart = heading;
        float headingEnd = heading;

        for (int i = 0; i < pathInfo.Count; i++)
        {
            Vector3 pathPoint1 = V2toV3(pathInfo[i].XP1, height);
            pathPoints.Add(pathPoint1);

            headingEnd += pathInfo[i].A;
            List<Vector3> arcPoints;
            if (pathInfo[i].ST)
            {
                float headingStart2 = headingStart - Mathf.Sign(pathInfo[i].A) * pi / 3;
                float headingEnd2 = headingEnd - Mathf.Sign(pathInfo[i].A) * pi / 3;
                headingEnd -= 2 * pi / 3;

                arcPoints = FindPointsOnArc(pathInfo[i].XC1, headingStart, headingStart2, r_t, height);
                arcPoints.AddRange(FindPointsOnArc(pathInfo[i].XC, headingStart2, headingEnd2, r_t, height));
                arcPoints.AddRange(FindPointsOnArc(pathInfo[i].XC2, headingEnd2, headingEnd, r_t, height));
            }
            else
            {
                arcPoints = FindPointsOnArc(pathInfo[i].XC, headingStart, headingEnd, r_t, height);
            }
            headingStart = headingEnd;

            pathPoints.AddRange(arcPoints);

            Vector3 pathPoint2 = V2toV3(pathInfo[i].XP2, height);
            pathPoints.Add(pathPoint2);

            List<Vector3> straightPoints;
            if (i != pathInfo.Count - 1)
            {
                straightPoints = LinSpaceV2(pathInfo[i].XP2, pathInfo[i + 1].XP1);
            }
            else
            {
                straightPoints = LinSpaceV2(pathInfo[i].XP2, finalPoint);
            }
            pathPoints.AddRange(straightPoints);
        }

        return pathPoints;
    }

    public static List<Vector3> FindPointsOnArc(Vector2 circleCentre, float headingStart, float headingEnd, float r_t, float height = 0, int num = 20)
    {
        List<Vector3> arcPoints = new List<Vector3>();

        float turnDirection = Mathf.Sign(headingEnd - headingStart);

        float alpha = headingStart;

            for (int j = 1; j < num; j++)
            {
                alpha = headingStart + j * (headingEnd - headingStart) / (num);

                Vector3 turningCirclePoint = new Vector3(
                    circleCentre.x + turnDirection * r_t * Mathf.Sin(alpha),
                    height,
                    circleCentre.y - turnDirection * r_t * Mathf.Cos(alpha));

                arcPoints.Add(turningCirclePoint);
            }

        return arcPoints;
    }

    public static bool CheckPathPoints(List<Vector3> linearSamplePoints)
    {
        bool validPath = true;
        NavMeshHit hit;
        foreach (var samplePoint in linearSamplePoints)
        {
            if (!NavMesh.SamplePosition(samplePoint, out hit, 0.1f, NavMesh.AllAreas))
            {
                validPath = false;
                return validPath;
            }
        }

        return validPath;
    }

    public static Vector2 V3toV2(Vector3 point3D)
    {
        Vector2 point2D = new Vector2(
            point3D.x,
            point3D.z);

        return point2D;
    }

    public static Vector3 V2toV3(Vector2 point2D, float height = 0)
    {
        Vector3 point3D = new Vector3(
            point2D.x,
            height,
            point2D.y);

        return point3D;
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

    public static List<Vector3> LinSpaceV2(Vector2 start, Vector2 end, float chordLength = 0.5f, float height = 0.1f)
    {
        int steps = (int)(start.magnitude / chordLength + 1);
        var xCoords = LinSpace(start.x, end.x, steps, false);
        var yCoords = LinSpace(start.y, end.y, steps, false);
        var xCoordsL = xCoords.ToList();
        var yCoordsL = yCoords.ToList();

        List<Vector3> linearSamplePoints = new List<Vector3>();

        for (int i = 0; i < xCoordsL.Count; i++)
        {
            linearSamplePoints.Add(new Vector3((float)xCoordsL[i], height, (float)yCoordsL[i]));
        }

        return linearSamplePoints;
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
        public int T { get; set; }

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


    public class PathParams
    {
        public float R_turn { get; set; } // agent turning radius
        public float R_max { get; set; } // waypoint accuracy radius
        public float L { get; set; } // chord length
        public bool ShowPoints { get; set; }
        public Vector2[] Delta_XY { get; set; } = new Vector2[3]; // path segment deltas

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
        }
    }

    public static IEnumerator HybridAStarPath(Vector3 startPos, float startHead, Vector3 goalPos1, PathParams pathParams)
    {

        List<RSpoint> openNodes = new List<RSpoint>();
        List<RSpoint> closedNodes = new List<RSpoint>();

        //float robotHeadingRad = Mathf.Deg2Rad * robot.transform.eulerAngles.y;

        int nodeCounter = 0;

        print("here");

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
                var pathPoints = BuildPath(closedNodes, chosenNode.N);
                var pathPointPositions = FindPathPositions(pathPoints);
                yield break;
            }
            else
            {
                nodeCounter = UpdateNeighbours(chosenNode, goalPos1, nodeCounter, pathParams, ref openNodes, ref closedNodes);
            }
            yield return null;
        }
        print("done");
        yield break;
    }

    public static int UpdateNeighbours(RSpoint Node, Vector3 goalPos, int nodeCounter, PathParams pathParams, ref List<RSpoint> openNodes, ref List<RSpoint> closedNodes)
    {
        foreach (Vector2 delta in pathParams.Delta_XY)
        {
            RSpoint newPoint = new RSpoint(Node);
            newPoint.N_p = Node.N;
            newPoint.N = nodeCounter;
            newPoint.X += Vector3.forward * (delta.x * Mathf.Cos(Node.Theta) + delta.y * Mathf.Sin(Node.Theta));
            newPoint.X += Vector3.right * (delta.x * Mathf.Sin(Node.Theta) - delta.y * Mathf.Cos(Node.Theta));

            if (pathParams.ShowPoints)
            {
                GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                cube.transform.localScale = new Vector3(0.03f, 0.03f, 0.03f);
                cube.transform.position = newPoint.X;
                cube.GetComponent<Collider>().enabled = false;
            }

            if (delta.y > 0.001)
            {
                newPoint.Theta = Node.Theta - pathParams.L / pathParams.R_turn;
            }
            else if (delta.y < -0.001)
            {
                newPoint.Theta = Node.Theta + pathParams.L / pathParams.R_turn;
            }
            newPoint.G = Node.G + pathParams.L;
            newPoint.H = CostToGoal(newPoint.X, goalPos, pathParams.R_max);
            float newTotalCost = newPoint.G + 1.05f * newPoint.H;
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

        return pathPoints;
    }


    public static float CostToGoal(Vector3 pos, Vector3 goalPos, float r_max)
    {
        float cost = Vector3.Distance(pos, goalPos) - r_max;
        return cost;
    }
}

