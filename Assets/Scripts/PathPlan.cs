using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

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

    public class TurningCircleInfo
    {
        public float A { get; set; }
        public float O { get; set; }
        public float D { get; set; }
        public Vector2 XC { get; set; }
        public Vector2 XP1 { get; set; }
        public Vector2 XP2 { get; set; }
        public bool ST { get; set; }

        public TurningCircleInfo(float alpha,
            float offset,
            float waypoint_distance,
            Vector2 turning_circle_centre,
            Vector2 turn_start_pos,
            Vector2 turn_finish_pos,
            bool satellite_turn)

        {
            A = alpha;
            O = offset;
            D = waypoint_distance;
            XC = turning_circle_centre;
            XP1 = turn_start_pos;
            XP2 = turn_finish_pos;
            ST = satellite_turn;
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

        if (d23 < delta_x_abs || d12 < delta_x_abs || (r_max > 0 && d2c > (r_max + r_t)))
        {
            satellite_turn = true;
            offset = Mathf.Sqrt(3) * r_t;

            xc = new Vector2(x2.x, x2.y);
            alpha = turn_lr * (Mathf.Abs(alpha) + pi / 3);
        }
        else
        {
            satellite_turn = false;
            offset = delta_x_abs;
        }

        var xp1_1 = x2_1 - new Vector2(offset, 0);
        var xp1 = Rotate(xp1_1, -theta) + x1;

        var xp2 = x2 + (offset / d23) * x23;


        TurningCircleInfo turningCircleInfo = new TurningCircleInfo(alpha, offset, d23, xc, xp1, xp2, satellite_turn);
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

        var xc = Rotate(xc_1, -heading) + x1;

        var xp_1 = xc_1 + turn_lr * r_t * new Vector2(Mathf.Sin(alpha), -Mathf.Cos(alpha));

        var xp = Rotate(xp_1, -heading) + x1;

        TurningCircleInfo turningCircleInfo = new TurningCircleInfo(alpha, 0f, dpg, xc, xp, Vector2.zero, false);
        return turningCircleInfo;
    }

    public static Tuple<List<float>, List<float>, List<bool>> BBPath(Vector2 pos, float heading, List<Vector2> waypoints, float r_t, float r_max=0)
    {
        List<float> turn_angles = new List<float>();
        List<float> straight_distances = new List<float>();
        List<bool> satellite_turns = new List<bool>();

        List<float> waypoint_distances = new List<float>();
        List<float> offsets = new List<float>();
        float straight_distance;

        TurningCircleInfo turningCircleInfo = FindTurningCircle(pos, waypoints[0], heading, r_t);

        turn_angles.Add(turningCircleInfo.A);
        waypoint_distances.Add(turningCircleInfo.D);
        offsets.Add(0f);
        satellite_turns.Add(false);


        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            if (i == 0)
            {
                turningCircleInfo = FindTurningCircle(turningCircleInfo.XP1, waypoints[i], waypoints[i + 1], r_t, r_max);
            }
            else
            {
                turningCircleInfo = FindTurningCircle(waypoints[i - 1], waypoints[i], waypoints[i + 1], r_t, r_max);
            }

            turn_angles.Add(turningCircleInfo.A);
            offsets.Add(turningCircleInfo.O);
            waypoint_distances.Add(turningCircleInfo.D);
            straight_distance = waypoint_distances[i] - offsets[i] - offsets[i + 1];
            straight_distances.Add(straight_distance);
            satellite_turns.Add(turningCircleInfo.ST);
        }

        straight_distance = waypoint_distances[waypoints.Count - 1] - offsets[waypoints.Count - 1];
        straight_distances.Add(straight_distance);

        return Tuple.Create(turn_angles, straight_distances, satellite_turns);

    }
}
