using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SlowDownTurn : MonoBehaviour
{
    public float maxTurnAnglePerFrame = 1f;

    public float previousHeadAngle;
    public float currentHeadAngle;
    public float headAngleDiff;

    float angleDiff;

    void Start()
    {
        previousHeadAngle = 0;
    }

    // Update is called once per frame
    void Update()
    {
        currentHeadAngle = transform.eulerAngles.y;
        angleDiff = currentHeadAngle - previousHeadAngle;
        if (angleDiff > 180)
        {
            angleDiff -= 360;
        }
        else if (angleDiff < -180)
        {
            angleDiff += 360;
        }
        headAngleDiff = Mathf.Abs(angleDiff);

        if (headAngleDiff > maxTurnAnglePerFrame)
        {
            print(headAngleDiff);
        }

        previousHeadAngle = transform.eulerAngles.y;
    }
}
