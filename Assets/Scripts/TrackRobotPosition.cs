using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrackRobotPosition : MonoBehaviour
{
    public GameObject robot;

    public bool egocentric = true;
    public bool trackRotation = true;

    public float maxTurnAnglePerSecond = 10f;

    Vector3 robotPositionXZ;
    Vector3 heading;

    float currentX;
    float currentZ;
    float previousX;
    float previousZ;

    Vector3 playerTranslation;

    float currentPlayerAngle;
    float currentRobotAngle;
    float angleDiff;
    float headRotationAngle;


    // Start is called before the first frame update
    void Start()
    {
        previousX = robot.transform.position.x;
        previousZ = robot.transform.position.z;
    }

    // Update is called once per frame
    void Update()
    {
        if (egocentric)
        {
            TranslatePlayerWithRobot();

            if (trackRotation)
            {
                TurnPlayerWithRobot();
            }
        }
        else
        {
            TurnPlayerTowardsRobot();
        }
    }

    private void TranslatePlayerWithRobot()
    {
        currentX = robot.transform.position.x;
        currentZ = robot.transform.position.z;

        playerTranslation.x = currentX - previousX;
        playerTranslation.z = currentZ - previousZ;

        transform.Translate(playerTranslation, Space.World);

        previousX = currentX;
        previousZ = currentZ;
    }

    private void TurnPlayerWithRobot()
    {
        currentRobotAngle = robot.transform.eulerAngles.y;
        currentPlayerAngle = transform.eulerAngles.y;
        angleDiff = currentRobotAngle - currentPlayerAngle;

        if (angleDiff > 180)
        {
            angleDiff -= 360;
        }
        else if (angleDiff < -180)
        {
            angleDiff += 360;
        }

        if (Mathf.Abs(angleDiff) < maxTurnAnglePerSecond * Time.deltaTime)
        {
            headRotationAngle = angleDiff;
        }
        else
        {
            headRotationAngle = Mathf.Sign(angleDiff) * maxTurnAnglePerSecond * Time.deltaTime;
        }

        transform.Rotate(transform.up, headRotationAngle);
    }

    private void TurnPlayerTowardsRobot()
    {
        transform.LookAt(robot.transform);
    }
}
