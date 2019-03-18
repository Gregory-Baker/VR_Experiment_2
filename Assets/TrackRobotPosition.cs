using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrackRobotPosition : MonoBehaviour
{
    public GameObject robot;

    public Camera vrCamera;

    public bool egocentric = true;
    public bool trackRotation = false;

    public float maxTurnAnglePerSecond = 10f;

    float currentX;
    float currentZ;
    float previousX;
    float previousZ;

    Vector3 playerTranslation;

    float currentHeadAngle;
    float currentRobotAngle;
    float previousRobotAngle;
    float angleDiff;
    public float headRotationAngle;

    Vector3 playerRotation;

    // Start is called before the first frame update
    void Start()
    {
        previousX = robot.transform.position.x;
        previousZ = robot.transform.position.z;
        vrCamera = GetComponentInChildren<Camera>();
        previousRobotAngle = robot.transform.eulerAngles.y;
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
    }

    private void TranslatePlayerWithRobot()
    {
        currentX = robot.transform.position.x;
        currentZ = robot.transform.position.z;

        playerTranslation.x = currentX - previousX;
        playerTranslation.z = currentZ - previousZ;

        transform.Translate(playerTranslation);

        previousX = currentX;
        previousZ = currentZ;
    }

    private void TurnPlayerWithRobot()
    {
        currentRobotAngle = robot.transform.eulerAngles.y;
        // currentHeadAngle = vrCamera.transform.eulerAngles.y;
        // angleDiff = currentRobotAngle - currentHeadAngle;
        angleDiff = currentRobotAngle - previousRobotAngle;

        if (angleDiff > 180)
        {
            angleDiff -= 360;
        }
        else if (angleDiff < -180)
        {
            angleDiff += 360;
        }
        angleDiff = Mathf.Abs(angleDiff);

        headRotationAngle = Mathf.Min(angleDiff, 1000); // maxTurnAnglePerSecond * Time.deltaTime);

        playerRotation.y = headRotationAngle;

        transform.Rotate(transform.up, headRotationAngle);

        previousRobotAngle = currentRobotAngle;
    }
}
