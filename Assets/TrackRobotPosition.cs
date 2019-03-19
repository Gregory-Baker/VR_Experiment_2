using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrackRobotPosition : MonoBehaviour
{
    public GameObject robot;

    public bool egocentric = true;
    public bool trackRotation = false;

    public float maxTurnAnglePerSecond = 10f;
    public float maxHeadRotationAcc = 100f;

    int frame;

    float currentX;
    float currentZ;
    float previousX;
    float previousZ;

    Vector3 playerTranslation;

    float currentPlayerAngle;
    float currentRobotAngle;
    float previousHeadRotationAngle = 0;
    float angleDiff;
    public float headRotationAngle;
    public float headRotationVel;
    float previousHeadRotationVel = 0;
    public float headRotationAcc;

    Vector3 playerRotation;

    // Start is called before the first frame update
    void Start()
    {
        previousX = robot.transform.position.x;
        previousZ = robot.transform.position.z;
    }

    // Update is called once per frame
    void Update()
    {

        frame++;

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

        headRotationVel = (headRotationAngle - previousHeadRotationAngle) / Time.deltaTime;
        headRotationAcc = (headRotationVel - previousHeadRotationVel) / Time.deltaTime;

        //if (Mathf.Abs(headRotationAcc) > maxHeadRotationAcc)
        //{
        //    print("Original Angle: " + headRotationAngle);
        //    for (int i = 0; i < 5; i++)
        //    {
        //        if (Mathf.Abs(headRotationAcc) < maxHeadRotationAcc)
        //        {
        //            print("Final Angle: " + headRotationAngle);
        //            break;
        //        }
        //        else
        //        {
        //            headRotationAngle = headRotationAngle / 2;
        //            headRotationVel = (headRotationAngle - previousHeadRotationAngle) / Time.deltaTime;
        //            headRotationAcc = (headRotationVel - previousHeadRotationVel) / Time.deltaTime;
        //        }
        //    }
        //    print("Final Angle2: " + headRotationAngle);
        //}

        //if (Mathf.Abs(headRotationAcc) > maxHeadRotationAcc && frame % 3 == 0)
        //{
        //    headRotationVel = Mathf.Sign(angleDiff) * maxHeadRotationAcc * Time.deltaTime + previousHeadRotationVel;
        //    headRotationAngle = headRotationVel * Time.deltaTime + previousHeadRotationAngle;
        //    print("Acceleration: " + headRotationAcc);
        //}

        //while (Mathf.Abs(headRotationAcc) > maxHeadRotationAcc)
        //{
        //    headRotationAngle *= 0.5f;
        //    headRotationVel = (headRotationAngle - previousHeadRotationAngle) / Time.deltaTime;
        //    headRotationAcc = (headRotationVel - previousHeadRotationVel) / Time.deltaTime;
        //    print(headRotationAcc);
        //}

        transform.Rotate(transform.up, headRotationAngle);


        previousHeadRotationAngle = headRotationAngle;
        previousHeadRotationVel = headRotationVel;

    }
}
