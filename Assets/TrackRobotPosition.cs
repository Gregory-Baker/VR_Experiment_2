using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrackRobotPosition : MonoBehaviour
{
    public GameObject robot;

    public bool egocentric = true;

    public int frame = 0;

    float currentX;
    float currentZ;

    float previousX;
    float previousZ;

    Vector3 playerTranslation;

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
            currentX = robot.transform.position.x;
            currentZ = robot.transform.position.z;

            playerTranslation.x = currentX - previousX;
            playerTranslation.z = currentZ - previousZ;

            transform.Translate(playerTranslation);

            previousX = currentX;
            previousZ = currentZ;
        }
        frame++;
    }
}
