using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Status : MonoBehaviour
{
    public float communicationDelay = 0f;

    Vector3 lastPos;
    Vector3 curPos;

    
    public bool isMoving = false;

    void Start()
    {
        lastPos = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        curPos = transform.position;
        if (curPos == lastPos)
        {
            isMoving = false;
        }
        else
        {
            isMoving = true;
        }
        lastPos = curPos;
    }
}
