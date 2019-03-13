using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class SampleAgentScript : MonoBehaviour
{
    public Transform target;
    NavMeshAgent agent;
    LineRenderer line;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        line = this.GetComponent<LineRenderer>();
    }

    void Update()
    {
        agent.SetDestination(target.position);
        OnDrawGizmosSelected();
    }

    void OnDrawGizmosSelected()
    {

        if (agent == null || agent.path == null)
            return;

        if (line == null)
        {
            line.startWidth = 0.5f;
            line.endWidth = 0.5f;
            line.startColor = Color.yellow;
        }

        var path = agent.path;

        line.positionCount = path.corners.Length;

        for (int i = 0; i < path.corners.Length; i++)
        {
            line.SetPosition(i, path.corners[i]);
        }

    }
}
