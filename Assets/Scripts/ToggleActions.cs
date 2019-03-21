using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR.InteractionSystem;

public class ToggleActions : MonoBehaviour
{
    [Header("Direct Robot Turn")]
    public bool allowDirectTurn;
    TurnAgent[] turnAgents;


    // Start is called before the first frame update
    void Start()
    {
        turnAgents = GetComponentsInChildren<TurnAgent>();
        foreach (TurnAgent turnAgent in turnAgents)
        {
            turnAgent.enabled = allowDirectTurn;
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
