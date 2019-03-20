using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR.InteractionSystem;

namespace Valve.VR.InteractionSystem
{
    public class TurnAgent : MonoBehaviour
    {
        public SteamVR_Action_Boolean confirmTargetAction;

        public GameObject agent;

        public Hand hand;

        public bool clockwiseTurn;

        [Tooltip("deg/sec")] public float angularSpeed = 1f;

        Vector3 previousRobotPos;

        void OnEnable()
        {
            if (hand == null)
                hand = this.GetComponent<Hand>();

            if (confirmTargetAction == null)
            {
                Debug.LogError("<b>[SteamVR Interaction]</b> No action assigned");
                return;
            }

            previousRobotPos = agent.transform.position;
        }

        void Update()
        {
            if (IsActionButtonDown(hand))
            {
                if (IsEligibleForTurn())
                {
                    if (clockwiseTurn)
                    {
                        agent.transform.Rotate(Vector3.up, angularSpeed * Time.deltaTime);
                    }
                    else
                    {
                        agent.transform.Rotate(Vector3.up, -angularSpeed * Time.deltaTime);
                    }
                }
                previousRobotPos = agent.transform.position;
            }
        }

        private bool IsActionButtonDown(Hand hand)
        {
            return confirmTargetAction.GetState(hand.handType);
        }

        // Do not allow turning while agent is moving
        private bool IsEligibleForTurn()
        {
            float distanceMoved = Vector3.Distance(agent.transform.position, previousRobotPos);
            if (distanceMoved < float.Epsilon)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
}
