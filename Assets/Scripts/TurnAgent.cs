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

        void OnEnable()
        {
            if (hand == null)
                hand = this.GetComponent<Hand>();

            if (confirmTargetAction == null)
            {
                Debug.LogError("<b>[SteamVR Interaction]</b> No action assigned");
                return;
            }
        }

        void Update()
        {
            if (IsActionButtonDown(hand) && IsEligibleForTurn())
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
        }

        private bool IsActionButtonDown(Hand hand)
        {
            return confirmTargetAction.GetState(hand.handType);
        }

        private bool IsEligibleForTurn()
        {
            // todo - check that robot is stationary
            return true;
        }
    }
}
