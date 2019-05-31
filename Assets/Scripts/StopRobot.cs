using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR.InteractionSystem;

namespace Valve.VR.InteractionSystem
{
    public class StopRobot : MonoBehaviour
    {
        public SteamVR_Action_Boolean confirmTargetAction;

        public Hand hand;

        public GameObject[] objectsToMove;

        public GameObject robot;

        public float verticalOffset = 0f;

        float communicationDelay;

        private void OnEnable()
        {
            if (hand == null)
                hand = this.GetComponent<Hand>();

            if (confirmTargetAction == null)
            {
                Debug.LogError("<b>[SteamVR Interaction]</b> No action assigned");
                return;
            }

            confirmTargetAction.AddOnChangeListener(OnConfirmActionChange, hand.handType);

            communicationDelay = robot.GetComponent<Status>().communicationDelay;
        }

        private void OnDisable()
        {
            if (confirmTargetAction != null)
                confirmTargetAction.RemoveOnChangeListener(OnConfirmActionChange, hand.handType);
        }

        private void OnConfirmActionChange(SteamVR_Action_Boolean actionIn, SteamVR_Input_Sources inputSource, bool newValue)
        {
            if (newValue)
            {
                StartCoroutine(MoveTargetCoroutine(communicationDelay));
            }
        }

        IEnumerator MoveTargetCoroutine(float delayTime)
        {
            yield return new WaitForSeconds(delayTime);
            Vector3 targetPosition = robot.transform.position;
            targetPosition.y += verticalOffset;
            foreach (GameObject objectToMove in objectsToMove)
            {
                objectToMove.transform.position = targetPosition;
            }
        }
    }
}
