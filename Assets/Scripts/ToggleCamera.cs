using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR.InteractionSystem;

namespace Valve.VR.InteractionSystem
{
    public class ToggleCamera : MonoBehaviour
    {
        public SteamVR_Action_Boolean confirmTargetAction;

        public Hand hand;

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
                SwitchCameraModes();
            }
        }

        public void SwitchCameraModes()
        {
            TrackRobotPosition trackRobotScript = GetComponentInParent<TrackRobotPosition>();

            if (trackRobotScript.egocentric)
            {
                trackRobotScript.SetTeleportActive(true);
            }
            else
            {
                // trackRobotScript.MovePlayerToRobotPos();
                trackRobotScript.SetTeleportActive(false);
            }

            trackRobotScript.egocentric = !trackRobotScript.egocentric;
            trackRobotScript.trackRotation = !trackRobotScript.trackRotation;
        }
    }
}
