﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR.InteractionSystem;
using UnityEngine.AI;

namespace Valve.VR.InteractionSystem
{
    public class DirectControl : MonoBehaviour
    {
        public SteamVR_Action_Boolean toggleControlMethod;
        public SteamVR_Action_Boolean forwardAction;
        public SteamVR_Action_Boolean reverseAction;
        public SteamVR_Action_Boolean leftAction;
        public SteamVR_Action_Boolean rightAction;

        public Hand hand;

        public GameObject[] targetDisks;

        public float linearSpeed = 0.5f;
        public float angularSpeed = 1f;

        private bool directControlOn = false;

        void OnEnable()
        {
            if (hand == null)
                Debug.LogError("<b>[SteamVR Interaction]</b> No hand assigned");

            if (forwardAction == null || reverseAction == null || leftAction == null || rightAction == null)
            {
                Debug.LogError("<b>[SteamVR Interaction]</b> No action assigned");
                return;
            }

            toggleControlMethod.AddOnChangeListener(OnConfirmActionChange, hand.handType);
        }

        private void OnDisable()
        {
            if (toggleControlMethod != null)
                toggleControlMethod.RemoveOnChangeListener(OnConfirmActionChange, hand.handType);
        }

        private void OnConfirmActionChange(SteamVR_Action_Boolean actionIn, SteamVR_Input_Sources inputSource, bool newValue)
        {
            if (newValue)
            {
                directControlOn = !directControlOn;
                if (directControlOn)
                {
                    EnableNavAgent(false);
                    EnableTargetDisks(false);
                }
                else
                {
                    EnableNavAgent(true);
                    EnableTargetDisks(true);
                }
            }
        }

        private void EnableTargetDisks(bool on)
        {
            if (targetDisks != null)
            {
                foreach (GameObject targetDisk in targetDisks)
                {
                    targetDisk.transform.position = transform.position;
                    targetDisk.SetActive(on);
                }
            }
        }

        private void EnableNavAgent(bool on)
        {
            GetComponent<NavMeshAgent>().enabled = on;
            GetComponent<SampleAgentScript>().enabled = on;
            GetComponent<LineRenderer>().enabled = on;
        }

        private bool IsActionButtonDown(Hand hand, SteamVR_Action_Boolean action)
        {
            return action.GetState(hand.handType);
        }

        void Update()
        {
            if (directControlOn)
            {
                if (IsActionButtonDown(hand, forwardAction))
                {
                    transform.Translate(Vector3.forward * linearSpeed * Time.deltaTime);
                }

                if (IsActionButtonDown(hand, reverseAction))
                {
                    transform.Translate(Vector3.forward * -linearSpeed * Time.deltaTime);
                }

                if (IsActionButtonDown(hand, rightAction))
                {
                    transform.Rotate(transform.up * angularSpeed * Time.deltaTime);
                }

                if (IsActionButtonDown(hand, leftAction))
                {
                    transform.Rotate(transform.up * -angularSpeed * Time.deltaTime);
                }
            }
        }
    }
}
