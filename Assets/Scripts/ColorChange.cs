using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ColorChange : MonoBehaviour
{

    public void HighlightObject(bool highlight)
    {
        if(highlight)
        {
            gameObject.GetComponent<Renderer>().material.EnableKeyword("_EMISSION");
        }
        else
        {
            gameObject.GetComponent<Renderer>().material.DisableKeyword("_EMISSION");
        }
    }
}
