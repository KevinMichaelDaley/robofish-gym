using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
public class FormationAcademy : MonoBehaviour
{
    public GameObject self;
    public void Awake()
    {
        Academy.Instance.OnEnvironmentReset += EnvironmentReset;
    }

    void EnvironmentReset()
    {
	var objects=Object.FindObjectsOfType<FishAgent>();
	foreach(var agent in objects){
		agent.Reset();
	}
    }
}
