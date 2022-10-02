using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
public class FishAgent : Agent
{
    FishModel model;
    public float distance_noise=0.4f;
    public float ambient_noise=0.04f;
    public float sensor_limit=30f;
    public bool negative_agent=false;
    public bool bumper_agent=false;
    public float dof_noise=0.2f;
    public float vel_noise=0.4f;
    public float signal=0f;
    public float broadcast=0f;
    public int batch=0;
    // Start is called before the first frame update
    public override void Initialize()
    {
        model=GetComponent<FishModel>();
      	model.transform.position=new Vector3(Random.value*120+batch*99999,Random.value*120+batch*99999,Random.value*120+batch*99999);
    }
    public override void OnEpisodeBegin(){    
	    Reset();
    }
    public void Reset(){
    if(model==null){
    
        model=GetComponent<FishModel>();
    }
     	model.transform.position=new Vector3(Random.value*120,Random.value*120,Random.value*120);
      float theta=Random.value;
      model.transform.forward=new Vector3(Mathf.Cos(theta),0f,Mathf.Sin(theta)).normalized;
      model.U=Random.value*model.l;
      model.time=0f;
    }
    public override void CollectObservations(VectorSensor sensor){
    	RaycastHit hitinfo;
	sensor.AddObservation((float)model.time);
    	if(Physics.Raycast(model.transform.position, model.transform.forward, out hitinfo, sensor_limit)){
	    	float r=hitinfo.distance;
	    	float noise=((float)FishModel.RandomNormal())*distance_noise*(r)/sensor_limit;
	    	sensor.AddObservation(r*(1+noise)+(float)FishModel.RandomNormal()*ambient_noise);//TODO: simulated sonar propagation 
	}
	else{
	    	float noise=((float)FishModel.RandomNormal())*distance_noise;
		sensor.AddObservation(noise+sensor_limit);
	}
   	float a=((float)FishModel.RandomNormal())*dof_noise;
  	float b=((float)FishModel.RandomNormal())*dof_noise;
  	float c=((float)FishModel.RandomNormal())*dof_noise;
  	
   	float d=((float)FishModel.RandomNormal())*dof_noise;
  	float e=((float)FishModel.RandomNormal())*dof_noise;
  	float f=((float)FishModel.RandomNormal())*dof_noise;
	//magnetic field sensor for noisy position estimate
	sensor.AddObservation(transform.position+new Vector3(a,b,c));
	//gyro
	sensor.AddObservation(transform.eulerAngles+new Vector3(d,e,f));
	//sonar from neighbors.
	sensor.AddObservation(signal);
    }
    public override void OnActionReceived(ActionBuffers actionBuffers){
	model.pulse=actionBuffers.ContinuousActions[0];
	broadcast=actionBuffers.ContinuousActions[1];
	
	var potential_neighbors=FindObjectsOfType<FishAgent>();
	foreach(var agent in potential_neighbors){
		float r=(transform.position-agent.transform.position).magnitude;
		if(r<sensor_limit){
			float transmission=broadcast/(1+r*r);
			float noise=((float)FishModel.RandomNormal())*r*distance_noise/sensor_limit;
			agent.signal+=transmission*(1+noise)+(float)FishModel.RandomNormal()*ambient_noise;
		}
	}
    }
    public void OnCollisionEnter(Collision c){
    	if(c.collider.GetComponent<FishModel>()!=null){
		 SetReward((negative_agent && bumper_agent)?10000f:-10000f);
	}
	else{
		SetReward(-10000f);	
		var potential_neighbors=FindObjectsOfType<FishAgent>();
		foreach(var agent in potential_neighbors){
			float r=(transform.position-agent.transform.position).magnitude;
			if(r<sensor_limit && agent.negative_agent){
				agent.SetReward(10000f);
			}
		}
	} 
	EndEpisode();//TODO: figure out how to synchronize these calls for the whole swarm.
    }
    public void Update(){	
	int N=0;
	Vector3 mean_speed=new Vector3(0,0,0);
	var potential_neighbors=FindObjectsOfType<FishAgent>();
	foreach(var agent in potential_neighbors){
		if(agent.model==null) continue;
			
		float r=(transform.position-agent.transform.position).magnitude;
		if(r<sensor_limit){
			float a=((float)FishModel.RandomNormal())*vel_noise*r/sensor_limit;
			float b=((float)FishModel.RandomNormal())*vel_noise*r/sensor_limit;
			float c=((float)FishModel.RandomNormal())*vel_noise*r/sensor_limit;
			mean_speed+=(float)agent.model.U*agent.transform.forward+new Vector3(a,b,c);
			N+=1;
		}
	}
	SetReward(Vector3.Dot((mean_speed/N),(float)model.U*transform.forward));
	if(Mathf.Abs(signal)>100 && !negative_agent){
		SetReward(signal>0?100:-100);
	}
     }
     
    public override void Heuristic(in ActionBuffers actionsOut)
    {
	    var continuousActionsOut = actionsOut.ContinuousActions;
	    continuousActionsOut[0] = Input.GetAxis("Horizontal");
	    continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}
