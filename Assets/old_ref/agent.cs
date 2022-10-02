using System;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;
using Random = UnityEngine.Random;


/// <summary>
/// an class that defines the basic properties of a reinforcement learning agent
/// </summary>
public class FishAgentOld
{
    /// <summary>
    /// [i, j, k, w]
    /// quaternion representing the agent's current orientation
    /// </summary>
    public Quaternion move_tensor;
    public Quaternion move_tensor_0;

    /// <summary>
    /// contains the [x, y, z] coordinates for the FishAgentOld's current position in 3-space
    /// </summary>
    public Vector3 pos_tensor;

    /// <summary>
    /// contains the [dx, dy, dz] velocities that define the FishAgentOld's current speed
    /// </summary>
    public Vector3 vel_tensor;

    /// <summary>
    /// class that defines the reward distribution (orientation, reward)
    /// defined in `reward.cs`
    /// </summary>
    public RewardTensor reward_tensor;
    
    /// <summary>
    /// amount of reward given
    /// </summary>
    
    public float reward_rate;
    
    /// <summary>
    /// name that identifies the agent (e.g. "predator")
    /// </summary>
    public string name;

    /// <summary>
    ///   Time limit for simulation
    /// </summary>
    public static int time_lim;
    
    /// <summary>
    ///   defines the number of bins used for the reward distribution
    /// </summary>
    private int nr_reward_bins;

    /// <summary>
    ///   Distance threshold that determines when the predator is close enough to the prey to end the simulation
    /// </summary>
    public float thresh;
    
    ///https://link.springer.com/content/pdf/10.1007%2Fs00285-014-0843-2.pdf
    ///M. Porfiri et al model of zebrafish locomotion
    public float mu_velocity=14.02f;
    public float theta_velocity=4.21f;
    public float sigma_velocity=5.9f;
    
    /// <summary>
    ///   rotation speed
    /// </summary>
    
    public float velocity_scale = 1f;
    
    public float mu_turn=-0.02f;
    public float theta_turn=2.74f;
    public float sigma_turn=2.85f;
    
    public float sigma_0 = 12f;
    public float turn_speed;
    
    float current_input_w;
    float current_input_v;
    /// wall avoidance term
    float f_W(Transform transform){
        RaycastHit h;
        Physics.SphereCast(transform.position, 0.4f, transform.forward, out h);
        return 3f*Mathf.Exp(-2.98f*h.distance/velocity);
    }
    //turn speed coupled to velocity
    float f_c(){
        return sigma_0*Mathf.Pow(2f*sigma_0/sigma_turn,-velocity/mu_velocity);
    }
    void update_velocity(Transform transform){
        velocity+=theta_velocity*(mu_velocity-velocity)*Time.fixedDeltaTime+sigma_velocity*10f*current_input_v*Time.fixedDeltaTime;
    }
    void update_turn_speed(Transform transform){
        turn_speed+=theta_turn*(mu_turn+f_W(transform)-turn_speed)*Time.fixedDeltaTime+f_c()*10f*current_input_w*Time.fixedDeltaTime;
    }
    /// <summary>
    /// constructor for agent
    /// </summary>
    /// <param name="name">name of agent (e.g. predator)</param>
    /// <param name="time_lim">time to end simulation</param>
    /// <param name="nr_reward_bins">the number of bins to use for approximating the reward distribution</param>
    public FishAgentOld(string name,
                int time_lim,
                int nr_reward_bins,
                Quaternion initial_orient,
                Vector3 initial_pos,
                float thresh,
                float velocity,
                float max_dist,
                float turn_speed,
                float velocity_scale,
                float reward_rate)
    {
    
        FishAgentOld.time_lim = time_lim;
        this.name = name;
        this.thresh = thresh;
        this.current_input_v=0f;
        this.current_input_w=0f;
        this.velocity = velocity;
        this.velocity_scale = velocity_scale;
        this.move_tensor = initial_orient;
        this.pos_tensor = initial_pos;
        this.reward_tensor = new RewardTensor(nr_reward_bins, max_dist); // instantiate reward_tensor
        this.nr_reward_bins = nr_reward_bins;
        this.reward_rate=reward_rate;
    }
    
    Quaternion move_tensor0; // previous orientation
    
    float tupd=0f; // @kmd: what is this?

    public float velocity=1f; // velocity
    
    /// <summary>
    /// update the agent position using the `move_tensor` orientation quaternion
    /// </summary>
    /// <remarks>
    /// perform the hamiltonian product Pos' = Pos*Quat
    /// </remarks>
    public void update_position(Transform transform, int time)
    {
	// rotation with orientation quaternion move_tensor
        float angle = Mathf.Abs(Quaternion.Angle(this.move_tensor0, this.move_tensor*this.move_tensor0));
        Quaternion interp = Quaternion.Slerp(this.move_tensor0,this.move_tensor*this.move_tensor0,turn_speed*angle/Mathf.Pow(2f*Mathf.PI,2f)*Time.fixedDeltaTime*(time-tupd));
        
        transform.rotation=interp;
        
	
        // move position using unit velocity (1 unit time increment)
        transform.position=transform.position+transform.forward*Time.fixedDeltaTime*velocity*velocity_scale;
        this.pos_tensor = transform.position;

	// reset reward_tensor if some conditions
        if(((this.reward_tensor.min(transform.position) <= -0.1*Mathf.Max(dist0,15f) || this.reward_tensor.max(transform.position) < 0.0f) &
            time > Mathf.Pow(nr_reward_bins,4.0f)) || transform.forward.magnitude==0.0)
        {
            tupd=time;
            this.move_tensor0=transform.rotation;
            if((reward_tensor.max(transform.position)<0 || reward_tensor.min(transform.position)<=-2000000f*dist0/10.0f) && time>100){
                if(Random.value<0.001){
                    this.move_tensor = Random.rotation;
                    this.reward_tensor.reset(transform.position,current_input_v,current_input_w);
                }
            }
        }
    }

    public void select_optimal_orientation(Transform transform,int time)
    {
        tupd=time;
        this.move_tensor0=transform.rotation;
        float newv=0, neww=0;
        this.move_tensor = Quaternion.FromToRotation(transform.forward,(this.reward_tensor.opt(transform.position, ref newv, ref neww)+Random.onUnitSphere*0.1f).normalized);
        current_input_v=newv*0.9f+Random.value*0.1f-0.05f;
        current_input_w=neww*0.9f+Random.value*0.1f-0.05f;
        update_turn_speed(transform);
        update_velocity(transform);
    }

    public static int increment_time(int time, FishAgentOld agent, Transform transform)
    { // call before or after reward function
        if(time + 1 < time_lim)
        {
            if(Random.value<0.5)
            {   
		agent.select_optimal_orientation(transform,time); // select new movement direction as the maximally rewarding orientation tensor
            }
	    agent.update_position(transform, time); // update position
            time += 1;
        }
        return time;
    }
    
    Matrix4x4 ScalingMatrix;
    public bool exitflag = false;
    private float dist0 = 0.0f;
    private float v0 = 0.0f;
    private float R0 = 0.0f;
    public float fov=0.1f; // defines the field of view for spotting
    public int nr_spotted = 0; // number of times the prey was spotted
    public float R_delta = -1.0f; // degradation rate for reward
    private Vector3[,] visrays; // raycasting (positions, directions)
    public void reward_function(Transform transform, Vector3 target_position, GameObject target_object, bool evade, int time)
    {
        float forward_dot_best=Vector3.Dot(transform.forward,(target_position-transform.position).normalized);
        reward_tensor.degrade(R_delta,transform.position);
        float R = -0.0f;
        bool spotted=false;
        int nr_rays = (int)(180 * this.fov); // calculate number of rays to use
        this.visrays = new Vector3[nr_rays,2];
        RaycastHit h2;
        bool hit_obstacle = Physics.SphereCast(transform.position, 0.5f, transform.forward, out h2, velocity*0.1f);
        if(hit_obstacle){
            if(h2.collider.gameObject!=target_object)
            {
                reward_tensor.upd((target_position-transform.position).normalized,-100.0f,transform.position,current_input_v,current_input_w);
            }
        }
        for(int i=0; i<nr_rays; ++i)
        {
            RaycastHit h;
            visrays[i,0] = transform.position;
            Vector3 dir_sph = transform.forward + transform.right * ((nr_rays/2)-i) * 0.05f + transform.up * (0.5f-Random.value); // direction of ray
            visrays[i,1] = dir_sph;
            bool hit=Physics.Raycast(visrays[i,0], visrays[i,1], out h, 20f); // raycast to determine if the prey was spotted
            bool hit_target = false;
            if(hit){
                hit_target = h.collider.gameObject==target_object;
            }
            spotted=spotted || ( hit_target );
            Color color_draw=(hit_target)?Color.yellow:(hit? Color.blue: Color.black);
            Debug.DrawRay(transform.position, dir_sph*20.0f,color_draw);
        }

        if(spotted)
        {
            reward_tensor.upd((target_position-transform.position).normalized,evade?0.5f:70.0f,transform.position,current_input_v,current_input_w);
            nr_spotted += 1;
        }
        
	// compute distance to target
        float dist = Mathf.Abs(Vector3.Distance(transform.position, target_position));
        dist += Random.value * 0.01f; // noisy distance
	
        Debug.Log("*Distance = " + dist);
	
        if(dist <= thresh)
        { // exit if close enough
            Debug.Log(" ----------> ExitFlag (distance below thresh!)");
            // reward_tensor.upd(transform.forward, 100.0f, transform.position, current_input_v, current_input_w);
            exitflag = true;
        }
        float vbest=0, wbest=0;
        Vector3 D=reward_tensor.opt(transform.position, ref vbest, ref wbest).normalized;
        
        R -= (evade?-reward_rate:reward_rate)*(10f*(dist-dist0)-5.0f*(forward_dot_best-0.25f)); // velocity term of reward function

        
        Debug.Log("v=" + velocity);
        Debug.Log("w=" + turn_speed);
        
        dist0 = dist; // store previous distance
        R0 = R; // ""
        v0 = velocity; // ""
        
        if(D.x==0)
        {
            D.x=0.00001f;
        }
        
        float x=Mathf.Atan(D.y/D.x);
        float y=Mathf.Asin(D.z);
        float[][] M=Utils.genSHscaling(x,y);
        this.reward_tensor.scale(M,transform.position,current_input_v,current_input_w);
        this.reward_tensor.spot(target_position+Random.onUnitSphere*Random.value*0.05f); // updates estimated distance to target
        this.reward_tensor.upd(transform.forward,R,transform.position,current_input_v,current_input_w);
        Debug.Log("Max Reward: " + reward_tensor.max(transform.position) + "; Min Reward: " + reward_tensor.min(transform.position));
        Debug.Log("# Times Spotted: "    + nr_spotted);
    }

}
