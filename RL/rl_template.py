from mlagents_envs.environment import UnityEnvironment
from gym_unity.envs import UnityToGymWrapper
import numpy as np
class Simulation:
    def __init__(self, agents):
        uenv=UnityEnvironment("Formation",timeout_wait=1000)
        self.env = UnityToGymWrapper(uenv)
        self.env.reset()
        self.agents=agents
        self.done_agents=[]
    def step(self, done_condition=25):
        status=self.env.active
        if status is None:
            self.env.step(self.env.action_space.sample())
        if status.done and status.agent not in self.done_agents:
            self.done_agents.append(status.agent)
            if len(self.done_agents)>=done_condition:
                self.env.reset()
                self.done_agents=[]
        a=self.agents[status.behavior](status.agent, status.obs, status.reward, self.env.action_space)
        self.env.step(a)

class FishAgent:
    def __init__(self, model, test=False):
        self.history={}
        self.actions={}
        self.test=test
        self.model=model
    def __call__(self, agentid, state_at_time, reward, action_space):
        if agentid not in self.history:
            self.history[agentid]=[np.concatenate([np.zeros(2),state_at_time,state_at_time,reward*np.ones(1)])]
            self.actions[agentid]=[np.zeros(2)]
        else:
            self.history[agentid].append(np.concatenate([self.actions[agentid][-1],state_at_time,self.history[agentid][-1][0:2],reward*np.ones(1)]))
        a=self.model.policy(agentid, state_at_time, action_space)
        self.actions[agentid].append(a)
        if not self.test:
            self.model.train(self.history)


def straight_line_error(s,s0,c):
    return 1/2/np.pi*np.arctan((s[4]-s0[4])/(s[2]-s0[2]+1e-7))+c

class PISteering:
    def __init__(self, error=straight_line_error, kp=0.05, ki=0.15):
        self.i=0
        self.kp=kp
        self.ki=ki
        self.error=error
    def __call__(self, s, s0, signal):
        e=self.error(s,s0,signal)
        print(s[0],s[2],s[4],signal)
        self.i+=e*(s[0]-s0[0])
#        d=(self.error(s,signal)-self.error(s0,signal))/(s[0]-s0[0])
        return np.maximum(-1,np.minimum(1,self.kp*e+self.ki*self.i)) #+self.kd*d
    def train(self,hist):
        pass

class SyncModel:
    def __init__(self, steering_model=PISteering()):
        self.steering=steering_model
        self.vel=[]
        self.trajectory={}
    def train(self, history):
        self.trajectory=history
        self.steering.train(history)
    def policy(self, i, s, space):
        if i not in self.trajectory:
            return space.sample()
        if len(self.trajectory[i])==0:
            return space.sample()
        s0=self.trajectory[i][-1][2:]
        pulse=self.steering(s,s0,s[-1])
        signal=np.linalg.norm(s[1:4]-s0[1:4])/(s[0]-s0[0])
        return np.array([pulse,signal])
    

         

            



    

    
def run(model_formation=SyncModel(), model_infiltrator=SyncModel(), model_wrecker=SyncModel(), test=False):
    world=Simulation({"FishBehavior":FishAgent(model_formation,test), "FishInfiltratorBehavior":FishAgent(model_infiltrator,test), "FishWreckerBehavior":FishAgent(model_wrecker,test)})
    for t in range(100000):
        world.step()

if __name__=="__main__":
    run()
