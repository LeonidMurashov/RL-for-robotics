import mujoco_py as mp
import os
import numpy as np
import random
from gym import utils
from gym.envs.mujoco import mujoco_env

class SpiderEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        self.rep_steps = 5
        self.env_timer = 0
        self.model = mp.load_model_from_path('/home/suriknik/Projects/sirius/mujoco/spider.xml')
        self.simulation = mp.MjSim(self.model)
        self.viewer = None
        self.direction = np.array([1, 0])
        self.prevCoord = self.simulation.data.body_xpos[1][:2]
        mujoco_env.MujocoEnv.__init__(self, '/home/suriknik/Projects/sirius/mujoco/spider.xml', 5)
        utils.EzPickle.__init__(self)
    
    #движение серво 18 входных данных
    def _move(self, angles):
        for i in range(len(angles)):
            self.simulation.data.ctrl[i] = angles[i]

    #берет данные из среды
    def step(self, act):
        act += np.array([-(i > 5) * 45 for i in range(18)])

        self._move(act)
        
        for i in range(self.rep_steps):
            self.simulation.step()

        obs = np.concatenate([self.simulation.data.ctrl, self.direction])

        current_coord = self.simulation.data.body_xpos[1][:2]
        rew = ((current_coord - self.prevCoord) * self.direction).sum()*1000 + 0.5
        done = self.simulation.data.body_xpos[1][2] < 0.7
        self.prevCoord = current_coord.copy()


        self.env_timer += 1
        if self.env_timer >= 1500:
            done = True

        return obs, rew, done, 0

    def render(self):
        if self.viewer == None:
            self.viewer = mp.MjViewer(self.simulation)
        self.viewer.render()

    def action_sample(self):
        return [random.randint(-90, 90) for i in range(18)]

    def reset(self):
        self.env_timer = 0
        self.simulation.reset()
        self.simulation.step()
        self._move([-(i > 5) * 45 for i in range(18)])
        for i in range(50):
            self.simulation.step()

        obs = np.concatenate([self.simulation.data.ctrl, self.direction])
        self.prevCoord = self.simulation.data.body_xpos[1][:2].copy()
        return obs

    def close(self):
        if self.viewer is not None:
            # self.viewer.finish()
            self.viewer = None
            self._viewers = {}
