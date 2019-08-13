import mujoco_py as mp
import os
import numpy as np
import random
from gym import utils
from gym.envs.mujoco import mujoco_env
import time


class SpiderEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        self.env_timer = 0
        self.model = mp.load_model_from_path('D:/Sirius/SpiderEnv/spider.xml')
        self.simulation = mp.MjSim(self.model)
        self.viewer = None
        self.direction = np.array([1, 0])
        self.prevCoord = self.simulation.data.body_xpos[1][:2]
        self.save_action = []
        self.b = 10
        self.preww_action = 0
        self.a = 0
        self.obs, self.rew, self.done = 0, 0, 0
        mujoco_env.MujocoEnv.__init__(self, 'D:/Sirius/SpiderEnv/spider.xml', 5)
        utils.EzPickle.__init__(self)
    
    #движение серво 18 входных данных
    def _move(self, angles):
        for i in range(len(angles)):
            self.simulation.data.ctrl[i] = angles[i]

    #берет данные из среды
    def step(self, action):
        
        if self.a == 0:
            act = action
            self.preww_action = act
        if self.a > 0 and self.a <= self.b:
            act = self.preww_action

        act += np.array([-(i > 5) * 45 for i in range(18)])

        self._move(act)
        self.simulation.step()

        self.obs = np.concatenate([self.simulation.data.ctrl,
                              self.direction,
                              self.simulation.data.body_xquat.flatten(),
                              self.simulation.data.qvel.flatten(),
                              self.simulation.data.body_xvelp.flatten(),
                              self.simulation.data.body_xvelr.flatten(),
                              self.simulation.data.cvel.flatten(),
                              self.simulation.data.actuator_force.flatten(),
                              self.simulation.data.actuator_velocity.flatten(),
                              self.simulation.data.actuator_moment.flatten()])

        current_coord = self.simulation.data.body_xpos[1][:2]
        self.rew = ((current_coord - self.prevCoord) * self.direction).sum()*1000 + 0.5
        self.done = self.simulation.data.body_xpos[1][2] < 0.7
        self.prevCoord = current_coord.copy()


        self.env_timer += 1
        if self.env_timer >= 1500:
            self.done = True

        self.a += 1
        if self.a >= self.b:
            self.a = 0
        
        return self.obs, self.rew, self.done, 0

    def render(self):
        if self.viewer == None:
            self.viewer = mp.MjViewer(self.simulation)
        self.viewer.render()

    def robot_step(self, robot):

        for i in range(18):
            robot.move_servo(servo_dict[i + 1], self.simulation.data.ctrl[i])

    def action_sample(self):
        return [random.randint(-90, 90) for i in range(18)]

    def reset(self):
        self.env_timer = 0
        self.simulation.reset()
        self.simulation.step()
        self._move([-(i > 5) * 45 for i in range(18)])
        for i in range(50):
            self.simulation.step()


        obs = np.concatenate([self.simulation.data.ctrl,
                              self.direction,
                              self.simulation.data.body_xquat.flatten(),
                              self.simulation.data.qvel.flatten(),
                              self.simulation.data.body_xvelp.flatten(),
                              self.simulation.data.body_xvelr.flatten(),
                              self.simulation.data.cvel.flatten(),
                              self.simulation.data.actuator_force.flatten(),
                              self.simulation.data.actuator_velocity.flatten(),
                              self.simulation.data.actuator_moment.flatten()])
        self.prevCoord = self.simulation.data.body_xpos[1][:2].copy()
        return obs

    def close(self):
        if self.viewer is not None:
            # self.viewer.finish()
            self.viewer = None
            self._viewers = {}
