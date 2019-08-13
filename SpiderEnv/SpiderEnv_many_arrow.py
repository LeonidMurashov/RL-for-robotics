import mujoco_py as mp
import os
import numpy as np
import random
from gym import utils
from gym.envs.mujoco import mujoco_env
import math


arrow_controls_number = 3
ignore_action_period = 20


class SpiderEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        self.env_timer = 0
        self.model = mp.load_model_from_path('D:/Sirius/SpiderEnv/spider_arrow.xml')
        self.simulation = mp.MjSim(self.model)
        self.viewer = None
        self.change_direction()
        self.prevCoord = self.get_position()[:2]
        mujoco_env.MujocoEnv.__init__(self, 'D:/Sirius/SpiderEnv/spider_arrow.xml', 5)
        utils.EzPickle.__init__(self)

    def change_direction(self):
        angle = np.random.random() * 2 * np.pi
        self.direction = [np.cos(angle), np.sin(angle)]

    def get_direction_angle(self):
        value = (90 + math.degrees(math.atan2(self.direction[0], self.direction[1])))
        return value * 0.965

    #движение серво 18 входных данных
    def _move(self, angles):
        self.simulation.data.ctrl[0] = self.get_direction_angle()
        self.simulation.data.ctrl[1:3] = self.get_position()[:2]
        for i in range(len(angles)):
            self.simulation.data.ctrl[i + arrow_controls_number] = angles[i]

    def get_observation(self):
        return np.concatenate([self.simulation.data.ctrl[arrow_controls_number:],
                              self.direction,
                              self.simulation.data.qpos.flatten(),
                              self.simulation.data.qvel.flatten(),
                              self.simulation.data.qacc.flatten(),
                              self.simulation.data.body_xquat.flatten(),
                              self.simulation.data.body_xvelp.flatten(),
                              self.simulation.data.body_xvelr.flatten(),
                              self.simulation.data.cvel.flatten(),
                              self.simulation.data.actuator_force.flatten(),
                              self.simulation.data.actuator_velocity.flatten(),
                              self.simulation.data.actuator_moment.flatten()])

    def get_position(self):
        return self.simulation.data.body_xpos[3]

    #берет данные из среды
    def step(self, act):
        if len(act) == 18 + arrow_controls_number: # MujocoEnv runs this :(
            act = act[arrow_controls_number:]

        if self.env_timer % ignore_action_period == 0:
            act += np.array([-(i > 5) * 45 for i in range(18)])
            self._move(act)

        self.simulation.step()

        if np.random.random() < 0.0005:
            self.change_direction()

        obs = self.get_observation()

        current_coord = self.get_position()[:2]
        rew = ((current_coord - self.prevCoord) * self.direction).sum() * 1000 + 0.5
        done = self.get_position()[2] < 0.5
        self.prevCoord = current_coord.copy()


        self.env_timer += 1
        if self.env_timer >= 3000:
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

        self.change_direction()
        obs = self.get_observation()
        self.prevCoord = self.get_position()[:2].copy()
        return obs

    def close(self):
        if self.viewer is not None:
            # self.viewer.finish()
            self.viewer = None
            self._viewers = {}
