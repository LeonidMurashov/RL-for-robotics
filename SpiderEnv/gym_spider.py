import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env
from os import path


class SpiderEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        mujoco_env.MujocoEnv.__init__(self, path.abspath('./spider.xml'), 10)
        utils.EzPickle.__init__(self)
        self.servo_dict = {14: 5, 8: 6, 2: 7,
                           13: 9, 7: 10, 1: 11,
                           18: 13, 12: 14, 6: 15,
                           17: 18, 11: 17, 5: 16,
                           16: 22, 10: 21, 4: 20,
                           15: 26, 9: 25, 3: 24}

    def step(self, a):
        xpos_before = self.get_body_com("spider")[0]
        self.sim.data.ctrl[:] = a
        self.sim.step()
        xpos_after = self.get_body_com("spider")[0]
        self.render()
        forward_reward = (xpos_after - xpos_before) / self.dt
        ctrl_cost = .5 * np.square(a).sum()
        contact_cost = 0.5 * 1e-3 * np.sum(
            np.square(np.clip(self.sim.data.cfrc_ext, -1, 1)))
        survive_reward = 1.0
        reward = forward_reward - ctrl_cost - contact_cost + survive_reward

        state = self.state_vector()
        done = not np.isfinite(state).all() and 0.2 <= state[2] <= 1.0
        ob = self._get_obs()

        return ob, reward, done, dict(
            reward_forward=forward_reward,
            reward_ctrl=-ctrl_cost,
            reward_contact=-contact_cost,
            reward_survive=survive_reward)

    def _get_obs(self):
        return np.concatenate([self.sim.data.ctrl, self.sim.data.body_xpos[1][:2]])

    def reset_model(self):
        qpos = self.init_qpos + self.np_random.uniform(size=self.model.nq, low=-.1, high=.1)
        qvel = self.init_qvel + self.np_random.randn(self.model.nv) * .1
        self.set_state(qpos, qvel)
        return self._get_obs()

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5
