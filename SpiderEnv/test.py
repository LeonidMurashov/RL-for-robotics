from SpiderEnv import SpiderEnv
import numpy as np

env = SpiderEnv()
env.reset()
done = False
while not done:
	env.render()
	observation, reward, done, _ = env.step(env.action_sample() + np.array([0,0,0,0,0,0,-45,-45,-45,-45,-45,-45,-45,-45,-45,-45,-45,-45])) #[0,0,0,0,0,0,-45,-45,-45,-45,-45,-45,-45,-45,-45,-45,-45,-45]
	print(reward)