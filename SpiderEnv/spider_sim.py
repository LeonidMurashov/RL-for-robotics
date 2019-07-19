from gym_spider import *
spider_env = SpiderEnv()
spider_env.reset_model()
        
for i in range(10000):
    n = np.sin(i/100)*90
    ob, reward, done, reward_dict = spider_env.step(np.array([n for i in range(18)]))
    spider_env.render()