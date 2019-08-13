import tensorflow as tf
import numpy as np
import copy
import random
import gym

tf.reset_default_graph()
sess = tf.Session()

def copy_net(name1, name2):
    variables = tf.trainable_variables()
    for var1 in variables:
        if name2+"/" in var1.name:
            trained_var = [var2 for var2 in tf.trainable_variables() if var2.op.name in str.replace(var1.name, name2+"/", name1+"/")][0]
            value = sess.run(trained_var)
            sess.run(tf.assign(var1, value))

class QualityNet:
    def __init__(self, **kwargs):
        with tf.variable_scope(kwargs.get("net_name")+"/"):
            self.action_space_size = kwargs.get("action_space_size", 2)
            self.observation_space_size = kwargs.get("observation_space_size", 2)
            state_queue_size = kwargs.get("state_queue_size", 4)

            layers_config = kwargs.get("layers_config", (self.action_space_size + state_queue_size*self.observation_space_size, 32))

            self.input_state = tf.placeholder(tf.float32, shape=[None, state_queue_size, self.action_space_size], name="input_state")
            self.flattened_state = tf.reshape(self.input_state, [-1, self.action_space_size * state_queue_size])
            self.input_action = tf.placeholder(tf.int32, shape=[None], name="input_action")
            input_action_one_hot = tf.one_hot(self.input_action, depth=self.action_space_size)

            self.input_data = tf.concat([self.flattened_state, input_action_one_hot], 1)

            self.input_layer = tf.layers.dense(self.input_data, units=layers_config[0], activation='relu')
            self.hidden_layer = tf.layers.dense(self.input_layer, units=layers_config[1], activation = 'relu')
            self.output_layer = tf.layers.dense(self.hidden_layer, units=1)

            self.exp_value = tf.placeholder(tf.float32, name="exp_value")
            self.loss = tf.losses.mean_squared_error(self.exp_value, self.output_layer)

            optimizer = tf.train.RMSPropOptimizer(0.005)
            self.train_op = optimizer.minimize(self.loss)

    def train(self, qvalues, batch): # TODO: refactor for batch
        global sess
        input_state = [batch[:, 0][i].queue for i in range(len(batch[:, 0]))]
        input_action = batch[:, 1]

        _, loss = sess.run((self.train_op, self.loss),
                           feed_dict = {
                                self.input_state: input_state,
                                self.input_action: input_action,
                                self.exp_value: qvalues,
                            })
        return loss

    def feed_forward(self, states_list, action):
        global sess
        output = sess.run((self.output_layer), feed_dict = {self.input_state: states_list, self.input_action: action})
        return output

    def predict(self, state_queue):
        rewards = np.array([])
        for i in range(self.action_space_size+1):
            reward = self.feed_forward([state_queue.queue], [i])
            rewards = np.append(rewards, reward)
        return rewards.max(), np.argmax(rewards)

class StateQueue():
    queue = [np.array([0, 0]), np.array([0, 0]), np.array([0, 0]), np.array([0, 0])]
    def __init__(self, size):
        self.queue_size = size

    def update(self, state):
        self.queue.insert(0, state)
        if len(self.queue) > self.queue_size:
            self.queue.pop()
        return copy.copy(self)

    def last(self):
        return self.queue[-1]

    def size(self):
        return len(self.queue)

    def __iter__(self):
        for s in self.queue:
            yield s

class QLearning:
    def __init__(self, env, **kwargs):
        self.env = env
        self.stochastic_action_likelihood = kwargs.get("stoch_act_ch", 0.5)
        self.stochastic_action_likelihood_d = kwargs.get("stoch_act_ch_d", 0.01)
        self.discount = kwargs.get("discount", 0.9)

        prediction_net_params = kwargs.get("prediction_net_params", {})
        prediction_net_params["net_name"] = "prediction_net"
        self.prediction_net = QualityNet(**prediction_net_params )

        train_net_params = kwargs.get("train_net_params", {})
        train_net_params["net_name"] = "train_net"
        self.train_net = QualityNet(**train_net_params)

    def get_qvalues(self, batch):
        qvals = list()
        for b in batch:
            q, _ = self.prediction_net.predict(b[3])
            qvals.append(b[2] + self.discount * q)
        return qvals

    def get_random_batch(self, history, batch_size):
        return history[np.random.choice(history.shape[0], size=batch_size), :]

    def run(self, iterations=1000, batch_size=64, state_window=4, update_net_period=5, render_every=100):
        history = np.array([])
        """
            history[0] = queue of four last states
            history[1] = action
            history[2] = reward
            history[3] = states queue with new state
        """
        state_queue = StateQueue(state_window)

        for i in range(iterations):
            observation = self.env.reset()
            state_queue.update(observation)

            for j in range(self.env.spec.max_episode_steps):
                if random.random() < self.stochastic_action_likelihood or state_queue.size() < state_window:
                    action = self.env.action_space.sample()
                else:
                    _, action = self.prediction_net.predict(state_queue)

                observation, reward, done, _ = self.env.step(action)

                new_history = (state_queue, action, reward, state_queue.update(observation))
                history = np.append(history, new_history)
                history = history.reshape(len(history) // len(new_history), len(new_history))


                if (i+1) % render_every == 0:
                    self.env.render()

            random_batch = self.get_random_batch(history, batch_size)
            qvalues = self.get_qvalues(random_batch)
            self.train_net.train(qvalues, random_batch)


            if i % update_net_period == 0:
                pass#copy_net("train_net", "prediction_net")

            print("iteration {}".format(i))
            self.stochastic_action_likelihood -= self.stochastic_action_likelihood_d

def main():
    env = gym.make("MountainCar-v0")

    ql = QLearning(env)

    sess.run(tf.global_variables_initializer())
    sess.run(tf.global_variables_initializer())

    ql.run()

if __name__ == "__main__":
    main()

