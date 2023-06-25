import gym


class TimeLimitWrapper(gym.Wrapper):
    """
    A wrapper for limiting the number of steps per episode in an environment.

    This wrapper terminates an episode after a specified number of steps have been taken.

    Args:
        env (gym.Env): The environment to wrap.
        max_steps (int): The maximum number of steps per episode.
    """

    def __init__(self, env, max_steps=100):
        # init Wrapper
        super().__init__(env)

        # Maximum steps for the env
        self.max_steps = max_steps

        # counter to track the current steps
        self.counter = 0

    def step(self, action):
        """
        Take a step in the environment.

        Args:
            action (np.ndarray): The action to take.

        Returns:
            observation (Any): The observation representing the current state of the environment.
            reward (float): The reward for taking the given action.
            done (bool): Whether the episode has ended.
            info (dict): Additional information about the environment.
        """
        # Take a step in the underlying environment
        observation, reward, done, info = self.env.step(action)

        # Increment the step counter
        self.counter += 1

        # If the maximum number of steps has been reached, overwrite done to True
        if self.counter >= self.max_steps:
            done = True
            info['time_limit_reached'] = True

            # This is to log the success rate in stable_baselines3
            info['is_success'] = 0.0

        return observation, reward, done, info

    def reset(self):
        """
        Reset the environment.

        Returns:
            observation (Any): The initial observation of the environment after resetting it.
        """
        # Reset the step counter
        self.counter = 0

        # Reset the underlying environment
        return self.env.reset()
