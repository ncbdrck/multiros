import numpy as np
import gymnasium as gym

import rospy


class NormalizeActionWrapper(gym.Wrapper):
    """
    A wrapper for normalizing the action space of an environment.

    This wrapper normalizes the actions to be between -1.0 and 1.0.
    It can be used with environments whose action space is of type Box.

    Args:
        env (gym.Env): The environment to wrap.

    Raises:
        ValueError: If the action space of the environment is not of type Box.
    """

    def __init__(self, env):
        # init the wrapper
        super().__init__(env)

        # get the current action space
        action_space = env.action_space

        # Get the low and high values of the original action space
        self.low = action_space.low
        self.high = action_space.high

        # Check if the action space of the environment is of type Box
        if not isinstance(action_space, gym.spaces.Box):
            raise ValueError(f"Expected env.action_space to be of type Box, but got {type(action_space)}")

        # Set the action space of the wrapper to be a Box with low=-1.0 and high=1.0
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=self.env.action_space.shape, dtype=np.float32)

    def denormalize_action(self, action):
        """
        Normalize the action to the range of the original action space.

        Args:
            action (np.ndarray): The action to normalize.

        Returns:
            np.ndarray: The normalized action.
        """
        # Normalize the action to the range of the original action space
        action = self.low + (action + 1.0) * 0.5 * (self.high - self.low)

        # Clip the action to be within the range of the original action space
        action = np.clip(action, self.low, self.high)

        return action

    def reverse_action(self, action):
        """
        Reverse the normalization of an action.

        Args:
            action (np.ndarray): The action to denormalize.

        Returns:
            np.ndarray: The denormalized action.
        """
        # Reverse the normalization of the action
        action = 2 * (action - self.low) / (self.high - self.low) - 1.0

        return action

    def step(self, action):
        """
        Take a step in the environment using a normalized action.

        Args:
            action (np.ndarray): The normalized action to take.

        Returns:
            observation (Any): The observation representing the current state of the environment.
            reward (float): The reward for taking the given action.
            done (bool): Whether the episode has ended.
            info (dict): Additional information about the environment.
        """
        # Denormalize the action before passing it to the underlying environment's step method
        # rospy.logwarn(f"action from policy:{action}")
        action = self.denormalize_action(action)
        # rospy.logwarn(f"action after norm:{action}")
        return self.env.step(action)
