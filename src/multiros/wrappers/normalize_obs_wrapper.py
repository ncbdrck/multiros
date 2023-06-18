import gym
import numpy as np

import rospy
class NormalizeObservationWrapper(gym.ObservationWrapper):
    """
    A wrapper for normalizing the observation space of an environment.

    This wrapper normalizes the observations to be between -1 and 1.
    It can handle environments whose observation space is either a Box or
    a dictionary with keys for 'observation', 'achieved_goal', and 'desired_goal'.

    Args:
        env (gym.Env): The environment to wrap.
        normalize_goal_spaces (bool): Whether to normalize the achieved_goal and desired_goal spaces.

    Raises:
        ValueError: If the observation space of the environment is not supported.
    """

    def __init__(self, env, normalize_goal_spaces=False):

        # init the ObservationWrapper
        super().__init__(env)

        self.normalize_goal_spaces = normalize_goal_spaces

        # check if it is gym.Env based
        if isinstance(env.observation_space, gym.spaces.Box):
            self.observation_space = gym.spaces.Box(low=-1.0, high=1.0, shape=env.observation_space.shape, dtype=np.float32)
            self.normalize_observation = self._normalize_box_observation

        # check if it is gym.GoalEnv based
        elif isinstance(env.observation_space, gym.spaces.Dict):
            self.observation_space = gym.spaces.Dict({
                'observation': gym.spaces.Box(low=-1.0, high=1.0, shape=env.observation_space['observation'].shape, dtype=np.float32),
                'achieved_goal': env.observation_space['achieved_goal'],
                'desired_goal': env.observation_space['desired_goal']
            })
            self.normalize_observation = self._normalize_dict_observation
        else:
            raise ValueError(f"Unsupported observation space: {type(env.observation_space)}")

    def _normalize_box_observation(self, observation):
        # Normalize a Box observation to be between -1 and 1
        if isinstance(self.env.observation_space, gym.spaces.Box):
            low = self.env.observation_space.low
            high = self.env.observation_space.high

            # ---- debug
            # rospy.logwarn(f"observations low:{low}")
            # rospy.logwarn(f"observations high:{high}")

        elif isinstance(self.env.observation_space, gym.spaces.Dict):
            low = self.env.observation_space['observation'].low
            high = self.env.observation_space['observation'].high
        else:
            raise ValueError(f"Unsupported observation space: {type(self.env.observation_space)}")

        observation = 2 * (observation - low) / (high - low) - 1.0

        return observation

    def _normalize_achieved_goal(self, achieved_goal):
        # Check that the achieved_goal_space is a Box space
        if not isinstance(self.env.observation_space['achieved_goal'], gym.spaces.Box):
            raise ValueError(f"Unsupported achieved_goal space: {type(self.env.observation_space['achieved_goal'])}")

        # Normalize an achieved_goal observation to be between -1 and 1
        low = self.env.observation_space['achieved_goal'].low
        high = self.env.observation_space['achieved_goal'].high

        achieved_goal = 2 * (achieved_goal - low) / (high - low) - 1.0

        return achieved_goal

    def _normalize_desired_goal(self, desired_goal):
        # Check that the desired_goal_space is a Box space
        if not isinstance(self.env.observation_space['desired_goal'], gym.spaces.Box):
            raise ValueError(f"Unsupported desired_goal space: {type(self.env.observation_space['desired_goal'])}")

        # Normalize a desired_goal observation to be between -1 and 1
        low = self.env.observation_space['desired_goal'].low
        high = self.env.observation_space['desired_goal'].high

        desired_goal = 2 * (desired_goal - low) / (high - low) - 1.0

        return desired_goal

    def _normalize_dict_observation(self, observation):
        # Normalize a dictionary observation with keys for 'observation', 'achieved_goal', and 'desired_goal'
        observation['observation'] = self._normalize_box_observation(observation['observation'])

        if self.normalize_goal_spaces:
            observation['achieved_goal'] = self._normalize_achieved_goal(observation['achieved_goal'])
            observation['desired_goal'] = self._normalize_desired_goal(observation['desired_goal'])

        return observation

    def observation(self, observation):

        # --------for debug
        # if isinstance(self.env.observation_space, gym.spaces.Box):
        #     low = self.env.observation_space.low
        #     high = self.env.observation_space.high
        # elif isinstance(self.env.observation_space, gym.spaces.Dict):
        #     low = self.env.observation_space['observation'].low
        #     high = self.env.observation_space['observation'].high
        # else:
        #     raise ValueError(f"Unsupported observation space: {type(self.env.observation_space)}")
        #
        # if np.any(observation < low) or np.any(observation > high):
        #     rospy.logwarn(f"Observation out of bounds: {observation}")
        # rospy.logwarn(f"observations before norm:{observation}")
        # rospy.logwarn(f"observations after norm:{self.normalize_observation(observation)}")

        # Normalize the observation using the appropriate method
        return self.normalize_observation(observation)
