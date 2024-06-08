from copy import deepcopy
import gymnasium as gym
from typing import Optional, Union

from gymnasium.envs.registration import EnvSpec

class TimeLimitWrapper(gym.Wrapper, gym.utils.RecordConstructorArgs):
    """This wrapper will issue a `truncated` signal if a maximum number of timesteps is exceeded.

    If a truncation is not defined inside the environment itself, this is the only place that the truncation signal is issued.
    Critically, this is different from the `terminated` signal that originates from the underlying environment as part of the MDP.
    """

    def __init__(self, env: gym.Env, max_episode_steps: int):
        """Initializes the :class:`TimeLimitWrapper` with an environment and the number of steps after which truncation will occur.

        Args:
            env: The environment to apply the wrapper
            max_episode_steps: An optional max episode steps (if ``None``, ``env.spec.max_episode_steps`` is used)
        """
        gym.utils.RecordConstructorArgs.__init__(self, max_episode_steps=max_episode_steps)
        gym.Wrapper.__init__(self, env)

        self._max_episode_steps = max_episode_steps
        self._elapsed_steps = 0

    def step(self, action):
        """Steps through the environment and if the number of steps elapsed exceeds ``max_episode_steps`` then truncate.

        Args:
            action: The environment step action

        Returns:
            The environment step ``(observation, reward, terminated, truncated, info)`` with `truncated=True`
            if the number of steps elapsed >= max episode steps
        """
        observation, reward, terminated, truncated, info = self.env.step(action)
        self._elapsed_steps += 1

        if self._elapsed_steps >= self._max_episode_steps:
            truncated = True
            info['time_limit_reached'] = True
            if 'is_success' not in info:
                info['is_success'] = False

        return observation, reward, terminated, truncated, info

    def reset(self, **kwargs):
        """Resets the environment with :param:`**kwargs` and sets the number of steps elapsed to zero.

        Args:
            **kwargs: The kwargs to reset the environment with

        Returns:
            The reset environment
        """
        self._elapsed_steps = 0
        return self.env.reset(**kwargs)

    @property
    def spec(self) -> Optional[EnvSpec]:
        """Modifies the environment spec to include the `max_episode_steps=self._max_episode_steps`."""
        if self._cached_spec is not None:
            return self._cached_spec

        env_spec = self.env.spec
        if env_spec is not None:
            env_spec = deepcopy(env_spec)
            env_spec.max_episode_steps = self._max_episode_steps

        self._cached_spec = env_spec
        return self._cached_spec

# Usage of the public API to get max_episode_steps
def get_env_params(env):
    params = {}
    params['max_timesteps'] = env.spec.max_episode_steps if env.spec else env._max_episode_steps
    return params
