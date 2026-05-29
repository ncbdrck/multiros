from typing import Any, Dict, Union

import gymnasium as gym
import numpy as np


class NormalizeObservationWrapper(gym.ObservationWrapper):
    """
    A wrapper for normalizing the observation space of an environment.

    Observations are normalized to ``[-1, 1]``. For Box observation spaces
    this is straightforward; for Dict observation spaces shaped like the
    Gymnasium-robotics ``GoalEnv`` API (keys ``observation``,
    ``achieved_goal``, ``desired_goal``), the ``observation`` slot is
    always normalized; the goal slots are normalized only when
    ``normalize_goal_spaces=True``.

    When the goal slots are normalized, the wrapper also:

      * declares the normalized goal entries in ``observation_space`` as
        ``Box(-1, 1, ...)`` (so the obs the wrapper returns is actually
        inside the declared space), and
      * overrides ``compute_reward`` / ``compute_terminated`` /
        ``compute_truncated`` to unnormalize the goal arguments before
        delegating to the underlying env. This is required for HER-style
        algorithms: the replay buffer stores the WRAPPER's observation
        (normalized goals) and then calls
        ``env.compute_reward(desired_goal, achieved_goal, info)`` for
        reward recomputation. Without the override, the underlying env's
        ``compute_reward`` would receive ``[-1, 1]`` arrays and apply a
        metric-distance tolerance to them, producing meaningless rewards.

    Args:
        env: The environment to wrap.
        normalize_goal_spaces: Whether to also normalize the
            ``achieved_goal`` and ``desired_goal`` entries (default
            ``False``; only meaningful for Dict / GoalEnv spaces).

    Raises:
        ValueError: If the observation space is neither Box nor a Dict
            with the goal-env keys.
    """

    def __init__(self, env: gym.Env, normalize_goal_spaces: bool = False) -> None:

        # init the ObservationWrapper
        super().__init__(env)

        self.normalize_goal_spaces = normalize_goal_spaces

        # Box observation space → normalize to [-1, 1].
        if isinstance(env.observation_space, gym.spaces.Box):
            self.observation_space = gym.spaces.Box(low=-1.0, high=1.0, shape=env.observation_space.shape,
                                                    dtype=np.float32)
            self.normalize_observation = self._normalize_box_observation

        # Dict observation space (GoalEnv) → normalize 'observation', and
        # optionally the goal entries.
        elif isinstance(env.observation_space, gym.spaces.Dict):
            ag_space = env.observation_space['achieved_goal']
            dg_space = env.observation_space['desired_goal']

            # Cache the underlying metric bounds so the unnormalize helpers
            # below can roundtrip values from [-1, 1] back to the original
            # space before delegating to the wrapped env's reward / done.
            self._ag_low = ag_space.low
            self._ag_high = ag_space.high
            self._dg_low = dg_space.low
            self._dg_high = dg_space.high

            if normalize_goal_spaces:
                ag_decl = gym.spaces.Box(low=-1.0, high=1.0, shape=ag_space.shape, dtype=np.float32)
                dg_decl = gym.spaces.Box(low=-1.0, high=1.0, shape=dg_space.shape, dtype=np.float32)
            else:
                ag_decl = ag_space
                dg_decl = dg_space

            self.observation_space = gym.spaces.Dict({
                'observation': gym.spaces.Box(low=-1.0, high=1.0, shape=env.observation_space['observation'].shape,
                                              dtype=np.float32),
                'achieved_goal': ag_decl,
                'desired_goal': dg_decl,
            })
            self.normalize_observation = self._normalize_dict_observation
        else:
            raise ValueError(f"Unsupported observation space: {type(env.observation_space)}")

    def _normalize_box_observation(self, observation: np.ndarray) -> np.ndarray:
        # Normalize a Box observation to be between -1 and 1
        if isinstance(self.env.observation_space, gym.spaces.Box):
            low = self.env.observation_space.low
            high = self.env.observation_space.high

        elif isinstance(self.env.observation_space, gym.spaces.Dict):
            low = self.env.observation_space['observation'].low
            high = self.env.observation_space['observation'].high
        else:
            raise ValueError(f"Unsupported observation space: {type(self.env.observation_space)}")

        observation = 2 * (observation - low) / (high - low) - 1.0

        return observation

    def _normalize_achieved_goal(self, achieved_goal: np.ndarray) -> np.ndarray:
        if not isinstance(self.env.observation_space['achieved_goal'], gym.spaces.Box):
            raise ValueError(f"Unsupported achieved_goal space: {type(self.env.observation_space['achieved_goal'])}")
        return 2 * (achieved_goal - self._ag_low) / (self._ag_high - self._ag_low) - 1.0

    def _normalize_desired_goal(self, desired_goal: np.ndarray) -> np.ndarray:
        if not isinstance(self.env.observation_space['desired_goal'], gym.spaces.Box):
            raise ValueError(f"Unsupported desired_goal space: {type(self.env.observation_space['desired_goal'])}")
        return 2 * (desired_goal - self._dg_low) / (self._dg_high - self._dg_low) - 1.0

    def _unnormalize_achieved_goal(self, achieved_goal: np.ndarray) -> np.ndarray:
        return (np.asarray(achieved_goal) + 1.0) * (self._ag_high - self._ag_low) / 2.0 + self._ag_low

    def _unnormalize_desired_goal(self, desired_goal: np.ndarray) -> np.ndarray:
        return (np.asarray(desired_goal) + 1.0) * (self._dg_high - self._dg_low) / 2.0 + self._dg_low

    def _normalize_dict_observation(self, observation: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        # Build a NEW dict instead of mutating the caller's. The
        # underlying env may cache the dict it returned from step() /
        # reset() (real-time mode does this for the rospy.Timer loop),
        # and SB3 buffers may also retain references. In-place writes
        # corrupted those callers under the old behaviour.
        out = {'observation': self._normalize_box_observation(observation['observation'])}
        if self.normalize_goal_spaces:
            out['achieved_goal'] = self._normalize_achieved_goal(observation['achieved_goal'])
            out['desired_goal'] = self._normalize_desired_goal(observation['desired_goal'])
        else:
            out['achieved_goal'] = observation['achieved_goal']
            out['desired_goal'] = observation['desired_goal']
        return out

    def observation(self, observation: Union[np.ndarray, Dict[str, np.ndarray]]) -> Union[np.ndarray, Dict[str, np.ndarray]]:

        # Normalize the observation using the appropriate method
        return self.normalize_observation(observation)

    # ----- GoalEnv hooks: HER replay-buffer recomputation path ---------------
    # When normalize_goal_spaces=True the wrapper returns goals in [-1, 1].
    # The wrapped env's compute_reward / compute_terminated / compute_truncated
    # interpret goals in the original metric units. Unnormalize before
    # delegating so HER (which stores the wrapper's observation and calls
    # these methods on relabeled transitions) sees a consistent metric view.
    #
    # Argument order follows the gymnasium-robotics convention
    # (achieved_goal, desired_goal, info), matching SB3 2.1+ HER which
    # invokes env.compute_reward(next_obs["achieved_goal"],
    # obs["desired_goal"], infos). Each goal is unnormalized with the
    # bounds of its own space before the call is forwarded.

    def compute_reward(self, achieved_goal, desired_goal, info):
        if self.normalize_goal_spaces:
            achieved_goal = self._unnormalize_achieved_goal(achieved_goal)
            desired_goal = self._unnormalize_desired_goal(desired_goal)
        return self.env.compute_reward(achieved_goal, desired_goal, info)

    def compute_terminated(self, achieved_goal, desired_goal, info):
        if self.normalize_goal_spaces:
            achieved_goal = self._unnormalize_achieved_goal(achieved_goal)
            desired_goal = self._unnormalize_desired_goal(desired_goal)
        return self.env.compute_terminated(achieved_goal, desired_goal, info)

    def compute_truncated(self, achieved_goal, desired_goal, info):
        if self.normalize_goal_spaces:
            achieved_goal = self._unnormalize_achieved_goal(achieved_goal)
            desired_goal = self._unnormalize_desired_goal(desired_goal)
        return self.env.compute_truncated(achieved_goal, desired_goal, info)
