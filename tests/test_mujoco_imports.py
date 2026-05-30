"""
Smoke tests: the MuJoCo backend modules import and expose the expected
public surface, mirroring the Gazebo backend. With the message packages
stubbed (see conftest), the service-call helpers are importable; the
import guard means they also import cleanly when mujoco_ros_pkgs is
absent (the modules just raise a clear error when the helpers are used).
"""


class TestMujocoModuleSurface:

    def test_core_helpers_present(self):
        from multiros.utils import mujoco_core
        for name in ("launch_mujoco", "close_mujoco", "pause_mujoco",
                     "unpause_mujoco", "reset_mujoco", "mujoco_step"):
            assert hasattr(mujoco_core, name), f"mujoco_core.{name} missing"

    def test_physics_helpers_present(self):
        from multiros.utils import mujoco_physics
        for name in ("set_mujoco_max_update_rate", "get_mujoco_max_update_rate",
                     "set_mujoco_time_step", "get_mujoco_time_step",
                     "set_mujoco_gravity", "get_mujoco_gravity", "get_mujoco_sim_info"):
            assert hasattr(mujoco_physics, name), f"mujoco_physics.{name} missing"

    def test_models_helpers_present(self):
        from multiros.utils import mujoco_models
        for name in ("mujoco_reload", "mujoco_get_body_state", "mujoco_set_body_state",
                     "spawn_robot_in_mujoco", "MujocoSceneManager"):
            assert hasattr(mujoco_models, name), f"mujoco_models.{name} missing"

    def test_env_classes_present(self):
        from multiros.envs import MujocoBaseEnv, MujocoGoalEnv
        assert hasattr(MujocoBaseEnv, "MujocoBaseEnv")
        assert hasattr(MujocoGoalEnv, "MujocoGoalEnv")

    def test_default_server_name(self):
        from multiros.utils import mujoco_core, mujoco_models, mujoco_physics
        assert mujoco_core.DEFAULT_SERVER_NAME == "mujoco_server"
        assert mujoco_models.DEFAULT_SERVER_NAME == "mujoco_server"
        assert mujoco_physics.DEFAULT_SERVER_NAME == "mujoco_server"


class TestStepValidation:
    """mujoco_step rejects out-of-range step counts before any ROS call."""

    def test_rejects_zero(self):
        import pytest
        from multiros.utils import mujoco_core
        with pytest.raises(ValueError):
            mujoco_core.mujoco_step(0)

    def test_rejects_negative(self):
        import pytest
        from multiros.utils import mujoco_core
        with pytest.raises(ValueError):
            mujoco_core.mujoco_step(-1)

    def test_rejects_above_uint16(self):
        import pytest
        from multiros.utils import mujoco_core
        with pytest.raises(ValueError):
            mujoco_core.mujoco_step(65536)

    def test_rejects_non_integer(self):
        import pytest
        from multiros.utils import mujoco_core
        with pytest.raises(ValueError):
            mujoco_core.mujoco_step(1.5)

    def test_rejects_bool(self):
        # bool is a subclass of int in Python; True/False are not valid step counts.
        import pytest
        from multiros.utils import mujoco_core
        with pytest.raises(ValueError):
            mujoco_core.mujoco_step(True)
