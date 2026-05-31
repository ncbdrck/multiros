"""
Unit tests for the MuJoCo scene composition (MujocoSceneManager).

These exercise the pure-XML composition logic that backs the
reload-based spawn/delete mechanism: adding/removing objects,
free-joint emission, asset merging, and resolving the base scene's
asset directories to absolute paths so a regenerated scene resolves
meshes from any working directory. The reload service call itself is
monkeypatched out — these tests do not require a running simulator.
"""
import xml.etree.ElementTree as ET

import pytest

from multiros.utils import mujoco_models


BASE = """<mujoco model="base">
  <compiler meshdir="meshes"/>
  <worldbody>
    <geom name="ground" type="plane" size="1 1 0.1"/>
    <body name="robot_base"><geom type="box" size="0.05 0.05 0.05"/></body>
  </worldbody>
</mujoco>"""


@pytest.fixture
def no_reload(monkeypatch):
    """Capture mujoco_reload calls instead of hitting the ROS service."""
    calls = []
    def _capture(model_path=None, model_string=None, server_name="mujoco_server", ros_port=None):
        calls.append({"model_path": model_path, "model_string": model_string})
        return True, "ok"
    monkeypatch.setattr(mujoco_models, "mujoco_reload", _capture)
    return calls


class TestComposition:

    def test_spawn_primitive_appends_body_with_free_joint(self):
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE)
        sm.spawn_primitive("cube", geom_type="box", size=(0.02, 0.02, 0.02),
                           pos_x=0.2, pos_z=0.1, reload=False)
        root = ET.fromstring(sm._compose())
        bodies = {b.get("name"): b for b in root.find("worldbody").findall("body")}
        assert "cube" in bodies
        assert bodies["cube"].find("freejoint").get("name") == "cube_freejoint"

    def test_quaternion_is_emitted_in_mjcf_order(self):
        # ROS order is (x, y, z, w); MJCF order is (w, x, y, z).
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE)
        sm.spawn_primitive("cube", ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0, reload=False)
        root = ET.fromstring(sm._compose())
        cube = [b for b in root.find("worldbody").findall("body") if b.get("name") == "cube"][0]
        assert cube.get("quat") == "1.0 0.0 0.0 0.0"

    def test_get_model_names_tracks_objects(self):
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE)
        sm.spawn_primitive("cube", reload=False)
        sm.spawn_primitive("ball", geom_type="sphere", size=(0.03,), reload=False)
        assert sm.get_model_names() == ["cube", "ball"]

    def test_remove_model_drops_body(self):
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE)
        sm.spawn_primitive("cube", reload=False)
        sm.spawn_primitive("ball", geom_type="sphere", size=(0.03,), reload=False)
        sm.remove_model("cube", reload=False)
        names = [b.get("name") for b in ET.fromstring(sm._compose()).find("worldbody").findall("body")]
        assert names == ["robot_base", "ball"]
        assert sm.get_model_names() == ["ball"]

    def test_remove_unknown_model_is_noop(self):
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE)
        # Should warn, not raise.
        ok, _ = sm.remove_model("does_not_exist", reload=False)
        assert ok is True

    def test_clear_restores_base_scene(self):
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE)
        sm.spawn_primitive("cube", reload=False)
        sm.clear(reload=False)
        names = [b.get("name") for b in ET.fromstring(sm._compose()).find("worldbody").findall("body")]
        assert names == ["robot_base"]

    def test_spawn_model_merges_assets(self):
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE)
        sm.spawn_model(
            "widget",
            body_string='<body name="widget" pos="0 0 0.5"><freejoint/><geom type="capsule" size="0.01 0.05"/></body>',
            asset_string='<material name="m1" rgba="0 1 0 1"/>',
            reload=False,
        )
        root = ET.fromstring(sm._compose())
        names = [b.get("name") for b in root.find("worldbody").findall("body")]
        assert names == ["robot_base", "widget"]
        assert root.find("asset").find("material").get("name") == "m1"

    def test_requires_a_base_scene(self):
        with pytest.raises(ValueError):
            mujoco_models.MujocoSceneManager()


class TestAssetDirResolution:

    def test_meshdir_absolutized_for_file_base(self, tmp_path):
        scene = tmp_path / "scene.xml"
        scene.write_text(BASE)
        sm = mujoco_models.MujocoSceneManager(base_scene_path=str(scene))
        compiler = ET.fromstring(sm._compose()).find("compiler")
        meshdir = compiler.get("meshdir")
        assert meshdir == str(tmp_path / "meshes")

    def test_string_base_leaves_compiler_untouched(self):
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE)
        compiler = ET.fromstring(sm._compose()).find("compiler")
        # No base directory is known for a string scene, so meshdir is unchanged.
        assert compiler.get("meshdir") == "meshes"


class TestReloadWiring:

    def test_spawn_with_reload_writes_file_and_calls_reload(self, no_reload, tmp_path):
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE, work_dir=str(tmp_path))
        ok, _ = sm.spawn_primitive("cube", reload=True)
        assert ok is True
        assert len(no_reload) == 1
        # A scene file was written and handed to reload by path.
        assert no_reload[0]["model_path"] is not None
        assert no_reload[0]["model_path"].endswith(".xml")

    def test_batched_spawns_single_reload(self, no_reload, tmp_path):
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE, work_dir=str(tmp_path))
        sm.spawn_primitive("cube", reload=False)
        sm.spawn_primitive("ball", geom_type="sphere", size=(0.03,), reload=False)
        sm.reload_scene()
        assert len(no_reload) == 1


class TestCleanup:

    def test_close_removes_last_generated_file(self, no_reload, tmp_path):
        import os
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE, work_dir=str(tmp_path))
        sm.spawn_primitive("cube", reload=True)
        generated = no_reload[-1]["model_path"]
        assert generated is not None and os.path.exists(generated)
        sm.close()
        assert not os.path.exists(generated)

    def test_close_is_idempotent(self, no_reload, tmp_path):
        sm = mujoco_models.MujocoSceneManager(base_scene_string=BASE, work_dir=str(tmp_path))
        sm.spawn_primitive("cube", reload=True)
        sm.close()
        # A second close must not raise even though the file is already gone.
        sm.close()
