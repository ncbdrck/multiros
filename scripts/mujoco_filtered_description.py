#!/usr/bin/env python3
"""Emit a robot_description with only the requested joints' transmissions kept.

Intended for use as a ``<param ... command="...">`` source in MuJoCo launch files, so the
manufacturer xacro can be reused unmodified while still satisfying mujoco_ros_control (which
aborts on any ``<transmission>`` whose joint is absent from the loaded MJCF).

Usage:
    mujoco_filtered_description.py <robot.urdf.xacro> [xacro args ...] --keep j1 j2 ...

Example (in a launch file):
    <param name="robot_description"
           command="$(find multiros)/scripts/mujoco_filtered_description.py
                    $(find my_robot_description)/urdf/my_robot.urdf.xacro
                    --keep waist shoulder elbow forearm_roll wrist_angle wrist_rotate" />

This is the launch-file counterpart of ``multiros.utils.mujoco_models.filter_urdf_transmissions``
(used by the self-launch / ``spawn_robot_in_mujoco`` path); both keep the same single behaviour.
"""
import subprocess
import sys
import xml.etree.ElementTree as ET


def main() -> int:
    argv = sys.argv[1:]
    if "--keep" not in argv:
        sys.stderr.write("usage: mujoco_filtered_description.py <robot.urdf.xacro> [xacro args] "
                         "--keep j1 j2 ...\n")
        return 2

    split = argv.index("--keep")
    xacro_args = argv[:split]
    keep = set(argv[split + 1:])
    if not xacro_args:
        sys.stderr.write("mujoco_filtered_description: no xacro file given before --keep\n")
        return 2

    # Process the xacro/URDF (the manufacturer description is reused unmodified).
    try:
        urdf_xml = subprocess.check_output(["xacro", *xacro_args], text=True)
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        sys.stderr.write(f"mujoco_filtered_description: xacro failed: {exc}\n")
        return 1

    root = ET.fromstring(urdf_xml)
    removed = []
    for trans in list(root.findall("transmission")):
        joint = trans.find("joint")
        name = joint.get("name") if joint is not None else None
        if name not in keep:
            root.remove(trans)
            removed.append(name)
    if removed:
        sys.stderr.write(f"mujoco_filtered_description: stripped transmissions for {removed}\n")

    sys.stdout.write(ET.tostring(root, encoding="unicode"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
