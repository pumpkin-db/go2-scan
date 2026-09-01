#!/usr/bin/env python3
"""Spawn physical Go2 only after the heavy Gazebo world is fully available."""

import time

import rospy
from gazebo_msgs.srv import GetModelProperties, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("physical_go2_spawn_gate")
    model_name = rospy.get_param("~model_name", "go2_gazebo")
    robot_xml = rospy.get_param("/robot_description")
    required_models = set(rospy.get_param("~required_world_models", []))
    stable_seconds = float(rospy.get_param("~world_stable_seconds", 3.0))
    render_grace_seconds = float(rospy.get_param("~render_sync_grace_seconds", 3.0))
    timeout = float(rospy.get_param("~timeout", 90.0))

    rospy.wait_for_service("/gazebo/get_world_properties", timeout=timeout)
    world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    deadline = time.monotonic() + timeout
    last_models = None
    stable_since = None
    while not rospy.is_shutdown():
        if time.monotonic() >= deadline:
            raise RuntimeError("Gazebo world did not become stable before spawn timeout")
        result = world_properties()
        models = frozenset(result.model_names)
        required_ready = required_models.issubset(models)
        if required_ready and models == last_models:
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= stable_seconds:
                break
        else:
            stable_since = None
        last_models = models
        time.sleep(0.25)

    # Gazebo Classic exposes server readiness but not gzclient scene-sync state.
    # This bounded grace starts only after the complete world model set is stable.
    rospy.loginfo("[physical_spawn] world stable (%d models); waiting %.1fs render sync grace",
                  len(last_models), render_grace_seconds)
    time.sleep(render_grace_seconds)

    pose = Pose()
    pose.position.x = float(rospy.get_param("~x"))
    pose.position.y = float(rospy.get_param("~y"))
    pose.position.z = float(rospy.get_param("~z"))
    q = quaternion_from_euler(0.0, 0.0, float(rospy.get_param("~yaw")))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

    rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=timeout)
    spawn = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    response = spawn(model_name, robot_xml, "/", pose, "world")
    if not response.success:
        raise RuntimeError("Go2 spawn failed: " + response.status_message)

    rospy.wait_for_service("/gazebo/get_model_properties", timeout=timeout)
    properties = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)(model_name)
    if not properties.success or len(properties.body_names) < 13:
        raise RuntimeError("Go2 spawned without its complete 13-link body")

    rospy.wait_for_service("/gazebo/unpause_physics", timeout=timeout)
    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
    rospy.loginfo("[physical_spawn] spawned %s with %d bodies; physics unpaused",
                  model_name, len(properties.body_names))
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logfatal("[physical_spawn] %s", exc)
        raise
