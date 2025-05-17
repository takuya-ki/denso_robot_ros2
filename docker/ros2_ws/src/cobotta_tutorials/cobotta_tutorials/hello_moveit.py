import sys
import copy
import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy
from config import MOVEIT_CONFIG
from quaternion import euler_to_quaternion, quaternion_to_euler


class CobottaArm:
    def __init__(self, node_name="moveit_py", moveit_config=MOVEIT_CONFIG, planning_group="arm"):
        self.logger = get_logger(node_name)
        self.moveit = MoveItPy(node_name=node_name, config_dict=moveit_config.to_dict())
        self.robot_model = self.moveit.get_robot_model()
        self.planner = self.moveit.get_planning_component(planning_group)

        self.eef = "J6"
        self.base_frame = "world"

    def get_current_state(self):
        return self.planner.get_start_state()

    def get_current_pose(self):
        return self.get_current_state().get_pose(self.eef)

    def get_current_position(self):
        pose = self.get_current_pose()
        return pose.position.x, pose.position.y, pose.position.z

    def get_current_rpy(self):
        quat = self.get_current_pose().orientation
        return quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0):
        current_pose = self.get_current_pose()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.base_frame
        goal_pose.pose = copy.deepcopy(current_pose)
        goal_pose.pose.position.x += dx
        goal_pose.pose.position.y += dy
        goal_pose.pose.position.z += dz

        return self._plan_and_execute(goal_pose)

    def move_absolute(self, x, y, z, roll, pitch, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.base_frame
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        return self._plan_and_execute(goal_pose)

    def _plan_and_execute(self, goal_pose):
        self.planner.set_start_state_to_current_state()
        self.planner.set_goal_state(pose_stamped_msg=goal_pose, pose_link=self.eef)

        self.logger.info("Planning trajectory")
        result = self.planner.plan()

        if result:
            self.logger.info("Executing trajectory")
            self.moveit.execute(result.trajectory, controllers=[])
            return True
        else:
            self.logger.error("Planning failed")
            return False

    def shutdown(self):
        self.moveit.shutdown()


def main():
    if len(sys.argv) < 3:
        print("Usage: ros2 run <your_package> <this_script> <dx> <dz>")
        sys.exit(1)

    dx = float(sys.argv[1])
    dz = float(sys.argv[2])

    rclpy.init()

    arm = CobottaArm()
    print("Available links:", arm.robot_model.joint_model_groups[0].link_model_names)
    print("Current pose:", arm.get_current_pose())

    success = arm.move_relative(dx=dx, dz=dz)
    if not success:
        print("Motion failed.")

    print("======= END")
    arm.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()