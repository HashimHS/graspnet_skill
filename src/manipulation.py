from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus
from copy import deepcopy
import moveit_commander
import sys
import threading
import tf
import rospy
import controller_manager_msgs.srv
from cartesian_trajectory_generator.msg import TrajectoryAction, TrajectoryGoal
import cartesian_trajectory_generator.srv
from geometry_msgs.msg import Pose, PoseStamped
import math

BASE_LINK = 'ur5e_base_link'
END_EFFECTOR_LINK = 'ur5e_ee_graspnet'
MOVE_GROUP = 'manipulator'


JOINTS_LOOKOUT_RIGHT = [3.141664981842041, -1.570361928348877, -1.5737504959106445, -1.5729748211302699, 1.5707507133483887, 1.5422992706298828]
JOINTS_LOOKOUT_FRONT = [4.749281406402588, -1.5703018468669434, -1.573847770690918, -1.5728908977904261, 1.570810317993164, 1.5423951148986816]
JOINTS_STORAGE_APPROACH = [5.13097620010376, -1.1483500760844727, -2.083643913269043, -1.4447429937175293, 1.5789122581481934, 3.53879976272583]
JOINTS_STORAGE = [5.108852863311768, -1.3763912481120606, -2.3912553787231445, -0.9401939672282715, 1.5543475151062012, 3.5336735248565674]
JOINTS_HOME = [1.5704865455627441, 0.0031444269367675304, -2.8347482681274414, -0.32037051141772466, 1.570810317993164, 1.570784568786621]
JOINTS_LOOKOUT_GRASPNET = [3.14136457, -1.5703018, -2.5623636, 0.2357253, 1.572078, 1.5442853]


class Manipulator:
    def __init__(self, end_effector_link="ur5e_ee_link"):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.move_group.MoveGroupCommander(MOVE_GROUP)

        self.group.set_pose_reference_frame(BASE_LINK)
        self.group.set_end_effector_link(end_effector_link)
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(0.3)	

    def goto_pose(self, pose, planning_time=5):
        self.group.set_planning_time(planning_time)
        result = self.group.go(pose, wait=True)
        self.group.stop()
        return result

    def goto_joint_state(self, joint_state, planning_time=1):
        self.group.set_planning_time(planning_time)
        result = self.group.go(joint_state, wait=True)
        self.group.stop()
        return result

    def goto_pose_base(self, pose, planning_time=10):
        # pose here is in base frame coordinates
        self.group.set_pose_target(pose)
        print(pose)
        self.group.set_planning_time(planning_time)
        success = self.group.go(wait=True)
        return success  

#################################################################################
# Predefined poses for a ur5e Arm
#################################################################################

class PredefinedMovement(SkillDescription):
    def createDescription(self):
        pass

class MovementPredefinedJoints(SkillDescription):
    def createDescription(self):
        self.addParam("Joints", JOINTS_HOME, ParamTypes.Required, description="joint values")

class arm_to_predefined_joints(PrimitiveBase):
    def createDescription(self):
        self.setDescription(MovementPredefinedJoints(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def run(self):
        joint_state = list(self.params["Joints"].values)
        manipulator = Manipulator()
        self.result = manipulator.goto_joint_state(joint_state)
        self.done = True

    def onStart(self):
        self.done = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        return True

    def execute(self):
        if not self.done:
            return self.step('Running...')

        self.thread.join()

        if self.result:
            return self.success('Done')
        else:
            return self.fail('Failed', -1)


class arm_lookout_front(SkillBase):
    def createDescription(self):
        self.setDescription(PredefinedMovement(), self.__class__.__name__)

    def expand(self, skill):
        # Change this to Serial
        skill.setProcessor(Sequential())
        skill(
            self.skill('MovementPredefinedJoints', '', specify={'Joints': JOINTS_LOOKOUT_FRONT})
        )

class arm_lookout_graspnet(SkillBase):
    def createDescription(self):
        self.setDescription(PredefinedMovement(), self.__class__.__name__)

    def expand(self, skill):
        # Change this to Serial
        skill.setProcessor(Sequential())
        skill(
            self.skill('MovementPredefinedJoints', '', specify={'Joints': JOINTS_LOOKOUT_GRASPNET})
        )

class arm_storage_approach(SkillBase):
    def createDescription(self):
        self.setDescription(PredefinedMovement(), self.__class__.__name__)

    def expand(self, skill):
        # Change this to Serial
        skill.setProcessor(Sequential())
        skill(
            self.skill('MovementPredefinedJoints', '', specify={'Joints': JOINTS_STORAGE_APPROACH})
        )

class arm_storage(SkillBase):
    def createDescription(self):
        self.setDescription(PredefinedMovement(), self.__class__.__name__)

    def expand(self, skill):
        # Change this to Serial
        skill.setProcessor(Sequential())
        skill(
            self.skill('MovementPredefinedJoints', '', specify={'Joints': JOINTS_STORAGE})
        )


class arm_lookout(SkillBase):
    def createDescription(self):
        self.setDescription(PredefinedMovement(), self.__class__.__name__)

    def expand(self, skill):
        # Change this to Serial
        skill.setProcessor(Sequential())
        skill(
            self.skill('MovementPredefinedJoints', '', specify={'Joints': JOINTS_LOOKOUT_RIGHT})
        )

class arm_home(SkillBase):
    def createDescription(self):
        self.setDescription(PredefinedMovement(), self.__class__.__name__)

    def expand(self, skill):
        # Change this to Serial
        skill.setProcessor(Sequential())
        skill(
            self.skill('MovementPredefinedJoints', '', specify={'Joints': JOINTS_HOME})
        )