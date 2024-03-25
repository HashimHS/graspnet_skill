#!/usr/bin/env python
from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.abstract_skill import State
import skiros2_common.tools.logger as log
import moveit_commander
import threading

import rospy
from cv_bridge import CvBridge, CvBridgeError
import tf
import cv2 as cv
from tf.transformations import quaternion_from_matrix, euler_matrix, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, QuaternionStamped
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

import pyrealsense2

from contact_graspnet_planner.srv import ContactGraspNetPlanner
from wsg_50_common.srv import Move
import numpy as np

import tf2_ros
import tf2_geometry_msgs

from graspnet_skill.manipulation import Manipulator
from graspnet_skill.perception import RGBListener, DepthListener, MLDetector
from robotweek_skills.visualization_graspnet import show_image
from grounding_sam_ros.srv import VitDetection, VitDetectionResponse


CAMERA_FRAME = 'realsense_rgb_optical_frame'
BASE_LINK = 'ur5e_base_link'
END_EFFECTOR_LINK = 'ur5e_ee_graspnet'


JOINTS_LOOKOUT_GRASPNET = [3.14136457, -1.5703018, -2.5623636, 0.2357253, 1.572078, 1.5442853]
JOINTS_LOOKOUT_PLACE = [3.913647174835205, -2.474722524682516, -0.8504171371459961, -1.3707054418376465, 1.6024999618530273, 1.5426583290100098]


class GraspNet(SkillDescription):

    def createDescription(self):
        self.addParam("Prompt", "", ParamTypes.Required, [], "")
        self.addParam("grasp_pose", Element("skiros:TransformationPose"), ParamTypes.Inferred)
        self.addParam("whole_scene", False, ParamTypes.Required, [], "")
        self.addParam("Visualize", False, ParamTypes.Required, [], "")

class detect_graspnet(PrimitiveBase):

    def createDescription(self):
        self.setDescription(GraspNet(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def run(self):

        # Set up the manipulator settings
        rospy.loginfo('Setting up the manipulator...')
        manipulator = Manipulator(end_effector_link=END_EFFECTOR_LINK)
        joint_state = manipulator.group.get_current_joint_values()
        rospy.loginfo('Current joint state: {}'.format(joint_state))

        obj = self.params['Prompt'].value

        if obj == "":
            obj = 'object'

        # We fetch the camera rgb and depth images
        rgb_listener = RGBListener()
        depth_listener = DepthListener()
        rospy.loginfo('Wait for the camera infos...')
        camera_intr = rospy.wait_for_message('/realsense/rgb/camera_info', CameraInfo)
        rospy.loginfo('Camera info received')
        rospy.loginfo('Wait for the camera image...')
        rgb = rgb_listener.get()
        depth = depth_listener.get().astype(np.float32) / 1000
        rospy.loginfo('Camera image received')

        detector = MLDetector('localhost:50051', prompt=obj)

        
        # We segment the object from the background using Grounding Dino
        try:
            rospy.loginfo('Calling the vit_detection service')
            vit_detection = rospy.ServiceProxy('vit_detection', VitDetection)
            cv_bridge = CvBridge()
            rgb_msg = cv_bridge.cv2_to_imgmsg(np.array(rgb))
            results = vit_detection(rgb_msg, obj)
            segmask = results.segmask
            rospy.loginfo('Detected Objects: {}' .format(results.labels))
            rospy.loginfo('Scores: {}'.format(results.scores))
            rospy.loginfo('bounding boxes: {}'.format(results.boxes))
        except:
            rospy.loginfo("Segmentation failed")
            self.result = False
            self.done = True
            return

        if segmask is None:
            self.done = True
            grasp_pose_base = None
            return
        
        # Images for visualization
        rgb_vis = rgb 
        segmask_vis = cv_bridge.imgmsg_to_cv2(segmask)  # segmask

        rospy.loginfo('Converting images to ros messages...')
        cv_bridge = CvBridge()
        # segmask = cv_bridge.cv2_to_imgmsg(np.array(segmask))
        rgb = cv_bridge.cv2_to_imgmsg(np.array(rgb))
        depth = cv_bridge.cv2_to_imgmsg(np.array(depth))
        rospy.loginfo('Conversion done')

        # We use contact_graspnet_planner to detect the grasp pose
        rospy.loginfo('Wait for the grasp_planner server')
        rospy.wait_for_service('/grasp_planner')
        try:
            rospy.loginfo('Generating the grasp poses...')
            contact_graspnet_planner = rospy.ServiceProxy('grasp_planner', ContactGraspNetPlanner)
            if self.params['whole_scene'].value:
                segmask = np.ones((rgb_vis.shape[0], rgb_vis.shape[1]), dtype=np.uint8)
                segmask = cv_bridge.cv2_to_imgmsg(segmask)
            resp = contact_graspnet_planner(rgb, depth, camera_intr, segmask)
            grasp_poses = resp.grasps
            rospy.loginfo('{} grasps generated:' .format(len(grasp_poses)))
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self.result = False
            self.done = True
            return

        if len(grasp_poses) == 0:
            rospy.loginfo('No grasp poses generated')
            self.result = False
            self.done = True
            return

        # We pick the best grasp pose
        Best_grasp = grasp_poses[0]
        rospy.loginfo('Best grasp: {}'.format(Best_grasp))

        try:
            rospy.loginfo('Transforming the grasp pose...')
            tfbuffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tfbuffer)

            # We transform the grasp pose from the camera frame to the base frame
            transform = tfbuffer.lookup_transform(BASE_LINK, CAMERA_FRAME, rospy.Time(0), rospy.Duration(5.0))
            rospy.loginfo('Transformation Lookup: {}'.format(transform))
            grasp_pose_base = tf2_geometry_msgs.do_transform_pose(Best_grasp, transform)
            rospy.loginfo('grasp_pose_base matrix: {}'.format(grasp_pose_base))
            rospy.loginfo('Transformation done')
            self.result = True

            # We send the grasp pose to the skill output
            self.params['grasp_pose'].value.setData(":PoseStampedMsg",grasp_pose_base)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo('Transformation failed: {}'.format(e))
            self.result = False
            self.done = True
            return

        # Visialize the segmentation
        if self.params['Visualize'].value:
            rospy.loginfo('Visualizing the segementation...')
            show_image(rgb_vis, segmask_vis, cv_bridge.imgmsg_to_cv2(results.annotated_frame))
            # cv.imshow('image', cv_bridge.imgmsg_to_cv2(results.annotated_frame))
            # cv.waitKey(0)
            rospy.loginfo('Visualizing the segementation done...')
        
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

class MoveGraspnet(SkillDescription):

    def createDescription(self):
        self.addParam("grasp_pose", Element("skiros:TransformationPose"), ParamTypes.Inferred)

class move_graspnet(PrimitiveBase):

    def createDescription(self):
        self.setDescription(MoveGraspnet(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def run(self):
        # We send the pose to the moveit planner using the manipulator skill
        try:
            # Set up the manipulator settings
            rospy.loginfo('Setting up the manipulator...')
            manipulator = Manipulator(end_effector_link="ur5e_ee_graspnet")
            manipulator.group.set_max_velocity_scaling_factor(0.3)
            manipulator.group.set_max_acceleration_scaling_factor(0.3)

            # Get the grasp pose
            rospy.loginfo('Getting the grasp pose parameter...')
            grasp_pose_base = self.params['grasp_pose'].value.getData(":PoseStampedMsg")

            # Move to the grasp pose
            rospy.loginfo('Moving to the grasp pose...')
            success = manipulator.goto_pose_base(grasp_pose_base, planning_time=5.0)
            if success:
                rospy.loginfo('Successfully moved to the grasp pose')
                self.result = success
            else:
                rospy.loginfo('Failed.')
                self.result = success
        except:
            rospy.loginfo('Failed to move to the grasp pose')
            self.result = False
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
        
class Arm_Lookout_Graspnet(SkillDescription):
    def createDescription(self):
        pass
class arm_lookout_graspnet(SkillBase):
    def createDescription(self):
        self.setDescription(Arm_Lookout_Graspnet(), self.__class__.__name__)

    def expand(self, skill):
        # Change this to Serial
        skill.setProcessor(Sequential())
        skill(
            self.skill('MovementPredefinedJoints', '', specify={'Joints': JOINTS_LOOKOUT_GRASPNET})
        )

class arm_lookout_place(SkillBase):
    def createDescription(self):
        self.setDescription(Arm_Lookout_Graspnet(), self.__class__.__name__)

    def expand(self, skill):
        # Change this to Serial
        skill.setProcessor(Sequential())
        skill(
            self.skill('MovementPredefinedJoints', '', specify={'Joints': JOINTS_LOOKOUT_PLACE})
        )