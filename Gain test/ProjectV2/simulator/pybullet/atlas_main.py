import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import time, math
from collections import OrderedDict
import copy
import signal
import shutil

import pybullet as p
import numpy as np
np.set_printoptions(precision=2)

from config.atlas_config import SimConfig
from config.atlas_config import SimConfigB
from pnc.atlas_pnc.atlas_interface import AtlasInterface
from pnc.atlas_pnc_b.atlas_interface import AtlasInterface as AtlasInterface_b
from util import pybullet_util
from util import util
from util import liegroup


def set_initial_config(robot, joint_id):
    # shoulder_x
    p.resetJointState(robot, joint_id["l_arm_shx"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_arm_shx"], np.pi / 4, 0.)
    # elbow_y
    p.resetJointState(robot, joint_id["l_arm_ely"], -np.pi / 2, 0.)
    p.resetJointState(robot, joint_id["r_arm_ely"], np.pi / 2, 0.)
    # elbow_x
    p.resetJointState(robot, joint_id["l_arm_elx"], -np.pi / 2, 0.)
    p.resetJointState(robot, joint_id["r_arm_elx"], -np.pi / 2, 0.)
    # hip_y
    p.resetJointState(robot, joint_id["l_leg_hpy"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_leg_hpy"], -np.pi / 4, 0.)
    # knee
    p.resetJointState(robot, joint_id["l_leg_kny"], np.pi / 2, 0.)
    p.resetJointState(robot, joint_id["r_leg_kny"], np.pi / 2, 0.)
    # ankle
    p.resetJointState(robot, joint_id["l_leg_aky"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_leg_aky"], -np.pi / 4, 0.)


def signal_handler(signal, frame):
    # if SimConfig.VIDEO_RECORD:
    # pybullet_util.make_video(video_dir)
    p.disconnect()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":

    # Environment Setup
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(cameraDistance=1.5,
                                 cameraYaw=120,
                                 cameraPitch=-30,
                                 cameraTargetPosition=[1, 0.5, 1.5])
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)  #hide menus
    p.setGravity(0, 0, -9.8)
    p.setPhysicsEngineParameter(fixedTimeStep=SimConfig.CONTROLLER_DT,
                                numSubSteps=SimConfig.N_SUBSTEP)
    # if SimConfig.VIDEO_RECORD:
    # video_dir = 'video/atlas_pnc'
    # if os.path.exists(video_dir):
    # shutil.rmtree(video_dir)
    # os.makedirs(video_dir)

    # Create Robot, Ground
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    robot = p.loadURDF(cwd + "/robot_model/atlas/atlas.urdf",
                       SimConfig.INITIAL_POS_WORLD_TO_BASEJOINT,
                       SimConfig.INITIAL_QUAT_WORLD_TO_BASEJOINT)
                       
    robotB = p.loadURDF(cwd + "/robot_model/atlas/atlas.urdf",
                        SimConfigB.INITIAL_POS_WORLD_TO_BASEJOINT,
                        SimConfigB.INITIAL_QUAT_WORLD_TO_BASEJOINT)

    p.loadURDF(cwd + "/robot_model/ground/plane.urdf", [0, 0, 0])
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        robot, SimConfig.INITIAL_POS_WORLD_TO_BASEJOINT,
        SimConfig.INITIAL_QUAT_WORLD_TO_BASEJOINT, SimConfig.PRINT_ROBOT_INFO)
        
    nq_b, nv_b, na_b, joint_id_b, link_id_b, pos_basejoint_to_basecom_b, rot_basejoint_to_basecom_b = pybullet_util.get_robot_config(
        robotB, SimConfigB.INITIAL_POS_WORLD_TO_BASEJOINT,
        SimConfigB.INITIAL_QUAT_WORLD_TO_BASEJOINT, SimConfigB.PRINT_ROBOT_INFO)

    simplebox = p.loadURDF(cwd + "/robot_model/bookcase/simplebox.urdf",
               basePosition=[0, 0, 1.2],
               baseOrientation=[0, 0, 0, 1])
               
    # Initial Config
    set_initial_config(robot, joint_id)
    set_initial_config(robotB, joint_id_b)

    # Link Damping
    pybullet_util.set_link_damping(robot, link_id.values(), 0., 0.)
    pybullet_util.set_link_damping(robotB, link_id_b.values(), 0., 0.)

    # Joint Friction
    pybullet_util.set_joint_friction(robot, joint_id, 0)
    pybullet_util.set_joint_friction(robotB, joint_id_b, 0)

    # Construct Interface
    interface = AtlasInterface()
    interface_b = AtlasInterface_b()

    # Run Sim
    t = 0
    dt = SimConfig.CONTROLLER_DT
    count = 0
    graphtime = []
    object_pos = []
    while (1):

        # Get SensorData
        # if count % (SimConfig.CAMERA_DT / SimConfig.CONTROLLER_DT) == 0:
        # camera_img = pybullet_util.get_camera_image_from_link(
        # robot, link_id['head'], 60., 2., 0.1, 10)
        sensor_data = pybullet_util.get_sensor_data(robot, joint_id, link_id,
                                                    pos_basejoint_to_basecom,
                                                    rot_basejoint_to_basecom)
        
        sensor_data_b = pybullet_util.get_sensor_data(robotB, joint_id_b, link_id_b,
                                                    pos_basejoint_to_basecom_b,
                                                    rot_basejoint_to_basecom_b)
                                                    
        #################################################################################
        #OBJECT POSITION
                                              
        sensor_data_o = p.getBasePositionAndOrientation(simplebox)
        graphtime.append(count/100)
        object_pos.append(sensor_data_o(2))
        fig, axs = plt.subplots(2,1)
            axs[0].plot(graphtime, object_pos)
            axs[0].set_ylabel('Position (m)')
            axs[0].legend(['x', 'y'])
            axs[0].grid(True)
            
            axs[1].plot(graphtime, object_pos)
            axs[1].set_xlabel('Time (s)')
            axs[1].set_ylabel('Angle (rad)')
            axs[1].legend(['theta'])
            axs[1].grid(True)
            plt.show()
            
            if count == 100
                break
        #################################################################################
        
        rf_height = pybullet_util.get_link_iso(robot, link_id['r_sole'])[2, 3]
        lf_height = pybullet_util.get_link_iso(robot, link_id['l_sole'])[2, 3]
        sensor_data['b_rf_contact'] = True if rf_height <= 0.01 else False
        sensor_data['b_lf_contact'] = True if lf_height <= 0.01 else False
        
        rf_height_b = pybullet_util.get_link_iso(robotB, link_id_b['r_sole'])[2, 3]
        lf_height_b = pybullet_util.get_link_iso(robotB, link_id_b['l_sole'])[2, 3]
        sensor_data_b['b_rf_contact'] = True if rf_height_b <= 0.01 else False
        sensor_data_b['b_lf_contact'] = True if lf_height_b <= 0.01 else False

        # Get Keyboard Event
        keys = p.getKeyboardEvents()
        if pybullet_util.is_key_triggered(keys, '8'):
            interface.interrupt_logic.b_interrupt_button_eight = True
            interface_b.interrupt_logic.b_interrupt_button_eight = True
        elif pybullet_util.is_key_triggered(keys, '5'):
            interface.interrupt_logic.b_interrupt_button_five = True
            interface_b.interrupt_logic.b_interrupt_button_five = True
        elif pybullet_util.is_key_triggered(keys, '4'):
            interface.interrupt_logic.b_interrupt_button_four = True
            interface_b.interrupt_logic.b_interrupt_button_four = True
        elif pybullet_util.is_key_triggered(keys, '2'):
            interface.interrupt_logic.b_interrupt_button_two = True
            interface_b.interrupt_logic.b_interrupt_button_two = True
        elif pybullet_util.is_key_triggered(keys, '6'):
            interface.interrupt_logic.b_interrupt_button_six = True
            interface_b.interrupt_logic.b_interrupt_button_four = True
        elif pybullet_util.is_key_triggered(keys, '7'):
            interface.interrupt_logic.b_interrupt_button_seven = True
            interface_b.interrupt_logic.b_interrupt_button_seven = True
        elif pybullet_util.is_key_triggered(keys, '9'):
            interface.interrupt_logic.b_interrupt_button_nine = True
            interface_b.interrupt_logic.b_interrupt_button_nine = True
        elif pybullet_util.is_key_triggered(keys, '1'):
            interface.interrupt_logic.b_interrupt_button_one = True
            interface_b.interrupt_logic.b_interrupt_button_one = True
        elif pybullet_util.is_key_triggered(keys, '3'):
            interface.interrupt_logic.b_interrupt_button_three = True
            interface_b.interrupt_logic.b_interrupt_button_three = True

        # Compute Command
        if SimConfig.PRINT_TIME:
            start_time = time.time()
        command = interface.get_command(copy.deepcopy(sensor_data))
        command_b = interface_b.get_command(copy.deepcopy(sensor_data_b))

        if SimConfig.PRINT_TIME:
            end_time = time.time()
            print("ctrl computation time: ", end_time - start_time)

        # Apply Trq
        pybullet_util.set_motor_trq(robot, joint_id, command)
        pybullet_util.set_motor_trq(robotB, joint_id_b, command_b)

        p.stepSimulation()

        # time.sleep(dt)
        t += dt
        count += 1
