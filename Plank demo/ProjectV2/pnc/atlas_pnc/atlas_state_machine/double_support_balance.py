import numpy as np

from config.atlas_config import WalkingState
from pnc.state_machine import StateMachine
from pnc.planner.locomotion.dcm_planner.footstep import Footstep
from pnc.atlas_pnc.atlas_state_provider import AtlasStateProvider
from util import util


class DoubleSupportBalance(StateMachine):
    def __init__(self, id, tm, hm, fm, robot):
        super().__init__(id, robot)
        self._trajectory_managers = tm
        self._hierarchy_managers = hm
        self._force_managers = fm
        self._sp = AtlasStateProvider()
        self._start_time = 0.
        self._b_state_switch_trigger = False
        self._lhand_task_trans_trigger = False
        self._rhand_task_trans_trigger = False
        ########################################################
        #
        self._moving_duration = 1
        self._end_time=self._start_time + self._moving_duration
        self._trans_duration = 0.
        self._rh_target_pos = np.zeros(3)
        self._lh_target_pos = np.zeros(3)
        #
        ########################################################

    @property
    def b_state_switch_trigger(self):
        return self._b_state_switch_trigger

    @b_state_switch_trigger.setter
    def b_state_switch_trigger(self, val):
        self._b_state_switch_trigger = val

    @property
    def lhand_task_trans_trigger(self):
        return self._lhand_task_trans_trigger

    @lhand_task_trans_trigger.setter
    def lhand_task_trans_trigger(self, value):
        self._lhand_task_trans_trigger = value

    @property
    def rhand_task_trans_trigger(self):
        return self._rhand_task_trans_trigger

    @rhand_task_trans_trigger.setter
    def rhand_task_trans_trigger(self, value):
        self._rhand_task_trans_trigger = value

    def one_step(self):
        self._state_machine_time = self._sp.curr_time - self._start_time
        #########################################################################
        #
        # Update Hierarchy
        self._hierarchy_managers["lhand_pos"].update_ramp_to_max(self._sp.curr_time)
        self._hierarchy_managers["rhand_pos"].update_ramp_to_max(self._sp.curr_time)
        #
        #########################################################################
        # Update Foot Task
        self._trajectory_managers["lfoot"].use_current()
        self._trajectory_managers["rfoot"].use_current()
        #########################################################################
        #
        # Update Hand Task
        #self._trajectory_managers['rhand'].update_hand_trajectory(self._sp.curr_time)
        #self._trajectory_managers['lhand'].update_hand_trajectory(self._sp.curr_time)
        #
        #########################################################################
    def first_visit(self):
        print("[WalkingState] BALANCE")
        self._b_state_switch_trigger = False
        self._start_time = self._sp.curr_time
        #####################################################################################
        #
        target_hand_iso = np.eye(4)
        target_hand_iso[0:3, 3] = self._rh_target_pos
        self._trajectory_managers['rhand'].initialize_hand_trajectory(self._start_time, self._moving_duration, target_hand_iso)
        self._hierarchy_managers["rhand_pos"].initialize_ramp_to_max(self._sp.curr_time, self._trans_duration)
        target_hand_iso = np.eye(4)
        target_hand_iso[0:3, 3] = self._lh_target_pos
        self._trajectory_managers['lhand'].initialize_hand_trajectory(self._start_time, self._moving_duration, target_hand_iso)
        self._hierarchy_managers["lhand_pos"].initialize_ramp_to_max(self._sp.curr_time, self._trans_duration)
        #
        ######################################################################################

    def last_visit(self):
        pass

    def end_of_state(self):
        if (self._b_state_switch_trigger) and (
                len(self._trajectory_managers["dcm"].footstep_list) > 0
        ) and not (self._trajectory_managers["dcm"].no_reaming_steps()):
            return True

        if self._lhand_task_trans_trigger:
            return True
        if self._rhand_task_trans_trigger:
            return True

        return False

    def get_next_state(self):
        b_valid_step, robot_side = self._trajectory_managers[
            "dcm"].next_step_side()
        if b_valid_step:
            if robot_side == Footstep.LEFT_SIDE:
                return WalkingState.LF_CONTACT_TRANS_START
            elif robot_side == Footstep.RIGHT_SIDE:
                return WalkingState.RF_CONTACT_TRANS_START
            else:
                raise ValueError("Wrong Footstep Side")

        #if self._lhand_task_trans_trigger:
        #    return WalkingState.LH_HANDREACH
        #if self._rhand_task_trans_trigger:
        #    return WalkingState.RH_HANDREACH
            
    @property
    def moving_duration(self):
        return self._moving_duration

    @moving_duration.setter
    def moving_duration(self, value):
        self._moving_duration = value

    @property
    def trans_duration(self):
        return self._trans_duration

    @trans_duration.setter
    def trans_duration(self, value):
        self._trans_duration = value

    @property
    def rh_target_pos(self):
        return self._rh_target_pos

    @rh_target_pos.setter
    def rh_target_pos(self, value):
        self._rh_target_pos = value

    @property
    def lh_target_pos(self):
        return self._lh_target_pos

    @lh_target_pos.setter
    def lh_target_pos(self, value):
        self._lh_target_pos = value
