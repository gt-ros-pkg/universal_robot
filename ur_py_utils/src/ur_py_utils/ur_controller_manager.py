#! /usr/bin/env python

import sys

import rospy
from controller_manager_msgs.srv import *

from arm_iface import CommandMode

class ControllerManager(object):
    def __init__(self, namespace='controller_manager', blocking=True):
        self.namespace = namespace
        self.list_ctrl_types = rospy.ServiceProxy(namespace + '/list_controller_types', 
                                             ListControllerTypes)
        self.reload_ctrl_libs = rospy.ServiceProxy(namespace + '/reload_controller_libraries', 
                                             ReloadControllerLibraries)
        self.list_ctrls = rospy.ServiceProxy(namespace + '/list_controllers', 
                                             ListControllers)
        self.load_ctrl = rospy.ServiceProxy(namespace + '/load_controller', 
                                             LoadController)
        self.switch_ctrl = rospy.ServiceProxy(namespace + '/switch_controller', 
                                             SwitchController)
        self.unload_ctrl = rospy.ServiceProxy(namespace + '/unload_controller', 
                                             UnloadController)
        if blocking:
            self.wait_for_services()

    def wait_for_services(self):
        namespace = self.namespace
        rospy.wait_for_service(namespace + '/list_controller_types')
        rospy.wait_for_service(namespace + '/reload_controller_libraries')
        rospy.wait_for_service(namespace + '/list_controllers')
        rospy.wait_for_service(namespace + '/load_controller')
        rospy.wait_for_service(namespace + '/switch_controller')
        rospy.wait_for_service(namespace + '/unload_controller')

    def list_controller_types(self):
        resp = self.list_ctrl_types.call(ListControllerTypesRequest())
        return resp.types

    def get_controller_names(self):
        resp = self.list_ctrls.call(ListControllersRequest())
        return [c.name for c in resp.controller]

    def reload_libraries(self, force_kill, restore = False):

        print "Restore:", restore
        if restore:
            originally = self.list_ctrls.call(ListControllersRequest())

        resp = self.reload_ctrl_libs.call(ReloadControllerLibrariesRequest(force_kill))
        if resp.ok:
            print "Successfully reloaded libraries"
            result = True
        else:
            print "Failed to reload libraries. Do you still have controllers loaded?"
            result = False

        if restore:
            for c in originally.controllers:
                self.load_ctrl(c)
            to_start = []
            for c, s in zip(originally.controllers, originally.state):
                if s == 'running':
                    to_start.append(c)
            self.switch_ctrl(start_controllers = to_start,
                       stop_controllers = [],
                       strictness = SwitchControllerRequest.BEST_EFFORT)
            print "Controllers restored to original state"
        return result

    def list_controllers(self):
        resp = self.list_ctrls.call(ListControllersRequest())
        if len(resp.controller) == 0:
            print "No controllers are loaded in mechanism control"
        else:
            for c in resp.controller:
                print '%s - %s ( %s )'%(c.name, c.hardware_interface, c.state)

    def load_controller(self, name):
        resp = self.load_ctrl.call(LoadControllerRequest(name))
        if resp.ok:
            print "Loaded", name
            return True
        else:
            print "Error when loading", name
            return False

    def unload_controller(self, name):
        resp = self.unload_ctrl.call(UnloadControllerRequest(name))
        if resp.ok == 1:
            print "Unloaded %s successfully" % name
            return True
        else:
            print "Error when unloading", name
            return False

    def start_controller(self, name):
        return self.start_stop_controllers([name], True)

    def start_controllers(self, names):
        return self.start_stop_controllers(names, True)

    def stop_controller(self, name):
        return self.start_stop_controllers([name], False)

    def stop_controllers(self, names):
        return self.start_stop_controllers(name, False)

    def start_stop_controllers(self, names, st):
        start = []
        stop = []
        strictness = SwitchControllerRequest.STRICT
        if st:
            start = names
        else:
            stop = names
        resp = self.switch_ctrl.call(SwitchControllerRequest(start, stop, strictness))
        if resp.ok == 1:
            if st:
                print "Started %s successfully" % names
            else:
                print "Stopped %s successfully" % names
            return True
        else:
            if st:
                print "Error when starting ", names
            else:
                print "Error when stopping ", names
            return False

class URControllerManager(ControllerManager):
    # VEL_MODE_HW_IFACE = 'ur_controllers::VelocityModeController'
    # PVA_MODE_HW_IFACE = 'ur_controllers::PosVelAccModeController'

    def __init__(self, namespace='controller_manager', blocking=True):
        super(URControllerManager, self).__init__(namespace, blocking)

    # def get_mode_ctrl_states(self):
    #     resp = self.list_ctrls()
    #     running_modes = []
    #     for c in resp.controller:
    #         if c.hardware_interface == VEL_MODE_HW_IFACE:
    #             if c.state == 'running'
    #                 running_modes.append(CommandMode.VEL)
    #         if c.hardware_interface == PVA_MODE_HW_IFACE:
    #             if c.state == 'running'
    #                 running_modes.append(CommandMode.PVA)
    #     if len(running_modes) > 1:
    #         rospy.logerror('Multiple joint command modes running, this should not happen...')
    #         return
    #     elif len(running_modes) == 0:
    #         return CommandMode.EMPTY
    #     return running_modes[0] # should be length 1

    def get_controller_modes(self):
        possible_modes = CommandMode.ids
        joint_controllers = []
        enable_controllers = [(None,None)]*len(possible_modes)
        for i in range(len(possible_modes)):
            joint_controllers.append([])
        resp = self.list_ctrls()
        for c in resp.controller:
            ctrl_name = c.name
            full_mode = rospy.get_param('/'+c.name+'/mode', 'NONE')
            split_mode = full_mode.split('_')
            mode = split_mode[0]
            is_enable = len(split_mode) > 1 and split_mode[1] == 'enable'
            if mode in possible_modes:
                is_running = c.state == 'running'
                modeidx = possible_modes.index(mode)
                if is_enable:
                    if enable_controllers[modeidx][0] is not None:
                        rospy.logerror('Multiple joint command modes running,',
                                       'this should not happen...')
                    enable_controllers[modeidx] = (ctrl_name, is_running)
                else:
                    joint_controllers[modeidx].append((ctrl_name, is_running))
        return joint_controllers, enable_controllers

    def start_joint_controller(self, start_ctrl_name):
        # determine all of the currently running controllers and
        # which mode the new controller is in
        running_controllers = []
        start_ctrl_mode = None
        joint_controllers, enable_controllers = self.get_controller_modes()
        for mode, controller_list in enumerate(joint_controllers):
            for ctrl_name, is_running in controller_list:
                if is_running:
                    running_controllers.append(ctrl_name)
                    if ctrl_name == start_ctrl_name:
                        rospy.logwarn('Starting controller already running, nothing will be done.')
                        return
                else:
                    if ctrl_name == start_ctrl_name:
                        start_ctrl_mode = mode

        if start_ctrl_mode == '':
            # there is no mode associated with this controller, thus we cannot
            # determine a mode conflict, so we should simply start the controller
            return start_controller(start_ctrl_name)

        running_mode_ctrl_name = ''
        start_ctrl_mode_name = ''
        print enable_controllers
        for mode, (ctrl_name, is_running) in enumerate(enable_controllers):
            if is_running:
                running_mode_ctrl_name = ctrl_name
            if mode == start_ctrl_mode:
                start_ctrl_mode_name = ctrl_name
        if start_ctrl_mode_name == '':
            rospy.logerror('Cannot start the mode for this controller since it is not running')
            return False
        
        stop_controllers = running_controllers
        start_controllers = [start_ctrl_name]
        if running_mode_ctrl_name != '' and start_ctrl_mode_name != running_mode_ctrl_name:
            # need to switch mode as well
            stop_controllers.append(running_mode_ctrl_name)
            start_controllers.append(start_ctrl_mode_name)
            
        strictness = SwitchControllerRequest.STRICT
        resp = self.switch_ctrl.call(SwitchControllerRequest(
                        start_controllers, stop_controllers, strictness))
        if resp.ok == 1:
            print "Started %s and stopped %s successfully" % (start_controllers, stop_controllers)
            return True
        else:
            print "Error when starting %s and stopping %s" % (start_controllers, stop_controllers)
            return False

def main():
    import numpy as np
    def print_joint_controller_states():
        joint_controllers, enable_controllers = cman.get_controller_modes()
        print "Joint controllers:"
        for modeidx, mode in enumerate(CommandMode.names):
            print '\t%s:' % mode, joint_controllers[modeidx]
        print
        print "Enable controllers:"
        for modeidx, mode in enumerate(CommandMode.names):
            print '\t%s:' % mode, enable_controllers[modeidx]
    rospy.init_node('ur_controller_manager')
    cman = URControllerManager()
    print cman.get_controller_names()
    joint_controllers, enable_controllers = cman.get_controller_modes()
    all_jnt_ctrls = []
    for mode, ctrl_list in enumerate(joint_controllers):
        all_jnt_ctrls.extend([c[0] for c in ctrl_list])
    print "All joint controllers:", all_jnt_ctrls

    rand_jnt_ctrl = all_jnt_ctrls[np.random.randint(len(all_jnt_ctrls))]

    print_joint_controller_states()
    print 'Switching to random joint controller %s' % rand_jnt_ctrl
    cman.start_joint_controller(rand_jnt_ctrl)
    print_joint_controller_states()

if __name__ == "__main__":
    main()
