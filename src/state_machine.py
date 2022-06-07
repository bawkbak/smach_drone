#!/usr/bin/env python2
import rospy
import smach
import smach_ros

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Init')
        return 'init_done'
    

class Navigation_exploration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigation', 'path'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Navigation_exploration')
        return 'navigation'

class Navigation_vector_path(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_target', 'reach_desired_height', 'path'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Navigation_vector_path')
        state = 11
        if state > 10:
            rospy.sleep(100)
            return 'reach_desired_height'
        elif state > 9:
            return 'lost_target'

class Navigation_level_path(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_target', 'not_yet', 'reach_landing_position', 'path'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Navigation_level_path')
        state = 11
        if state > 10:
            return 'lost_target'
        elif state > 9:
            return 'not_yet'
        elif state > 8:
            return 'reach_landing_position'

class Navigation_landing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_target', 'not_yet', 'is_land', 'path'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Navigation_landing')
        state = 11
        if state > 10:
            return 'lost_target'
        elif state > 9:
            return 'not_yet'
        elif state > 8:
            return 'is_land'
    

class State_estimation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detect_result', 'localization', 'mapping'])

    def execute(self, userdata):
        rospy.loginfo('Executing state State_estimation')
        state = 11
        if state > 10:
            return 'detect_result'
        elif state > 9:
            return 'localization'
        elif state > 8:
            return 'mapping'

class Path_planner(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cmd_vel'])

    def execute(self, userdata):
        rospy.loginfo('Executing state State_estimation')

        return 'cmd_vel'


class Motion_control(smach.State):
    def __init__(self):
        smach.State.__init__(self)

    def execute(self, userdata):
        rospy.loginfo('Executing state Motion_control')
        return



def main():
    rospy.init_node('smach_state_machine', anonymous=False)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Init', Init(), transitions={'init_done':'Navigation'})
        smach.StateMachine.add('State_estimation', State_estimation(), transitions={'detect_result':'Navigation', 'localization':'Navigation', 'mapping':'Path_planner'})
        smach.StateMachine.add('Path_planner', Path_planner(), transitions={'cmd_vel':'Motion_control'})
        smach.StateMachine.add('Motion_control', Motion_control())
        sm_sub = smach.StateMachine(outcomes=['outcome1', 'outcome2'])
        with sm_sub:
            smach.StateMachine.add('Navigation_exploration', Navigation_exploration(), 
                                    transitions={'navigation':'Navigation_vector_path', 
                                                'path':'outcome2'})
            smach.StateMachine.add('Navigation_vector_path', Navigation_vector_path(), 
                                    transitions={'lost_target':'Navigation_exploration',
                                                'path':'outcome2',
                                                'reach_desired_height':'Navigation_level_path'})
            smach.StateMachine.add('Navigation_level_path', Navigation_level_path(), 
                                    transitions={'lost_target':'Navigation_exploration', 
                                                'not_yet':'Navigation_vector_path',
                                                'path':'outcome2',
                                                'reach_landing_position':'Navigation_landing'})
            smach.StateMachine.add('Navigation_landing', Navigation_landing(), 
                                    transitions={'lost_target':'Navigation_exploration', 
                                                'not_yet':'Navigation_level_path',
                                                'path':'outcome2',
                                                'is_land':'outcome1'})

        smach.StateMachine.add('Navigation', sm_sub, transitions={'outcome1':'end', 'outcome2':'Path_planner'})   

        
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()