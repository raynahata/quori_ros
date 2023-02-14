#!/usr/bin/env python3
import roslib
import rospy
import smach
import smach_ros


class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_study'])

    def execute(self, userdata):
        rospy.loginfo('Executing state START - Pausing for 1 sec')
        # sleep for duration
        d = rospy.Duration(1, 0)
        rospy.sleep(d)
        return 'continue_study'


class Consent(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_study', 'exit_study'])
        self.consent_result = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state CONSENT - Pausing for 1 sec')
        # sleep for duration
        d = rospy.Duration(1, 0)
        rospy.sleep(d)
        if self.consent_result == 1:
            rospy.loginfo('CONSENT completed')
            return 'continue_study'
        else:
            rospy.loginfo('CONSENT not completed')
            return 'exit_study'
        

class Training(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_study'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TRAINING - Pausing for 1 sec')
        # sleep for duration
        d = rospy.Duration(1, 0)
        rospy.sleep(d)
        return 'continue_study'

class ExerciseGroup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_study', 'begin_exercise'],
                                    input_keys=['sm_exercise_order', 'sm_set_counter', 'sm_sets_per_exercise'],
                                    output_keys=['sm_current_exercise'])

        
    def execute(self, userdata):
        rospy.loginfo('Executing state EXERCISEGROUP')
        for exercise_index, (exercise, sets_complete) in enumerate(zip(userdata.sm_exercise_order, userdata.sm_set_counter)):
            if sets_complete < userdata.sm_sets_per_exercise:
                userdata.sm_current_exercise = [exercise_index, exercise]
                return 'begin_exercise'
            else:
                rospy.loginfo('Exercise {} complete'.format(exercise))
        
        return 'continue_study'


class Exercise(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end_exercise'],
                                    input_keys=['sm_current_exercise', 'sm_set_counter'],
                                    output_keys=['sm_set_counter'])

    def execute(self, userdata):
        rospy.loginfo('Executing state EXERCISE {} - {}'.format(userdata.sm_current_exercise[0], userdata.sm_current_exercise[1]))
        # sleep for duration
        d = rospy.Duration(1, 0)
        rospy.sleep(d)
        userdata.sm_set_counter[userdata.sm_current_exercise[0]] += 1
        return 'end_exercise'


#Start with this
#Start -> Consent -> Explanation -> Exercise 1 -> Rest -> Repeat 3 -> Done

def main():
    rospy.init_node('main_state_machine')

    # Create the top level SMACH state machine
    sm_main = smach.StateMachine(outcomes=['complete'])
    sm_main.userdata.sm_exercise_order = ['bicep_curls', 'lateral_raises']
    sm_main.userdata.sm_set_counter = [0, 0]
    sm_main.userdata.sm_sets_per_exercise = 3
    sm_main.userdata.sm_rest_between_sets = 30
    sm_main.userdata.sm_rest_between_exercises = 45
    sm_main.userdata.sm_current_exercise = [-1, '']

    # Open the container
    with sm_main:

        smach.StateMachine.add('START', Start(),
                               transitions={'continue_study':'CONSENT'})

        smach.StateMachine.add('CONSENT', Consent(),
                               transitions={'continue_study':'TRAINING', 'exit_study': 'complete'})
        
        smach.StateMachine.add('TRAINING', Training(),
                               transitions={'continue_study':'EXERCISEGROUP'})

        smach.StateMachine.add('EXERCISEGROUP', ExerciseGroup(),
                               transitions={'continue_study':'complete', 'begin_exercise': 'EXERCISE'})

        smach.StateMachine.add('EXERCISE', Exercise(),
                               transitions={'end_exercise': 'EXERCISEGROUP'})
       
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_main, '/')
    sis.start()

    # Execute the state machine
    outcome = sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()