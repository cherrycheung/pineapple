#!/usr/bin/env python

from main_controller.msg import *
from datetime import datetime
import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String
from random import randint

from sense_hat import SenseHat

g_envpub = None

def get_time_string():
    time_color = (55, 55, 255)
    now = datetime.now()
    current_time = now.strftime("%H:%M")
    return str(current_time)

def publish_state():
    g_envpub.publish(SH.get_humidity())

class Rest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waking up', 'let me rest'])

    def execute(self, userdata):
        # SH.show_on_led(SH.get_humidity(), (232, 121, 121))
        # SH.show_on_led(get_time_string(), (55, 55, 255))
        # last_time_weather_updated = DS.get_all_weather()
        # if last_time_weather_updated is not None:
        #     rospy.loginfo('REST: last update %d min ago' % (DS.get_all_weather()/60))
        # SH.show_on_led(DS.get_todays_summary(), (39, 255, 33))
        # SH.show_on_led('ZEAKKK', (232, 121, 121))
        # publish_state()

        last_time_weather_updated = DS.get_all_weather()
        humidity = SH.get_humidity()
        temperature_outside = DS.get_temperature_now()
        todays_summary = DS.get_todays_summary()
        weekly_summary = DS.get_weekly_summary()
        sunset_time = DS.get_sunset_time()
        sunrise_time = DS.get_sunrise_time()
        sunrise_timestamp = time.strftime("%H:%M", time.localtime(sunrise_time))
        sunset_timestamp = time.strftime("%H:%M", time.localtime(sunset_time))

        rospy.loginfo('**********************************************************************')
        rospy.loginfo('HOME:    humidity:            %s percent' % humidity)
        rospy.loginfo('WEATHER: temperature outside: %s deg C / %s deg F' % (temperature_outside, temperature_outside * 1.8 + 32))
        if todays_summary is not None:
            rospy.loginfo('WEATHER: today\'s weather:     %s' % todays_summary)
        if weekly_summary is not None:
            rospy.loginfo('WEATHER: this week\'s summary: %s' % weekly_summary)
        rospy.loginfo('WEATHER: sunrise time:        %s' % sunrise_timestamp)
        rospy.loginfo('WEATHER: sunset time:         %s' % sunset_timestamp)

        time.sleep(3)

        return 'let me rest'

class Awake(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stay awake', 'need to rest'])

    def execute(self, userdata):
        time.sleep(3)
        return 'need to rest'

def main():
    global SH
    from main_controller.SenseHatEnvironment import SenseHatEnvironment
    SH = SenseHatEnvironment()

    global DS
    from main_controller.DarkSky import DarkSky
    DS = DarkSky()

    global g_envpub
    g_envpub = rospy.Publisher('/environment_publisher', String, queue_size=10)

    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['waking up'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('REST', Rest(),
                transitions={'waking up':'AWAKE', 'let me rest':'REST'})
        smach.StateMachine.add('AWAKE', Awake(),
                transitions={'need to rest':'REST', 'stay awake':'AWAKE'})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
