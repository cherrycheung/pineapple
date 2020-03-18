#!/usr/bin/env python

import requests
import json
from datetime import datetime
import rospy

api_key = rospy.get_param('/api_key')

# Cheswycke lat lon
lat = 37.565701
lon = -122.003420

url = 'https://api.darksky.net/forecast/%s/%s,%s?units=si' % (api_key, lat, lon)

class DarkSky():
    weather_data = None
    weather_last_time_stamp = None

    def get_all_weather(self):
        if self.weather_last_time_stamp is None:
            self.weather_last_time_stamp = rospy.Time.now()
            self.weather_data = requests.get(url).json()
        else:
            time_since_last_weather_sec = (rospy.Time.now() - self.weather_last_time_stamp).to_sec()
            if time_since_last_weather_sec > 60*60:
                self.weather_data = requests.get(url).json()
                weather_last_time_stamp = rospy.Time.now()

    def get_temperature_now(self):
        return self.weather_data['currently']['temperature']

    def get_todays_summary(self):
        try:
            todays_summary = self.weather_data['hourly']['summary']
        except:
            rospy.loginfo('no hourly information')
            todays_summary = None
        return todays_summary

    def get_weekly_summary(self):
        try:
            weekly_summary = self.weather_data['daily']['summary']
        except:
            rospy.loginfo('no weekly information')
            weekly_summary = None
        return weekly_summary

    def get_current_summary(self):
        return self.weather_data['currently']['summary']

    def get_sunset_time(self):
        return self.weather_data['daily']['data'][0]['sunsetTime']

    def get_sunrise_time(self):
        return self.weather_data['daily']['data'][0]['sunriseTime']
