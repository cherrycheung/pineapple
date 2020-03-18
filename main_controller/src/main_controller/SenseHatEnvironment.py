#!/usr/bin/env python

from sense_hat import SenseHat

try:
    g_sh = SenseHat()
    g_sh.clear()
    g_sh.low_light = True
except:
    g_sh = None

class SenseHatEnvironment():
    def show_on_led(self, message, color):
        if g_sh is not None and message != '' and message is not None:
            g_sh.show_message(message, scroll_speed=0.1, text_colour=color)

    def get_humidity(self):
        if g_sh is not None:
            humidity_percentage = ("%.0f%%" % g_sh.get_humidity())
            return str(humidity_percentage)
        return ''
