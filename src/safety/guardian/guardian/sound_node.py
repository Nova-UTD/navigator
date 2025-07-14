'''
Package:   guardian
Filename:  sound_node.py
Author:    Will Heitman (w at heit.mn)

Simple node that listens for safety events and plays matching audio alerts.
'''
from navigator_msgs.msg import Mode
import rclpy
from rclpy.node import Node
import simpleaudio
from std_msgs.msg import Bool
import os
import time


class sound_node(Node):

    def __init__(self):
        super().__init__('sound_node')

        self.current_mode = Mode.DISABLED

        self.doPlayAlert = False
        self.doPlayWaitingSound = False

        sound_dir = "/home/nova/navigator/data/sounds"

        self.alert_wav = simpleaudio.WaveObject.from_wave_file(os.path.join(sound_dir, "alert.wav"))
        self.alert_play_obj = None

        self.waiting_wav = simpleaudio.WaveObject.from_wave_file(os.path.join(sound_dir, "waiting_loop.wav"))
        self.waiting_play_obj = None
        self.waiting_done_wav = simpleaudio.WaveObject.from_wave_file(os.path.join(sound_dir, "waiting_done.wav"))

        self.auto_on_wav = simpleaudio.WaveObject.from_wave_file(os.path.join(sound_dir, "auto_on.wav"))
        self.teleop_on_wav = simpleaudio.WaveObject.from_wave_file(os.path.join(sound_dir, "teleop_on.wav"))
        self.disable_wav = simpleaudio.WaveObject.from_wave_file(os.path.join(sound_dir, "disable.wav"))

        current_mode_sub = self.create_subscription(Mode, "/guardian/mode", self.currentModeCb, 1)
        alert_sound_timer = self.create_timer(1.0, self.playAlert)

        waiting_sound_timer = self.create_timer(2.5, self.playWaitingSound)
        is_waiting_sub = self.create_subscription(Bool, "/planning/is_waiting", self.isWaitingCb, 1)

    def isWaitingCb(self, msg: Bool):
        is_waiting = msg.data


        if not is_waiting and self.doPlayWaitingSound:
            self.doPlayWaitingSound = False
            self.waiting_done_wav.play()
        elif is_waiting:
            print("Is waiting")
            # self.doPlayWaitingSound = True

        # self.doPlayWaitingSound = False



    def playWaitingSound(self):
        if not self.doPlayWaitingSound:
            return

        self.waiting_play_obj = self.waiting_wav.play()

    def playAlert(self):
        if not self.doPlayAlert:
            return

        if self.alert_play_obj is None or not self.alert_play_obj.is_playing():
            self.alert_play_obj = self.alert_wav.play()

    def currentModeCb(self, msg: Mode):

        if msg.mode == self.current_mode:
            return # no change.
        
        self.current_mode = msg.mode
        
        if msg.mode == Mode.AUTO:
            self.auto_on_wav.play()
        elif msg.mode == Mode.MANUAL:
            self.teleop_on_wav.play()
        elif msg.mode == Mode.DISABLED:
            print("DISABLED")
            self.disable_wav.play() 
        


def main(args=None):
    rclpy.init(args=args)
    guardian = sound_node()
    rclpy.spin(guardian)
    sound_node.destroy_node()
    rclpy.shutdown()
