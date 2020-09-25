#!/usr/bin/python3
from __future__ import print_function

import rospy
import yaml
import wave
import pyaudio
import rospkg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from threading import Thread


class gps_diagnostics_parser:
    SOUND_PATH = rospkg.RosPack().get_path('vehicle_platform') + "/nodes/gps_diagnostics_parser/sounds/"
    first_message = True
    config = None
    diagnostics_pub = None
    pyaudio = pyaudio.PyAudio()
    playing = False
    sound_count_trigger = 2
    current_errors = []
    error_count = {}

    def __init__(self):
        self.config = yaml.safe_load(open(rospkg.RosPack().get_path('vehicle_platform') + "/config/gps_diagnostics_parser.yaml", 'r'))
        rospy.loginfo(self.__class__.__name__ + " - node started")
        rospy.Subscriber('/gps/diagnostics', DiagnosticArray, self.process_diagnostics_status, queue_size=1)
        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray,  queue_size=5)

    def process_diagnostics_status(self, diagnostic_array):
        """
        :type diagnostic_array: DiagnosticArray
        """
        if "gps/novatel" in diagnostic_array.status[0].name:
            diagnostic_array.status = self.process_gps_status_array(diagnostic_array.status)

        diagnostic_array.header.stamp = rospy.Time.now()
        self.diagnostics_pub.publish(diagnostic_array)

    def process_gps_status_array(self, gps_status_array):
        for i in range(len(gps_status_array)):
            if "GPS Fix" in gps_status_array[i].name:
                gps_status_array[i] = self.process_gps_fix_status(gps_status_array[i], self.config['GPS Fix'])
        return gps_status_array

    def process_gps_fix_status(self, fix_status, config):
        """
        :type fix_status: DiagnosticStatus
        :return:
        """
        valid = True
        message = "Nominal"

        # We only count errors that trigger the notif
        errors_in_last_status = 0
        for key in self.error_count.keys():
            if self.error_count[key] >= self.sound_count_trigger:
                errors_in_last_status += 1

        for status in fix_status.values:
            if self.playing:
                break

            if status.key not in config:
                break

            if self.is_valid(status.value, config[status.key]):
                self.process_valid_gps_fix_status(status)
            else:
                valid, message = self.process_invalid_gps_fix_status(status, config)

        if (errors_in_last_status > 0 and len(self.error_count.keys()) == 0) or (self.first_message and len(self.error_count.keys()) == 0):
            self.play_audio_async("gps_ok.wav")
            self.first_message = False

        fix_status.level = 0 if valid else 1
        fix_status.message = message
        return fix_status

    def process_invalid_gps_fix_status(self, status, config):
        valid = False
        message = config[status.key]['message']
        rospy.logwarn(self.__class__.__name__ + " - " + config[status.key]['message'])

        if status.key in self.error_count.keys():
            self.error_count[status.key] += 1
        else:
            self.error_count[status.key] = 1

        # Playing sound from config
        if 'sound' in config[status.key] and self.error_count[status.key] == self.sound_count_trigger and not self.playing:
            self.play_audio_async(config[status.key]['sound'])

        return valid, message

    def process_valid_gps_fix_status(self, status):
        if status.key in self.error_count.keys():
            del(self.error_count[status.key])

    def is_valid(self, value, config):
        if "exactly" in config.keys():
            return self.get_correct_type(value, config['type']) == self.get_correct_type(config['exactly'], config['type'])
        elif "above" in config.keys():
            return self.get_correct_type(value, config['type']) > self.get_correct_type(config['above'], config['type'])
        elif "below" in config.keys():
            return self.get_correct_type(value, config['type']) < self.get_correct_type(config['below'], config['type'])
        elif "in" in config.keys():
            assert isinstance(config['in'], list), "Not a list"
            correct_list = [self.get_correct_type(item, config['type']) for item in config['in']]
            return self.get_correct_type(value, config['type']) in correct_list

    def get_correct_type(self, value, type):
        if type == "string":
            return str(value)
        elif type == "int":
            return int(value)
        elif type == "float":
            return float(value)
        elif type == "bool":
            # Different rules here because of how YAML integrates bool
            if isinstance(value, str):
                if value in ['yes', 'true', 'True', 'on']:
                    return True
                elif value in ['no', 'false', 'False', 'off']:
                    return False
                raise Exception("'%s' is not a correct boolean value" % value)
            return bool(value)
        raise Exception("This type '%s' is not valid" % type)

    def play_audio_async(self, filename):
        """
        Allows to play sound asynchronously
        :type filename: str
        :return:
        """
        t = Thread(target=self.play_audio, args=(filename,))
        t.start()

    def play_audio(self, filename):
        """
        Allows to play sound
        :type filename: str
        :return:
        """
        self.playing = True
        audio_file = wave.open(self.SOUND_PATH + filename, "rb")

        stream = self.pyaudio.open(
            format=self.pyaudio.get_format_from_width(audio_file.getsampwidth()),
            channels=audio_file.getnchannels(),
            rate=audio_file.getframerate(),
            output=True
        )
        data = audio_file.readframes(audio_file.getframerate())

        while data:
            stream.write(data)
            data = audio_file.readframes(audio_file.getframerate())

        stream.stop_stream()
        stream.close()
        self.playing = False
        return

    @staticmethod
    def run():
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('gps_diagnostics_parser', anonymous=True, log_level=rospy.INFO)
    node = gps_diagnostics_parser()
    node.run()
