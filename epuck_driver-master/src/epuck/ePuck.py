#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# ePuck.py
#
# Copyright 2010 Manuel Martín Ortiz <mmartinortiz@gmail.com>
#
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 3 of the License, or
#       (at your option) any later version.
#
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#
#		-- ePuck.py --
#
#		The aim of this library is to provide access to the ePuck robots
#		through a bluetooth connection. Thus, you can write a program that
#		read from the ePuck's sensors and write in their actuators, This
#		will allow us to create advanced programs that can develop a wide
#		variety of complex tasks. It is necesary that the ePuck has installed
#		the Webot's fimware 1.4.2 or 1.4.3. You can find this fantastic
#		simulator on this site: http://www.cyberbotics.com/
#
#		This library is written in Python 2.6, and you can import it from
#		any program written in Python  (same version or later). In addition
#		to this, you will also need two extra libraries:
#
#			-> Python Bluetooth or Pybluez
#			-> Python Image Library (PIL)
#
#		In this package you will find some examples of how to use this library.
#
#		You may expetience some problems when you work with your ePuck, We
#		recommend you take into consideration the following special
#		characteristic: we use a bluetooth communciation, therefore our bandwith
#		is limited and we cannot expect to do too many tasks in short
#		time; i.e:  If you put the wheels speed to max and want
#		to make a quick process of the images, you will know what I'm saying.
#		So remember, you are processing in your computer, not on the ePuck,
#		and you need to take the sensors data and write on the actuators
#		values on the ePuck
#
#		For further information and updates visit http://abitworld.com/projects

import sys  # System library
import bluetooth  # Used for communications
import time  # Used for image capture process
import struct  # Used for Big-Endian messages
import Image  # Used for the pictures of the camera

__package__ = "ePuck"
__docformat__ = "restructuredtext"

"""
:newfield company: Company
"""

__version__ = "1.2.2"
__author__ = "Manuel Martin Ortiz"
__license__ = "GPL"
__contact__ = ["mmartinortiz@gmail.com"]

# This dictionary have as keys the first character of the message, that
# is used to know the number of lines. If no key for the message, 1 line is assumed
DIC_MSG = {
    "v": 2,  # Version
    "\n": 23,  # Menu
    "\x0c": 2,  # Welcome
    "k": 3,  # Calibration
    "R": 2  # Reset
}

# You have to use the keys of this dictionary for indicate on "enable" function
# the sensor that you want to read
DIC_SENSORS = {
    "accelerometer": "a",
    "selector": "c",
    "motor_speed": "e",
    "camera": "i",
    "floor": "m",
    "proximity": "n",
    "light": "o",
    "motor_position": "q",
    "microphone": "u"
}

# You have to use the keys of this dictionary for indicate the operating
# mode of the camera
CAM_MODE = {
    "GREY_SCALE": 0,
    "RGB_365": 1,
    "YUV": 2,
    "LINEAR_CAM": 3
}

# You can use three diferents Zoom in the camera
CAM_ZOOM = (1, 4, 8)


class ePuck():
    """
    This class represent an ePuck object
    """

    def __init__(self, address, debug=False):
        """
        Constructor process

        :param 	address: Robot's direction in AA:BB:CC:DD:EE:FF format
        :type	address: MAC Address
        :param 	debug: If you want more verbose information, useful for debugging
        :type	debug: Boolean

        :return: ePuck object
        """

        # Monitoring Variables
        self.messages_sent = 0
        self.messages_received = 0
        self.version = __version__
        self.debug = debug

        # Connection Attributes
        self.socket = None
        self.address = address
        self.conexion_status = False

        # Camera attributes
        self._cam_width = None
        self._cam_height = None
        self._cam_enable = False
        self._cam_zoom = None
        self._cam_mode = None
        self._cam_size = None

        # Sensors and actuators lists
        self._sensors_to_read = []
        self._actuators_to_write = []

        # Sensors
        self._accelerometer = (0, 0, 0)
        self._accelerometer_filtered = False
        self._selector = (0)
        self._motor_speed = (0, 0)  # left and right motor
        self._motor_position = (0, 0)  # left and right motor
        self._camera_parameters = (0, 0, 0, 0)
        self._floor_sensors = (0, 0, 0)
        self._proximity = (0, 0, 0, 0, 0, 0, 0, 0)
        self._light_sensor = (0, 0, 0, 0, 0, 0, 0, 0)
        self._microphone = (0, 0, 0)
        self._pil_image = None

        # Leds
        self._leds_status = [False] * 10

    #
    # Private methods
    #
    def _debug(self, *txt):
        """
        Show debug information and data, only works if debug information
        is enable (see "set_debug()")

        :param 	txt: Data to be showed separated by comma
        :type	txt: Any
        """

        if self.debug:
            print >> sys.stderr, '\033[31m[ePuck'+str(self.address)+']:\033[0m ', ' '.join([str(e) for e in txt])

        return 0

    def _recv(self, n=4096):
        """
        Receive data from the robot

        :param	n: 	Number of bytes you want to receive
        :type	n: 	int
        :return: 	Data received from the robot as string if it was successful, raise an exception if not
        :rtype:		String
        :raise Exception:	If there is a communication problem
        """
        if not self.conexion_status:
            raise Exception, 'There is not connection'

        try:
            line = self.socket.recv(n)
            self.messages_received += 1
        except bluetooth.btcommon.BluetoothError, e:
            txt = 'Bluetooth communication problem: ' + str(e)
            self._debug(txt)
            raise Exception, txt
        else:
            return line

    def _send(self, message):
        """
        Send data to the robot

        :param	message: Message to be sent
        :type	message: String
        :return: Number of bytes sent if it was successful. -1 if not
        :rtype:	int
        """
        if not self.conexion_status:
            raise Exception, 'There is not connection'

        try:
            n = self.socket.send(message)
            self.messages_sent += 1
        except Exception, e:
            self._debug('Send problem:', e)
            return -1
        else:
            return n

    def _read_image(self):
        """
        Returns an image obtained from the robot's camera. For communication
        issues you only can get 1 image per second

        :return: The image in PIL format
        :rtype: PIL Image
        """

        # Thanks to http://www.dailyenigma.org/e-puck-cam.shtml for
        # the code for get the image from the camera
        msg = struct.pack(">bb", - ord("I"), 0)

        try:
            n = self._send(msg)
            self._debug("Reading Image: sending " + repr(msg) + " and " + str(n) + " bytes")

            # We have to add 3 to the size, because with the image we
            # get "mode", "width" and "height"
            size = self._cam_size + 3
            img = self._recv(size)
            while len(img) != size:
                img += self._recv(size)

            # Create the PIL Image
            image = Image.frombuffer("RGB", (self._cam_width, self._cam_height),
                                     img, "raw",
                                     "BGR;16", 0, 1)

            image = image.rotate(180)
            self._pil_image = image

        except Exception, e:
            self._debug('Problem receiving an image: ', e)

    def _refresh_camera_parameters(self):
        """
        Method for refresh the camera parameters, it's called for some
        private methods
        """
        try:
            msg = self.send_and_receive("I").split(',')
        except:
            return False
        else:
            self._cam_mode, \
            self._cam_width, \
            self._cam_height, \
            self._cam_zoom, \
            self._cam_size = [int(i) for i in msg[1:6]]

            self._camera_parameters = self._cam_mode, self._cam_width, self._cam_height, self._cam_zoom

    def _write_actuators(self):
        """
        Write in the robot the actuators values. Don't use directly,
        instead use 'step()'
        """

        # Not all messages reply with AKC, only Ascii messages
        acks = ['j', 't']

        # We make a copy of the actuators list
        actuators = self._actuators_to_write[:]

        for m in actuators:
            if m[0] == 'L':
                # Leds
                msg = struct.pack('<bbb', - ord(m[0]), m[1], m[2])
                n = self._send(msg)
                self._debug('Binary message sent of [' + str(n) + '] bytes: ' + str(struct.unpack('<bbb', msg)))

            elif m[0] == 'D' or m[0] == 'P':
                # Set motor speed or set motor position
                msg = struct.pack('<bhh', - ord(m[0]), m[1], m[2])
                n = self._send(msg)
                self._debug('Binary message sent of [' + str(n) + '] bytes: ' + str(struct.unpack('<bhh', msg)))

            else:
                # Others actuators, parameters are separated by commas
                msg = ",".join(["%s" % i for i in m])
                reply = self.send_and_receive(msg)
                if reply == 'j':
                    self._refresh_camera_parameters()

                if reply not in acks:
                    self._debug('Unknown ACK reply from ePcuk: ' + reply)

            self._actuators_to_write.remove(m)
        return

    def _read_sensors(self):
        """
        This method is used for read the ePuck's sensors. Don't use directly,
        instead use 'step()'
        """

        # We can read sensors in two ways: Binary Mode and Ascii Mode
        # Ascii mode is slower than Binary mode, therefore, we use
        # Binary mode whenever we can. Not all sensors are available in
        # Binary mode

        def send_binary_mode(parameters):
            # Auxiliar function for sent messages in binary modes
            # Parameters: ('Char to be sent', 'Size of reply waited', 'Format of the teply')

            self._debug('Sending binary message: ', ','.join('%s' % i for i in parameters))
            message = struct.pack(">bb", - ord(parameters[0]), 0)
            self._send(message)
            reply = ()
            try:
                reply = self._recv()
                while len(reply) < parameters[1]:
                    reply += self._recv()
                self._debug('Binary message recived: ', reply)
                reply = struct.unpack(parameters[2], reply) # "reply" must contain the exact number of bytes requested by the format in "paramaters[2]".

            except Exception, e:
                if str(e) == "Bluetooth communication problem: timed out":
                    self._debug("Received " + str(len(reply)) + " of " + str(parameters[1]) + " bytes")
                    return 0
                else:
                    raise e

            return reply

        # Read differents sensors
        for s in self._sensors_to_read:

            if s == 'a':
                # Accelerometer sensor in a non filtered way
                if self._accelerometer_filtered:
                    parameters = ('A', 12, '@III')

                else:
                    parameters = ('a', 6, '@HHH')

                reply = send_binary_mode(parameters)
                if type(reply) is tuple and type(reply[0]) is int:
                    self._accelerometer = reply

            elif s == 'n':
                # Proximity sensors
                parameters = ('N', 16, '@HHHHHHHH')
                reply = send_binary_mode(parameters)
                if type(reply) is tuple and type(reply[0]) is int:
                    self._proximity = reply
                    #print("prox: " + str(reply[0]) + ", " + str(reply[1]) + ", " + str(reply[2]) + ", " + str(reply[3]) + ", " + str(reply[4]) + ", " + str(reply[5]) + ", " + str(reply[6]) + ", " + str(reply[7]))
            elif s == 'm':
                # Floor sensors
                parameters = ('M', 10, '@HHHHH')
                reply = send_binary_mode(parameters)
                if type(reply) is tuple and type(reply[0]) is int:
                    self._floor_sensors = reply

            elif s == 'q':
                # Motor position sensor
                parameters = ('Q', 4, '@HH')
                reply = send_binary_mode(parameters)
                if type(reply) is tuple and type(reply[0]) is int:
                    self._motor_position = reply

            elif s == 'o':
                # Light sensors
                parameters = ('O', 16, '@HHHHHHHH')
                reply = send_binary_mode(parameters)
                if type(reply) is tuple and type(reply[0]) is int:
                    self._light_sensor = reply

            elif s == 'u':
                # Microphone
                parameters = ('u', 6, '@HHH')
                reply = send_binary_mode(parameters)
                if type(reply) is tuple and type(reply[0]) is int:
                    self._microphone = reply

            elif s == 'e':
                # Motor Speed
                parameters = ('E', 4, '@HH')
                reply = send_binary_mode(parameters)
                if type(reply) is tuple and type(reply[0]) is int:
                    self._motor_speed = reply

            elif s == 'i':
                # Do nothing for the camera, is an independent process
                pass

            else:
                reply = self.send_and_receive(s).split(",")

                t = reply[0]
                response = tuple(reply[1:len(reply)])

                if t == "c":
                    # Selector
                    self._selector = response[0]

                else:
                    self._debug('Unknow type of sensor to read' + str(reply))


    #
    # Public methods
    #

    def connect(self):
        """
        Connect with the physic ePuck robot

        :return: If the connexion was succesful
        :rtype: Boolean
        :except Exception: If there are a communication proble, for example, the robot is off
        """

        if self.conexion_status:
            self._debug('Already connected')
            return False
        try:
            self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.socket.connect((self.address, 1))
            self.socket.settimeout(0.5)

        except Exception, e:
            txt = 'Connection problem: \n' + str(e)
            self._debug(txt)
            raise Exception, txt

        self.conexion_status = True
        self._debug("Connected")

        self.reset()
        self.clean_recv_buffer()

        return True

    def disconnect(self):
        """
        Disconnect from ePuck robot. Same as 'close()'
        """

        self.close()

    def close(self):
        """
        Close the connection with the robot. Same as 'disconnect()'

        :return: 0 if all ok
        :rtype: int
        :raise Exception: if it was a problem closing the connection
        """

        if self.conexion_status:
            try:
                # Stop the robot
                self.stop()

                # Close the socket
                self.socket.close()
                self.conexion_status = False
            except Exception, e:
                raise Exception, 'Closing connection problem: \n' + str(e)
            else:
                return 0

    def set_debug(self, debug):
        """
        Set / unset debug information
        :param debug: True or False, as you want or not Debug information
        :type debug: Boolean
        """

        self.debug = debug

    def send_and_receive(self, msg):
        """
        Send an Ascii message to the robot and return the reply. You can
        use it, but I don't recommend, use 'enable()', 'disable()'
        and 'step()' instead

        :param msg: The message you want to send
        :type msg:	String
        :return: Response of the robot
        :rtype: String
        """

        # Check the connection
        if not self.conexion_status:
            raise Exception, 'There is not connection'

        # Make sure the Message is a string
        message = str(msg)

        # Add carriage return if not
        if not message.endswith('\n'):
            message += '\n'

        # Check the lines of the waited reply
        if message[0] in DIC_MSG:
            lines = DIC_MSG[message[0]]
        else:
            lines = 1
        self._debug('Waited lines:', lines)

        # We make 5 tries before desist
        tries = 1
        while tries < 5:
            # Send the message
            bytes = self._send(message)
            self._debug('Message sent:', repr(message))
            self._debug('Bytes sent:', bytes)

            try:
                # Receive the reply. As we want to receive a line, we have to insist
                reply = ''
                while reply.count('\n') < lines:
                    reply += self._recv()
                    if message[0] == 'R':
                        # For some reason that I don't understand, if you send a reset
                        # command 'R', sometimes you recive 1 or 2 lines of 'z,Command not found\r\n'
                        # Therefor I have to remove it from the expected message: The Hello message
                        reply = reply.replace('z,Command not found\r\n', '')
                self._debug('Message received: ', reply)
                return reply.replace('\r\n', '')

            except Exception, e:
                tries += 1
                self._debug('Communication timeout, retrying')


    def save_image(self, name='ePuck.jpg'):
        """
        Save image from ePuck's camera to disk

        :param name: Image name, ePuck.jpg as default
        :type name: String

        :return: Operation result
        :rtype:  Boolean
        """

        if self._pil_image:
            return self._pil_image.save(name)
        else:
            return False

    def get_accelerometer(self):
        """
        Return Accelerometer values in (x, y, z)

        :return: Accelerometer values
        :rtype: Tuple
        """
        return self._accelerometer

    def get_selector(self):
        """
        Return the selector position (0-15)

        :return: Selector value
        :rtype: int
        """
        return self._selector

    def get_motor_speed(self):
        """
        Return the motor speed. Correct values are in the range [-1000, 1000]

        :return: Motor speed
        :rtype: Tuple
        """
        return self._motor_speed

    def get_camera_parameters(self):
        """
        Return the camera parameters as a tuple
        (mode, width, height, zoom)

        :return: Camera parameters
        :rtype: Tuple
        """
        return self._camera_parameters

    def get_floor_sensors(self):
        """
        Return the floor sensors values as (left, center, right)

        :return: Floor sensors values
        :rtype: Tuple
        """
        return self._floor_sensors

    def get_proximity(self):
        """
        Return the values of the 8 proximity sensors

        :return: Proximity sensors values
        :rtype: Tuple
        """
        return self._proximity

    def get_light_sensor(self):
        """
        Return the value of the light sensor

        :return: Ligth sensor value
        :rtype: Tuple
        """
        return self._light_sensor

    def get_motor_position(self):
        """
        Return the position of the left and right motor as a tuple

        :return: Motor position
        :rtype: Tuple
        """
        return self._motor_position

    def get_microphone(self):
        """
        Return the volume of the three microphones

        :return: Microphones values
        :rtype: Tuple
        """
        return self._microphone

    def is_connected(self):
        """
        Return a boolean value that indicate if the robot is connected to the PC

        :return: If the robot is connected to the PC
        :rtype: Boolean
        """
        return self.conexion_status

    def get_image(self):
        """
        Return the last image captured from the ePuck's camera (after a 'step()').
        None if	there are not images captured. The image is an PIL object

        :return: Image from robot's camera
        :rtype: PIL
        """
        return self._pil_image

    def get_sercom_version(self):
        """
        :return: Return the ePuck's firmware version
        :rtype: String
        """
        return self.send_and_receive("v")

    def set_accelerometer_filtered(self, filter=False):
        """
        Set filtered way for accelerometer, False is default value
        at the robot start

        :param filter: True or False, as you want
        :type filter: Boolean
        """
        self._accelerometer_filtered = filter

    def disable(self, *sensors):
        """
        Sensor(s) that you want to get disable in the ePuck

        :param sensors: Name of the sensors, take a look to DIC_SENSORS. Multiple sensors can be separated by commas
        :type sensors: String
        :return: Sensors enabled
        :rtype: List
        :except Exception: Some wrong happened
        """
        for sensor in sensors:
            try:
                if not DIC_SENSORS.has_key(sensor):
                    self._debug('Sensor "' + sensor + '" not in DIC_SENSORS')
                    break

                if sensor == "camera":
                    self._cam_enable = False

                if DIC_SENSORS[sensor] in self._sensors_to_read:
                    l = list(self._sensors_to_read)
                    l.remove(DIC_SENSORS[sensor])
                    self._sensors_to_read = tuple(l)
                    self._debug('Sensor "' + sensor + '" disabled')
                else:
                    self._debug('Sensor "' + sensor + '" alrady disabled')

            except Exception, e:
                self._debug('Something wrong happened to disable the sensors: ', e)

        return self.get_sensors_enabled()

    def enable(self, *sensors):
        """
        Sensor(s) that you want to get enable in the ePuck

        :param sensors: Name of the sensors, take a look to DIC_SENSORS. Multiple sensors can be separated by commas
        :type sensors: String
        :return: Sensors enabled
        :rtype: List
        :except Exception: Some wrong happened
        """

        # Using the * as a parameters, we get a tuple with all sensors
        for sensor in sensors:
            try:
                if not DIC_SENSORS.has_key(sensor):
                    self._debug('Sensor "' + sensor + '" not in DIC_SENSORS')
                    break

                if sensor == "camera":
                    # If the sensor is the Camera, then we refresh the
                    # camera parameters
                    if not self._cam_enable:
                        try:
                            self._refresh_camera_parameters()
                            self._cam_enable = True
                            self.timestamp = time.time()
                        except:
                            break

                if DIC_SENSORS[sensor] not in self._sensors_to_read:
                    l = list(self._sensors_to_read)
                    l.append(DIC_SENSORS[sensor])
                    self._sensors_to_read = tuple(l)
                    self._debug('Sensor "' + sensor + '" enabled')
                else:
                    self._debug('Sensor "' + sensor + '" alrady enabled')

            except Exception, e:
                self._debug('Something wrong happened to enable the sensors: ', e)
        return self.get_sensors_enabled()

    def get_sensors_enabled(self):
        """
        :return: Return a list of sensors thar are active
        :rtype: List
        """
        l = []
        for sensor in DIC_SENSORS:
            if DIC_SENSORS[sensor] in self._sensors_to_read:
                l.append(sensor)
        return l

    def set_motors_speed(self, l_motor, r_motor):
        """
        Set the motors speed. The MAX and MIN speed of the ePcuk is [-1000, 1000]

        :param l_motor: Speed of left motor
        :type l_motor: int
        :param r_motor: Speed of right motor
        :type r_motor: int
        """

        # I don't check the MAX and MIN speed because this check
        # will be made by the ePuck's firmware. Here we need speed
        # and we lose time mading recurrent chekings

        self._actuators_to_write.append(("D", int(l_motor), int(r_motor)))

        return True

    def set_motor_position(self, l_wheel, r_wheel):
        """
        Set the motor position, useful for odometry

        :param l_wheel: left wheel
        :type l_wheel: int
        :param r_wheel: right wheel
        :type r_wheel: int
        """

        self._actuators_to_write.append(("P", l_wheel, r_wheel))

    def set_led(self, led_number, led_value):
        """
        Turn on/off the leds

        :param led_number: If led_number is other than 0-7, all leds are set to the indicated value.
        :type led_number: int
        :param led_value:
            - 0 : Off
            - 1 : On (Red)
            - 2 : Inverse
        :type led_value: int
        """

        led = abs(led_number)
        value = abs(led_value)

        if led < 9:
            self._actuators_to_write.append(("L", led, value))
            if value == 0:
                self._leds_status[led] = False
            elif value == 1:
                self._leds_status[led] = True
            else:
                self._leds_status[led] = not self._leds_status[led]
            return True
        else:
            return False

    def set_body_led(self, led_value):
        """
        Turn on /off the body led

        :param led_value:
            - 0 : Off
            - 1 : On (green)
            - 2 : Inverse
        :type led_value: int
        """

        value = abs(led_value)

        self._actuators_to_write.append(("L", 8, value))

        if value == 0:
            self._leds_status[8] = False
        elif value == 1:
            self._leds_status[8] = True
        else:
            self._leds_status[8] = not self._leds_status[8]

        return True

    def set_front_led(self, led_value):
        """
        Turn on /off the front led

        :type	led_value: int
        :param 	led_value:
            - 0 : Off
            - 1 : On (green)
            - 2 : Inverse
        """
        value = abs(led_value)

        self._actuators_to_write.append(("L", 9, value))

        if value == 0:
            self._leds_status[9] = False
        elif value == 1:
            self._leds_status[9] = True
        else:
            self._leds_status[9] = not self._leds_status[9]

        return True

    def set_sound(self, sound):
        """
        Reproduce a sound

        :param sound: Sound in the range [1,5]. Other for stop
        :type sound: int
        """

        self._actuators_to_write.append(("T", sound))
        return True

    def set_camera_parameters(self, mode, width, height, zoom):
        """
        Set the camera parameters

        :param mode: GREY_SCALE, LINEAR_CAM, RGB_365, YUM
        :type  mode: String
        :param width: Width of the camera
        :type  width: int
        :param height: Height of the camera
        :type  height: int
        :param zoom: 1, 4, 8
        :type  zoom: int
        """

        if mode in CAM_MODE:
            self._cam_mode = CAM_MODE[mode]
        else:
            # self._debug(ERR_CAM_PARAMETERS, "Camera mode")
            return -1

        if int(zoom) in CAM_ZOOM:
            self._cam_zoom = zoom
        else:
            # self._debug(ERR_CAM_PARAMETERS, "Camera zoom")
            return -1

        if self.conexion_status and int(width) * int(height) <= 1600:
            # 1600 are for the resolution no greater than 40x40, I have
            # detect some problems
            self._actuators_to_write.append(("J",
                                             self._cam_mode,
                                             width,
                                             height,
                                             self._cam_zoom))

            self._debug(self.conexion_status)

            return 0

    def calibrate_proximity_sensors(self):
        """
        Calibrate proximity sensors, keep off any object in 10 cm

        :return: Successful operation
        :rtype: Boolean
        """

        reply = self.send_and_receive("k", tries_timeout=25)
        if reply[1] == "k":
            return True
        else:
            return False

    def reset(self):
        """
        Reset the robot

        :return: Successful operation
        :rtype: Boolean
        :raise Exception: If there is not connection
        """
        if not self.conexion_status:
            raise Exception, 'There is not connection'

        msg = self.send_and_receive("R")
        self._debug(msg)

        return True

    def stop(self):
        """
        Stop the motor and turn off all leds
        :return: Successful operation
        :rtype: Boolean
        :raise Exception: If there is not connection
        """

        if not self.conexion_status:
            raise Exception, 'There is not connection'

        reply = self.send_and_receive("S")
        self._debug(reply)

        if reply == "s":
            return True
        else:
            return False

    def step(self):
        """
        Method to update the sensor readings and to reflect changes in
        the actuators. Before invoking this method is not guaranteed
        the consistency of the sensors
        """

        if not self.conexion_status:
            raise Exception, 'There is not connection'

        self._write_actuators()
        self._read_sensors()

        # Get an image in 1 FPS
        if self._cam_enable and time.time() - self.timestamp > 1:
            self._read_image()
            self.timestamp = time.time()


    def clean_recv_buffer(self):
        """
        Clean the receiving buffer on connection in order to be sure the next commands answers are
        received well.

        :return: Successful operation
        :rtype: Boolean
        """

        # Request the current protocol version and read until no more characters are received;
        # do not take in consideration the answer since it can be wrong.
        message = "v\n"
        bytes = self._send(message)
        self._debug('Message sent:', repr(message))
        self._debug('Bytes sent:', bytes)
        reply = self._recv()
        self._debug('Message received: ', reply)
        try:
            while len(reply) > 0:
                reply = self._recv()
                self._debug('Message received: ', reply)
        except Exception, e:
            if str(e) == "Bluetooth communication problem: timed out":
                self._debug('Communication timeout, buffer cleaned')
            else:
                raise e

        return True

