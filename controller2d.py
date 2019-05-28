#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        # Definición de errores para el PID LONGITUDINAL
        self.err2 = 0
        self.err1 = 0
        self.err = 0
        self.u1 = 0
        self.u = 0

        # Definitions for the Stanley controller
        self.cterr = 0
        self.herr = 0
        self.l = 2.3
        self.steerout = 0
        self.rx = waypoints[0][0]
        self.ry = waypoints[0][1]

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]:
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)

                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here.
            """
            # PID Parameters
            Ts = 0.033 # Sample time
            kp = 0.5 # Proportional Gain (10)
            ki = 0.42 # Integral Gain   (5)
            kd = 0.12 # Derivative Gain (5)

            # Constants calculation
            q0 = kp + ((Ts * ki) / 2) + (kd / Ts)
            q1 = ((Ts * ki) / 2) - kp - ((2 * kd) / Ts)
            q2 = kd / Ts

            # Errors calculation
            self.err2 = self.err1
            self.err1 = self.err
            self.err = v_desired - v

            # Output calculation
            self.u1 = self.u
            self.u = self.u1 + (q0 * self.err) + (q1 * self.err1) + (q2 * self.err2)

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            if (self.u > 0):
                throttle_output = self.u
                brake_output    = 0
            else:
                throttle_output = 0
                brake_output    = -self.u

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER
            ######################################################
            ######################################################
            """
                Implement a lateral controller here.
            """
            # Angle conversion to work within: (0, -pi) or (0, -180)
            yaw = -np.pi - yaw

            # Front axle coordinates calculation
            xf = x + (1.5 * np.cos(yaw))
            yf = y - (1.5 * np.sin(yaw))

            # Distance between the front axle and the closest waypoint
            ex0 = self.rx - xf
            ey0 = self.ry - yf
            # Distance between the front axle and the current waypoint
            ex1 = waypoints[0][0] - xf
            ey1 = waypoints[0][1] - yf

            # Error to the closes waypoint
            de0 = np.sqrt((ex0 ** 2) + (ey0 ** 2))
            # Error to the curret waypoint
            de1 = np.sqrt((ex1 ** 2) + (ey1 ** 2))

            # Conditions to find the cross track error
            if de0 < de1:
                # Keep the closest distance and its keypoints
                self.cterr = de0
            else:
                # Keep the current distance and its keypoints
                self.cterr = de1
                self.rx = waypoints[0][0]
                self.ry = waypoints[0][1]

            # Distance calculation between two consecutive waypoints
            pathangx = waypoints[1][0] - waypoints[0][0] # (Siguiente - Actual)
            pathangy = waypoints[0][1] - waypoints[1][1] # (Siguiente - Actual)

            # Angle calculation between the two keypoints: (0, -180)
            pathang = np.arctan2(pathangy, pathangx) - np.pi
            headerror = (pathang - yaw) * -1
            pathd = pathang * (180 / np.pi)

            # Conditions to determine the correct sign of the cross track error
            if pathd > -170:
                if (ex0 < 0) or (ex1 < 0):
                    self.cterr = -self.cterr
                else:
                    self.cterr = self.cterr
            elif pathd < -190:
                if (ex0 < 0) or (ex1 < 0):
                    self.cterr = self.cterr
                else:
                    self.cterr = -self.cterr
            elif (pathd >= -190) and (pathd <= -170):
                if (ey0 < 0) or (ey1 < 0):
                    self.cterr = -self.cterr
                else:
                    self.cterr = self.cterr

            # Cross track error coefficient
            ksteer = 2 * self.cterr
            corsteer  = np.arctan2(ksteer, v)
            if self.cterr <  0:
                errorcor = - np.absolute(corsteer)
            else:
                errorcor = np.absolute(corsteer)

            # Steering angle calculation
            steer  = headerror + errorcor

            # Maximum and minimum steering angle
            if steer > 1.22:
                steer = 1.22
            elif steer < -1.22:
                steer = -1.22

            steer_output = steer

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Store old values
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
