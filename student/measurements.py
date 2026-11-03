# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Classes for sensor and measurement
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys

PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(
    os.path.realpath(
        os.path.join(
            os.getcwd(),
            os.path.expanduser(__file__)
        )
    )
)

sys.path.append(
    os.path.normpath(
        os.path.join(
            SCRIPT_DIR,
            PACKAGE_PARENT
        )
    )
)

import misc.params as params


class Sensor:
    '''Sensor class including measurement matrix'''

    def __init__(self, name, calib):
        self.name = name

        if name == 'lidar':
            self.dim_meas = 3

            # Lidar detections are already expressed in vehicle coordinates.
            self.sens_to_veh = np.matrix(
                np.identity(4)
            )

            # Horizontal field of view in radians.
            self.fov = [
                -np.pi / 2,
                np.pi / 2
            ]

        elif name == 'camera':
            self.dim_meas = 2

            # Transformation from camera sensor coordinates
            # to vehicle coordinates.
            self.sens_to_veh = np.matrix(
                calib.extrinsic.transform
            ).reshape(
                4,
                4
            )

            # Camera intrinsic parameters.
            self.f_i = calib.intrinsic[0]
            self.f_j = calib.intrinsic[1]
            self.c_i = calib.intrinsic[2]
            self.c_j = calib.intrinsic[3]

            # Horizontal camera field of view in radians.
            self.fov = [
                -0.35,
                0.35
            ]

        else:
            raise ValueError(
                'Unknown sensor type: {}'.format(name)
            )

        # Transformation from vehicle coordinates to sensor coordinates.
        self.veh_to_sens = np.linalg.inv(
            self.sens_to_veh
        )

    def in_fov(self, x):
        # check if an object x can be seen by this sensor
        ############
        # TODO Step 4: implement a function that returns True if x lies in the sensor's field of view,
        # otherwise False.
        ############

        # Create homogeneous position vector in vehicle coordinates.
        pos_veh = np.ones(
            (4, 1)
        )

        pos_veh[0:3] = x[0:3]

        # Transform vehicle position into sensor coordinates.
        pos_sens = (
            self.veh_to_sens
            * pos_veh
        )

        # In the sensor coordinate system, the first coordinate
        # represents the forward/depth direction.
        depth = float(
            pos_sens[0, 0]
        )

        lateral = float(
            pos_sens[1, 0]
        )

        # Objects behind the sensor cannot be observed.
        if depth <= 0.0:
            return False

        # Horizontal bearing angle relative to the sensor.
        alpha = np.arctan2(
            lateral,
            depth
        )

        return (
            self.fov[0]
            <= alpha
            <= self.fov[1]
        )

        ############
        # END student code
        ############

    def get_hx(self, x):
        # calculate nonlinear measurement expectation value h(x)

        if self.name == 'lidar':
            pos_veh = np.ones(
                (4, 1)
            )

            pos_veh[0:3] = x[0:3]

            # Transform from vehicle to lidar coordinates.
            pos_sens = (
                self.veh_to_sens
                * pos_veh
            )

            return pos_sens[0:3]

        elif self.name == 'camera':

            ############
            # TODO Step 4: implement nonlinear camera measurement function h:
            # - transform position estimate from vehicle to camera coordinates
            # - project from camera to image coordinates
            # - make sure to not divide by zero, raise an error if needed
            # - return h(x)
            ############

            # Create homogeneous position vector in vehicle coordinates.
            pos_veh = np.ones(
                (4, 1)
            )

            pos_veh[0:3] = x[0:3]

            # Transform position from vehicle to camera coordinates.
            pos_sens = (
                self.veh_to_sens
                * pos_veh
            )

            depth = float(
                pos_sens[0, 0]
            )

            # Projection is undefined when the depth is zero.
            if abs(depth) < 1e-6:
                raise NameError(
                    'Camera projection not defined for this x!'
                )

            # Project the 3D camera position into image coordinates.
            #
            # i = c_i - f_i * y / x
            # j = c_j - f_j * z / x
            i = (
                self.c_i
                - self.f_i
                * float(pos_sens[1, 0])
                / depth
            )

            j = (
                self.c_j
                - self.f_j
                * float(pos_sens[2, 0])
                / depth
            )

            return np.matrix(
                [
                    [i],
                    [j]
                ]
            )

            ############
            # END student code
            ############

    def get_H(self, x):
        # calculate Jacobian H at current x from h(x)

        H = np.matrix(
            np.zeros(
                (
                    self.dim_meas,
                    params.dim_state
                )
            )
        )

        # Rotation from vehicle to sensor coordinates.
        R = self.veh_to_sens[
            0:3,
            0:3
        ]

        # Translation from vehicle to sensor coordinates.
        T = self.veh_to_sens[
            0:3,
            3
        ]

        if self.name == 'lidar':
            H[
                0:3,
                0:3
            ] = R

        elif self.name == 'camera':

            # Position in camera coordinates.
            x_sens = (
                R[0, 0] * x[0]
                + R[0, 1] * x[1]
                + R[0, 2] * x[2]
                + T[0]
            )

            y_sens = (
                R[1, 0] * x[0]
                + R[1, 1] * x[1]
                + R[1, 2] * x[2]
                + T[1]
            )

            z_sens = (
                R[2, 0] * x[0]
                + R[2, 1] * x[1]
                + R[2, 2] * x[2]
                + T[2]
            )

            # Camera projection and Jacobian are undefined at zero depth.
            if abs(float(x_sens)) < 1e-6:
                raise NameError(
                    'Jacobian not defined for this x!'
                )

            # Derivatives of:
            #
            # i = c_i - f_i * y_sens / x_sens
            # j = c_j - f_j * z_sens / x_sens
            for column in range(3):
                H[0, column] = self.f_i * (
                    -R[1, column] / x_sens
                    + R[0, column]
                    * y_sens
                    / (x_sens ** 2)
                )

                H[1, column] = self.f_j * (
                    -R[2, column] / x_sens
                    + R[0, column]
                    * z_sens
                    / (x_sens ** 2)
                )

        return H

    def generate_measurement(
        self,
        num_frame,
        z,
        meas_list
    ):
        # generate new measurement from this sensor and add to measurement list
        ############
        # TODO Step 4: remove restriction to lidar in order to include camera as well
        ############

        # Measurement.__init__ handles the sensor-specific
        # initialization for lidar and camera.
        meas = Measurement(
            num_frame,
            z,
            self
        )

        meas_list.append(
            meas
        )

        return meas_list

        ############
        # END student code
        ############


###################


class Measurement:
    '''Measurement class including measurement values, covariance,
    timestamp, sensor'''

    def __init__(
        self,
        num_frame,
        z,
        sensor
    ):
        # create measurement object

        self.t = (
            num_frame - 1
        ) * params.dt

        if sensor.name == 'lidar':
            sigma_lidar_x = params.sigma_lidar_x
            sigma_lidar_y = params.sigma_lidar_y
            sigma_lidar_z = params.sigma_lidar_z

            # Lidar measurement vector:
            #
            # z = [x, y, z]^T
            self.z = np.matrix(
                np.zeros(
                    (
                        sensor.dim_meas,
                        1
                    )
                )
            )

            self.z[0] = z[0]
            self.z[1] = z[1]
            self.z[2] = z[2]

            # Sensor that generated this measurement.
            self.sensor = sensor

            # Lidar measurement-noise covariance.
            self.R = np.matrix(
                [
                    [
                        sigma_lidar_x ** 2,
                        0,
                        0
                    ],
                    [
                        0,
                        sigma_lidar_y ** 2,
                        0
                    ],
                    [
                        0,
                        0,
                        sigma_lidar_z ** 2
                    ]
                ]
            )

            # Lidar-only object attributes.
            self.width = z[4]
            self.length = z[5]
            self.height = z[3]
            self.yaw = z[6]

        elif sensor.name == 'camera':

            ############
            # TODO Step 4: initialize camera measurement including z, R, and sensor
            ############

            sigma_camera_i = params.sigma_cam_i
            sigma_camera_j = params.sigma_cam_j

            # Camera measurement vector:
            #
            # z = [image coordinate i, image coordinate j]^T
            self.z = np.matrix(
                np.zeros(
                    (
                        sensor.dim_meas,
                        1
                    )
                )
            )

            self.z[0] = z[0]
            self.z[1] = z[1]

            # Sensor that generated this measurement.
            self.sensor = sensor

            # Camera measurement-noise covariance.
            self.R = np.matrix(
                [
                    [
                        sigma_camera_i ** 2,
                        0
                    ],
                    [
                        0,
                        sigma_camera_j ** 2
                    ]
                ]
            )

            ############
            # END student code
            ############

        else:
            raise ValueError(
                'Unknown sensor type: {}'.format(
                    sensor.name
                )
            )