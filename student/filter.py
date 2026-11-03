# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
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


class Filter:
    '''Kalman filter class'''

    def __init__(self):
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############

        # Constant velocity motion model in 3 dimensions.
        #
        # State vector:
        #
        # x = [px, py, pz, vx, vy, vz]^T
        #
        # Position:
        # p(k+1) = p(k) + dt * v(k)
        #
        # Velocity:
        # v(k+1) = v(k)

        dt = params.dt

        F = np.matrix(
            [
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]
            ]
        )

        return F

        ############
        # END student code
        ############

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############

        # Continuous white noise acceleration model.
        #
        # q describes the process noise caused by unknown acceleration.

        dt = params.dt
        q = params.q

        Q = np.matrix(
            [
                [
                    (dt ** 3 / 3) * q,
                    0,
                    0,
                    (dt ** 2 / 2) * q,
                    0,
                    0
                ],
                [
                    0,
                    (dt ** 3 / 3) * q,
                    0,
                    0,
                    (dt ** 2 / 2) * q,
                    0
                ],
                [
                    0,
                    0,
                    (dt ** 3 / 3) * q,
                    0,
                    0,
                    (dt ** 2 / 2) * q
                ],
                [
                    (dt ** 2 / 2) * q,
                    0,
                    0,
                    dt * q,
                    0,
                    0
                ],
                [
                    0,
                    (dt ** 2 / 2) * q,
                    0,
                    0,
                    dt * q,
                    0
                ],
                [
                    0,
                    0,
                    (dt ** 2 / 2) * q,
                    0,
                    0,
                    dt * q
                ]
            ]
        )

        return Q

        ############
        # END student code
        ############

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        F = self.F()
        Q = self.Q()

        # Predict state:
        #
        # x' = F * x
        x = F * track.x

        # Predict covariance:
        #
        # P' = F * P * F^T + Q
        P = (
            F
            * track.P
            * F.T
            + Q
        )

        # Save predicted state and covariance in track.
        track.set_x(
            x
        )

        track.set_P(
            P
        )

        ############
        # END student code
        ############

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############

        # Get measurement matrix / Jacobian.
        H = meas.sensor.get_H(
            track.x
        )

        # Measurement residual:
        #
        # gamma = z - h(x)
        gamma = self.gamma(
            track,
            meas
        )

        # Residual covariance:
        #
        # S = H * P * H^T + R
        S = self.S(
            track,
            meas,
            H
        )

        # Kalman gain:
        #
        # K = P * H^T * S^-1
        K = (
            track.P
            * H.T
            * np.linalg.inv(S)
        )

        # Updated state:
        #
        # x = x + K * gamma
        x = (
            track.x
            + K * gamma
        )

        # Identity matrix with state dimension.
        I = np.matrix(
            np.identity(
                params.dim_state
            )
        )

        # Updated covariance:
        #
        # P = (I - K * H) * P
        P = (
            I
            - K * H
        ) * track.P

        # Save updated state and covariance in track.
        track.set_x(
            x
        )

        track.set_P(
            P
        )

        ############
        # END student code
        ############

        track.update_attributes(meas)

    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############

        # Evaluate the sensor measurement function at the current state.
        #
        # For lidar this is linear, but using get_hx() makes the filter
        # ready for the nonlinear camera model used later in Step 4.
        hx = meas.sensor.get_hx(
            track.x
        )

        # Residual / innovation:
        #
        # gamma = z - h(x)
        gamma = (
            meas.z
            - hx
        )

        return gamma

        ############
        # END student code
        ############

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        # Residual covariance:
        #
        # S = H * P * H^T + R
        S = (
            H
            * track.P
            * H.T
            + meas.R
        )

        return S

        ############
        # END student code
        ############