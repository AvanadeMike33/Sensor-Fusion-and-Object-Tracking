# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest
# neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together
# with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

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


class Association:
    '''Data association class with single nearest neighbor association
    and gating based on Mahalanobis distance'''

    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []

    def associate(self, track_list, meas_list, KF):

        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association
        #   matrix based on Mahalanobis distance for all tracks and
        #   all measurements
        # - update list of unassigned measurements and tracks
        ############

        # Rows correspond to tracks.
        # Columns correspond to measurements.
        # Infinity means the track-measurement pair did not pass gating.
        self.association_matrix = np.matrix(
            np.inf * np.ones(
                (
                    len(track_list),
                    len(meas_list)
                )
            )
        )

        # Initially all tracks and measurements are unassigned.
        self.unassigned_tracks = list(
            range(len(track_list))
        )

        self.unassigned_meas = list(
            range(len(meas_list))
        )

        # Compute the gated Mahalanobis distance for every possible pair.
        for track_index, track in enumerate(track_list):
            for meas_index, meas in enumerate(meas_list):

                # Association is meaningful only when the predicted track
                # lies inside the field of view of the current sensor.
                if not meas.sensor.in_fov(track.x):
                    continue

                distance = self.MHD(
                    track,
                    meas,
                    KF
                )

                if self.gating(
                    distance,
                    meas.sensor
                ):
                    self.association_matrix[
                        track_index,
                        meas_index
                    ] = distance

        ############
        # END student code
        ############

    def get_closest_track_and_meas(self):

        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete corresponding row and column
        # - remove corresponding track and measurement from unassigned
        # - return the association pair
        ############

        # Stop when no rows or columns remain.
        if (
            self.association_matrix.shape[0] == 0
            or self.association_matrix.shape[1] == 0
        ):
            return np.nan, np.nan

        minimum_value = np.min(
            self.association_matrix
        )

        # If the minimum is infinity, all remaining pairs failed gating.
        if np.isinf(minimum_value):
            return np.nan, np.nan

        flat_index = np.argmin(
            self.association_matrix
        )

        matrix_track_index, matrix_meas_index = np.unravel_index(
            flat_index,
            self.association_matrix.shape
        )

        # Map current matrix indices back to the original list indices.
        update_track = self.unassigned_tracks[
            matrix_track_index
        ]

        update_meas = self.unassigned_meas[
            matrix_meas_index
        ]

        # Remove the selected track row.
        self.association_matrix = np.delete(
            self.association_matrix,
            matrix_track_index,
            axis=0
        )

        # Remove the selected measurement column.
        self.association_matrix = np.delete(
            self.association_matrix,
            matrix_meas_index,
            axis=1
        )

        # Remove the selected indices from the unassigned lists.
        self.unassigned_tracks.pop(
            matrix_track_index
        )

        self.unassigned_meas.pop(
            matrix_meas_index
        )

        ############
        # END student code
        ############

        return update_track, update_meas

    def gating(self, MHD, sensor):

        ############
        # TODO Step 3: return True if measurement lies inside gate,
        # otherwise False
        ############

        gate_limit = chi2.ppf(
            params.gating_threshold,
            sensor.dim_meas
        )

        return float(MHD) < gate_limit

        ############
        # END student code
        ############

    def MHD(self, track, meas, KF):

        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############

        H = meas.sensor.get_H(
            track.x
        )

        gamma = KF.gamma(
            track,
            meas
        )

        S = KF.S(
            track,
            meas,
            H
        )

        # Squared Mahalanobis distance:
        # gamma.T * inverse(S) * gamma
        distance = (
            gamma.T
            * np.linalg.inv(S)
            * gamma
        )

        return float(
            distance[0, 0]
        )

        ############
        # END student code
        ############

    def associate_and_update(self, manager, meas_list, KF):

        # associate measurements and tracks
        self.associate(
            manager.track_list,
            meas_list,
            KF
        )

        # update associated tracks with measurements
        while (
            self.association_matrix.shape[0] > 0
            and self.association_matrix.shape[1] > 0
        ):

            # search for next association between a track and measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()

            if np.isnan(ind_track):
                print('---no more associations---')
                break

            ind_track = int(ind_track)
            ind_meas = int(ind_meas)

            track = manager.track_list[
                ind_track
            ]

            measurement = meas_list[
                ind_meas
            ]

            # only update tracks in the current sensor's field of view
            if not measurement.sensor.in_fov(
                track.x
            ):
                continue

            # Kalman update
            print(
                'update track',
                track.id,
                'with',
                measurement.sensor.name,
                'measurement',
                ind_meas
            )

            KF.update(
                track,
                measurement
            )

            # Save the timestamp of the associated measurement.
            track.set_t(
                measurement.t
            )

            # update score and track state
            manager.handle_updated_track(
                track
            )

            # save updated track
            manager.track_list[
                ind_track
            ] = track

        # run track management
        manager.manage_tracks(
            self.unassigned_tracks,
            self.unassigned_meas,
            meas_list
        )

        for track in manager.track_list:
            print(
                'track',
                track.id,
                'score =',
                track.score
            )