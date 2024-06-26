"""
    TODO
"""

import osgar.lib.quaternion as quaternion

from osgar.lib.localization import KalmanFilterXYZ

import numpy as np
from numpy.linalg import inv
import math

class Localization:
    def __init__(self):
        self.last_orientation = None
        self.last_gps_planar = None
        self.last_gps_planar_filtered = None
        self.xyz = [0, 0, 0]

        self.kf_pos = KalmanFilterXYZ(np.zeros(3))

    def update_gps_planar(self, gps_planar, time):
        """
            Vstup: Zpracuje a zapamatuje si dalsi GPS souradnice v poradi.

            Args:
                gps_planar (list of float): poloha [x, y] v metrech odvozena z gps
                time (float): (absolutni) cas ve vterinach
        """
        self.last_gps_planar = gps_planar
        self.kf_pos.input_measurement(gps_planar + [0], time)
        self.last_gps_planar_filtered = self.kf_pos.get_position_estimate(time)[:2]

    def get_gps_planar_filtered(self):
        return self.last_gps_planar_filtered

    def update_orientation(self, orientation):
        """
            Vstup: Zpracuje a zapamatuje si dalsi orientaci robota v poradi.

            Args:
                orientation (list of float): kvaternion; seznam o 4 polozkach
        """
        self.last_orientation = orientation

    def get_pose3d(self, dist):
        """
            Vystup: Vypocita a vrati korigovanou 3d pozici.

            Args:
                dist (float): ujeta vzdalenost

            Returns (list): seznam o dvou polozkach
                1. seznam tri cisel ... souradnice v metrech
                2. seznam ctyr cisel ... kvaternion
        """
        if self.last_orientation is None:
            return
        dist3d = quaternion.rotate_vector([dist, 0, 0], self.last_orientation)
        self.xyz = [a + b for a, b in zip(self.xyz, dist3d)]
        pose3d = [self.xyz, self.last_orientation]

        if self.last_gps_planar:
            pass  # add gps

        return pose3d
