"""
    TODO
"""

import osgar.lib.quaternion as quaternion

import numpy as np
from numpy.linalg import inv
import math

class KalmanFilterXYZ:

    def __init__(self, position, startTime = 0, P=np.array([[2,0,0,0,0,0],[0,2,0,0,0,0],[0,0,2,0,0,0],[0,0,0,2,0,0],[0,0,0,0,2,0],[0,0,0,0,0,2]])):
        # First three coordinates correspond to x,y,z positions, second three to x,y,z velocities
        # Matrices used in Kalman filter
        self.position = position
        self.velocity = np.array([[0,0,0]]) # initial velocity
        self.time_of_last_update = startTime
        self.P = P
        self.v = 2 # variance of the sensor
        self.A = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        self.AT = self.A.transpose()
        self.Q = np.array([[0.1,0,0,0.1,0,0],[0,0.1,0,0,0.1,0],[0,0,0.1,0,0,0.1],[0.1,0,0,0.2,0,0],[0,0.1,0,0,0.2,0],[0,0,0.1,0,0,0.2]])
        #self.R = np.array([[self.v*self.v,0,0],[0,self.v*self.v,0],[0,0,self.v*self.v]])
        self.R = np.array([[self.v*self.v,0,0],[0,self.v*self.v,0],[0,0,self.v*self.v]])
        self.H = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0]])
        self.HT = self.H.transpose()

        self.angle = 0

        self.update_angle_counter = 0

    def input_measurement(self, measurement, time):
        """
            Args:
                measurement (list of float): souradnive x, y, z
                time (float): cas
        """
        dt = time-self.time_of_last_update
        self.A = np.array([[1,0,0,dt,0,0],[0,1,0,0,dt,0],[0,0,1,0,0,dt],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])

        #we work with six dimensional vector composed of three position coordinates and three velocity coordinates
        estPos = self.A @ np.concatenate((self.position, self.velocity), axis = None) 

        estP = self.A @ self.P @ self.AT + self.Q
        K = estP @ self.HT @ inv(self.H @ estP @ self.HT + self.R)
        res = estPos + K@((measurement-self.H @ estPos).transpose())

        #saving current position and velocity
        self.position = res[:3]
        self.velocity = res[3:]
        self.P = estP - K @ self.H @ estP
        self.time_of_last_update = time

    def get_position_estimate(self, time):
        return self.position + (time-self.time_of_last_update)*self.velocity

    def update_angle(self, new_angle):
        """
            Args:
                new_angle (float): aktualni odhad uhlu mezi odometrii a gps
            Is called in rotate_odometry_measurement, it should not be called
            twice for one odometry measurement
            TODO Warning: this method does not increase
                self.number_of_odometry_measurements, must be added somewhere, can
                be even to this method
        """
        self.update_angle_counter += 1
        if self.angle == None:
            self.angle = new_angle
        else:
            weight_of_new_measurement = max(0.1, 1/(self.update_angle_counter+2))
            self_point = np.array([math.cos(self.angle), math.sin(self.angle)])
            new_angle_point = np.array([math.cos(new_angle), math.sin(new_angle)])
            # new point is a weighted average of two points of absolute value 1,
            # afterwards its angle is calculated
            new_point = (1-weight_of_new_measurement)*self_point + weight_of_new_measurement*new_angle_point
            point_as_complex_number = np.complex128(new_point[0]+new_point[1]*1j)
            self.angle = np.angle(point_as_complex_number)

    def rotate_odometry_measurement(self, measurement, time):
        """
            Args:
                measurement (list of float): souradnice x, y, z
                time (float): cas

            As a byproduct updates estimate of an angle between odometry and gps
            measurements
        """

        def calculate_angle(b1, b2):
            """
                Args:
                    b1 (list of float): souradnice x, y bodu v rovine
                    b2: souradnice x, y bodu v rovine

                Returns: angle at [0,0] at which are the points seen, result is in (-pi, pi>

                TODO Warning: this function is not continuous, in the case of angles
                close to pi, it may not work properly
            """
            c1 = np.complex128(b1[0]+b1[1]*1j)
            c2 = np.complex128(b2[0]+b2[1]*1j)
            # TODO muze se stat, ze bude deleni nulou !!!
            return np.angle(c1/c2)
            #if c1 == 0 or c2 == 0:
            #    return None
            #else:
            #    return np.angle(c1/c2)

        current_angle = calculate_angle(self.get_position_estimate(time)[:2], measurement[:2])
        self.update_angle(current_angle)
        matrix_of_rotation = np.array([[math.cos(self.angle), -math.sin(self.angle)],[-math.sin(self.angle), math.cos(self.angle)]])
        #matrix_of_rotation = np.array([[-math.cos(self.angle), -math.sin(self.angle)],[math.sin(self.angle), math.cos(self.angle)]])
        #return matrix_of_rotation @ measurement[:2]
        result = matrix_of_rotation @ measurement[:2]
        result[1] *= -1
        return result


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
