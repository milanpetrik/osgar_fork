"""
    TODO
"""

import osgar.lib.quaternion as quaternion

import numpy as np
from numpy.linalg import inv
import math

def interpolate(x, x_a, y_a, x_b, y_b):
    """
        Vrati afinne interpolovanou hodnotu mezi body [x_a, y_a] a [x_b, y_b]
            na zaklade parametru `x`.
    """
    if abs(x_b - x_a) < 0.000001:
        return (y_a + y_b) / 2
    else:
        return y_a + ( (x - x_a) * (y_b - y_a) / (x_b - x_a) )

class Localization:
    def __init__(self):
        self.kf_xyz = KalmanFilterXYZ(np.zeros(3))
        self.kf_gps = KalmanFilterXYZ(np.zeros(3))

        #self.kf_xyz_a = KalmanFilterAccXY(np.zeros(3))
        #self.kf_gps_a = KalmanFilterAccXY(np.zeros(3))

        self.history = {
                'distance': [],
                'orientation': [],
                'gps planar': [],
                'position from odometry': [],
                'gps planar filtered': [],
                'gps planar filtered a': [],
                'position from odometry filtered': [],
                'pose3d': [],
            }

    def update_gps_planar(self, time, gps_planar):
        """
            Zpracuje a zapamatuje si dalsi GPS souradnice v poradi.

            Args:
                gps_planar (list of float): poloha [x, y] v metrech odvozena z gps
                time (datetime.timedelta): (absolutni) cas;
                    cas ve vterinach lze ziskat pomoci `time.total_seconds()`
        """
        self.history['gps planar'].append( (time, gps_planar) )
        # Kalman filter
        time_in_seconds = time.total_seconds()
        self.kf_gps.input_measurement(gps_planar + [0], time_in_seconds)
        gps_planar_filtered = self.kf_gps.get_position_estimate(time_in_seconds)[:2]
        self.history['gps planar filtered'].append( (time, gps_planar_filtered) )
        # Kalman filter v2
        #self.kf_gps_a.input_measurement(gps_planar + [0], time_in_seconds)
        #gps_planar_filtered = self.kf_gps_a.get_position_estimate(time_in_seconds)[:2]
        #self.history['gps planar filtered a'].append( (time, gps_planar_filtered) )

    def update_orientation(self, time, orientation):
        """
            Zpracuje a zapamatuje si dalsi orientaci robota v poradi.

            Args:
                orientation (list of float): kvaternion
                time (datetime.timedelta): (absolutni) cas;
                    cas ve vterinach lze ziskat pomoci `time.total_seconds()`
        """
        self.history['orientation'].append( (time, orientation) )
        
    def update_distance(self, time, distance):
        """
            Zpracuje a zapamatuje si dalsi ujetou vzdalenost robota v poradi.

            Args:
                distance(float): ujeta vzdalenost; muze byt zaporna, pokud
                    robot couva
                time (datetime.timedelta): (absolutni) cas;
                    cas ve vterinach lze ziskat pomoci `time.total_seconds()`

            Returns (list): seznam o dvou polozkach
                1. seznam tri cisel ... souradnice v metrech
                2. seznam ctyr cisel ... kvaternion
        """
        self.history['distance'].append( (time, distance) )
        orientation_time, orientation = self.history['orientation'][-1]
        distance_3D = [distance * o for o in orientation]
        distance_3D = quaternion.rotate_vector([distance, 0.0, 0.0], orientation)
        if len(self.history['position from odometry']) == 0:
            last_xyz = [0.0, 0.0, 0.0]
        else:
            last_xyz_time, last_xyz = self.history['position from odometry'][-1]
        xyz = [a + b for a, b in zip(last_xyz, distance_3D)]
        self.history['position from odometry'].append( (time, xyz) )
        # Kalman filter
        time_in_seconds = time.total_seconds()
        self.kf_xyz.input_measurement(xyz, time_in_seconds)
        xy_filtered = self.kf_xyz.get_position_estimate(time_in_seconds)[:2]
        self.history['position from odometry filtered'].append( (time, xy_filtered) )
        # output
        pose3d = [xyz, orientation]
        self.history['pose3d'].append( (time, pose3d) )
        return pose3d

    def get_pose3d(self):
        """
            Vrati posledni hodnotu pose3d.

            Returns (list): seznam o dvou polozkach
                1. seznam tri cisel ... souradnice v metrech vyjadrujici polohu robota
                2. seznam ctyr cisel ... kvaternion vyjadrujici orientaci robota
        """
        time, pose3d = self.history['pose3d'][-1]
        return pose3d

    def get_interpolated_position_from_gps(self, time):
        """
            Zpracuje a zapamatuje si dalsi orientaci robota v poradi.

            Args:
                time (datetime.timedelta): cas;
                    napriklad: `time = datetime.timedelta(hours=1, minutes=14, seconds=5.31)`;
                    cas ve vterinach lze potom ziskat pomoci `time.total_seconds()`
        """
        #'gps planar filtered': [],
        #        'position from odometry filtered': [],
        pass

    def compute_angle_and_scale_for_odo_to_gps(self):
        gps = self.history[ 'gps planar filtered' ]
        odo = self.history[ 'position from odometry filtered' ]
        # posunu indexy tak, aby platilo: odo[j-1]['time'] <= gps[i] <= odo[j]['time']
        # nebo: gps[i-1]['time'] <= odo[j] <= gps[i]['time']
        i = j = 0
        gps_time, gps_pos = gps[i]
        odo_time, odo_pos = odo[j]
        if gps_time < odo_time:
            while gps_time < odo_time:
                i += 1
                gps_time, gps_pos = gps[i]
        elif odo_time < gps_time:
            while odo_time < odo_time:
                j += 1
                odo_time, odo_pos = odo[j]
        else:
            i += 1
            j += 1
        # posouvam indexy dale, dokud se v gps a odo neobjevi nenulove hodnoty
        while True:
            gps_time, gps_pos = gps[i]
            odo_time, odo_pos = odo[j]
            if any(abs(crd) > 0.0001 for crd in gps_pos) and any(abs(crd) > 0.0001 for crd in odo_pos):
                break
            if gps_time < odo_time:
                i += 1
            elif odo_time < gps_time:
                j += 1
            else:
                i += 1
                j += 1
        # vypocet uhlu mezi gps a odo
        # chceme otocit a natahnout odo tak, aby vysledek byl co nejblize gps
        counter = 0
        angle_sin_sum = 0.0
        angle_cos_sum = 0.0
        scale_upper_sum = 0.0
        scale_lower_sum = 0.0
        len_gps = len(gps)
        len_odo = len(odo)
        while i < len_gps and j < len_odo:
            gps_time, gps_pos = gps[i]
            odo_time, odo_pos = odo[j]
            if gps_time < odo_time:
                odo_time_prev, odo_pos_prev = odo[j - 1]
                gps_pos_res = gps_pos
                odo_pos_res = [ interpolate(gps_time.total_seconds(), odo_time_prev.total_seconds(), odo_pos_prev[k], odo_time.total_seconds(), odo_pos[k]) for k in range(2) ]
                i += 1
            elif odo_time < gps_time:
                gps_time_prev, gps_pos_prev = gps[j - 1]
                gps_pos_res = [ interpolate(odo_time.total_seconds(), gps_time_prev.total_seconds(), gps_pos_prev[k], gps_time.total_seconds(), gps_pos[k]) for k in range(2) ]
                odo_pos_res = odo_pos
                j += 1
            else:
                gps_pos_res = gps_pos
                odo_pos_res = odo_pos
                i += 1
                j += 1
            # prevod gps a odo na komplexni cisla
            # zjisteni jejich uhlu a absolutnich hodnot
            gps_complex = complex( gps_pos_res[0], gps_pos_res[1] )
            odo_complex = complex( odo_pos_res[0], odo_pos_res[1] )
            gps_abs = np.absolute(gps_complex)
            odo_abs = np.absolute(odo_complex)
            gps_ang = np.angle( gps_complex )
            odo_ang = np.angle( odo_complex )

            #angle_sin_sum += np.sin( (odo_ang - gps_ang) / 2 )
            #angle_cos_sum += np.cos( (odo_ang - gps_ang) / 2 )
            angle_sin_sum += np.sin( (odo_ang - gps_ang) / 2 )
            angle_cos_sum += np.cos( (odo_ang - gps_ang) / 2 )

            scale_upper_sum += odo_abs * gps_abs
            scale_lower_sum += odo_abs * odo_abs

        scale = scale_upper_sum / scale_lower_sum

        cos_sqr_angle_half = angle_sin_sum**2 / (angle_sin_sum**2 + angle_cos_sum**2)
        #angle = 2 * np.arccos( np.sqrt( cos_sqr_angle_half ) )
        angle = 2 * np.arccos( -np.sqrt( cos_sqr_angle_half ) )

        return angle, scale

    def get_odo_adjusted_to_gps(self):
        angle, scale = self.compute_angle_and_scale_for_odo_to_gps()
        adjuster = scale * np.exp( 1j * angle )
        #print('angle:', angle, 'scale:', scale, 'adjuster:', adjuster)
        odo_adjusted = []
        for odo_time, odo_pos in self.history[ 'position from odometry filtered' ]:
            odo_pos_complex = odo_pos[0] + 1j * odo_pos[1]
            odo_pos_adjusted_complex = odo_pos_complex * adjuster
            #print(odo_pos, ' * ', adjuster, ' = ', odo_pos_adjusted_complex)
            odo_pos_adjusted = [ odo_pos_adjusted_complex.real, odo_pos_adjusted_complex.imag ]
            #print(odo_pos, '...', odo_pos_adjusted)
            odo_adjusted.append( (odo_time, odo_pos_adjusted) )
        return odo_adjusted

    def get_gps_adjusted_to_odo(self):
        angle, scale = self.compute_angle_and_scale_for_odo_to_gps()
        adjuster = scale * np.exp( 1j * angle )
        #print('angle:', angle, 'scale:', scale, 'adjuster:', adjuster)
        gps_adjusted = []
        for gps_time, gps_pos in self.history[ 'gps planar filtered' ]:
            gps_pos_complex = gps_pos[0] + 1j * gps_pos[1]
            gps_pos_adjusted_complex = gps_pos_complex * adjuster
            #print(gps_pos, ' * ', adjuster, ' = ', gps_pos_adjusted_complex)
            gps_pos_adjusted = [ gps_pos_adjusted_complex.real, gps_pos_adjusted_complex.imag ]
            #print(gps_pos, '...', gps_pos_adjusted)
            gps_adjusted.append( (gps_time, gps_pos_adjusted) )
        return gps_adjusted





    def run_unit_tests(self):

        x_a = 1
        x_b = 9
        y_a = 2
        y_b = 7

        x_i = x_a
        while x_i <= x_b:
            print(x_i, self.localization.interpolate(x_i, x_a, y_a, x_b, y_b))
            x_i += 0.1



















class KalmanFilterAccXY:

    def __init__(self, position, startTime = 0, P=np.array([[2,0,0,0,0,0],[0,2,0,0,0,0],[0,0,2,0,0,0],[0,0,0,2,0,0],[0,0,0,0,2,0],[0,0,0,0,0,2]])):
        # First two coordinates correspond to x,y positions, second two to x,y
        # velocities, the last to acceleration
        # Matrices used in Kalman filter
        self.position = position
        self.velocity = np.array([[0,0]]) # initial velocity
        self.acceleration = np.array([[0,0]]) # initial acceleration
        self.time_of_last_update = startTime
        self.P = P
        self.v = 2 # variance of the sensor
        self.A = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        self.AT = self.A.transpose()
        self.Q = np.array([[0.1,0,0.1,0,0.1,0],[0,0.1,0,0.1,0,0.1],[0.1,0,0.2,0,0.2,0],[0,0.1,0,0.2,0,0.2],[0.1,0,0.2,0,0.2,0],[0,0.1,0,0.2,0,0.2]])
        self.R = np.array([[self.v*self.v,0],[0,self.v*self.v]])
        self.H = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0]])
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
        da = dt*dt/2
        self.A = np.array([[1,0,dt,0,da,0],[0,1,0,dt,0,da],[0,0,1,0,dt,0],[0,0,0,1,0,dt],[0,0,0,0,1,0],[0,0,0,0,0,1]])

        #we work with six dimensional vector composed of three position coordinates and three velocity coordinates
        estPos = self.A @ np.concatenate((self.position, self.velocity), axis = None) 

        estP = self.A @ self.P @ self.AT + self.Q
        K = estP @ self.HT @ inv(self.H @ estP @ self.HT + self.R)
        res = estPos + K@((measurement-self.H @ estPos).transpose())

        #saving current position and velocity
        self.position = res[:2]
        self.velocity = res[2:4]
        self.acceleration = res[4:]
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

