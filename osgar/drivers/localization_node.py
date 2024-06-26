"""
    TODO
"""

import math
import collections

from osgar.lib.localization import Localization
from osgar.node import Node

from osgar.lib.route import Convertor

Pose2d = collections.namedtuple("Pose2d", ("x", "y", "heading"))

def list2xy(data):
    """
        Rozdeli seznam dvojic cisel do dvou seznamu.

        Args:
            data (list of (int, int))

        Returns (list, list)
    """
    x = [coord[0] for coord in data]
    y = [coord[1] for coord in data]
    return x, y

class LocalizationNode(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose3d')  # register a stream to be published
        self.localization = Localization()
        self.last_odom = None
        self.verbose = False

        self.gps_convertor = Convertor() # prevadi uhly na 2D kartezske souradnice
        self.gps_planar_first = None # pro posunuti gps do pocatku

        # for debug example
        self.draw_odom = []
        self.draw_poses = []
        self.draw_gps_planar = []
        self.draw_gps_planar_filtered = []
        self.draw_poses_rotated = []
        self.draw_angles = []

    def on_pose2d(self, data):
        """
            To be run if pose2d is received.

            Args:
                data (list of int): seznam o trech polozkach:
                    * x: souradnice, asi v milimetrech
                    * y: souradnice, asi v milimetrech
                    * heading: uhel v setinach stupne
        """
        x, y, heading = data
        if self.verbose:
            self.draw_odom.append([x/1000, y/1000])
        odom = Pose2d(x / 1000.0, y / 1000.0, math.radians(heading / 100.0)) # odom je instance tridy Pose2d s atributy: x, y, heading
        # vypocet `dist` = ujeta vzdalenost; muze byt zaporna, pokud couva
        if self.last_odom is not None:
            dist = math.hypot(odom.x - self.last_odom.x, odom.y - self.last_odom.y) # math.hypot ... spocita delku prepony pravouhleho trojuhelnika
            direction = ((odom.x - self.last_odom.x) * math.cos(self.last_odom.heading) +
                         (odom.y - self.last_odom.y) * math.sin(self.last_odom.heading))
            if direction < 0:
                dist = -dist
        else:
            dist = 0.0
        self.last_odom = odom
        pose3d = self.localization.get_pose3d(dist) # tady se predpoklada aplikace Kalmanova filtru
        if pose3d:
            self.publish("pose3d", pose3d)
            if self.verbose:  # verbose/debug mode
                (x, y, z), q = pose3d
                self.draw_poses.append([x, y])
                #print('-->', self.localization.kf_pos.rotate_odometry_measurement([x, y, z], self.time.total_seconds()), 'angle', self.localization.kf_pos.angle)
                self.draw_poses_rotated.append(self.localization.kf_pos.rotate_odometry_measurement([x, y, z], self.time.total_seconds()))
                t = self.time.total_seconds()
                self.draw_angles.append([t, self.localization.kf_pos.angle])

    def on_position(self, data):
        """
            To be run if position (gps) is received.

            Args:
                data (list of int): zeměpisná souřadnice v milivteřinách;
                    seznam o dvou polozkach; asi to je [E, N].
                    Pozn: poloha katedry matematiky je 50.1291236N, 14.3736517E (podle mapy.cz).
        """
        if len(data) == 2 and all(elem != None for elem in data):

            gps_in_miliseconds = data
            gps_in_degrees = [value / 3600000 for value in gps_in_miliseconds]
            gps_planar = self.gps_convertor.geo2planar(gps_in_degrees)

            # posunuti gps_planar tak, aby prvni gps_planar bylo `[0.0, 0.0]`
            if self.gps_planar_first == None:
                self.gps_planar_first = gps_planar 
            gps_planar = [a - b for a, b in zip(gps_planar, self.gps_planar_first)]

            self.localization.update_gps_planar(gps_planar, self.time.total_seconds())

            if self.verbose:  # verbose/debug mode
                self.draw_gps_planar.append(gps_planar)
                a = self.localization.get_gps_planar_filtered()
                gps_planar_filtered = self.localization.get_gps_planar_filtered()
                self.draw_gps_planar_filtered.append(gps_planar_filtered)
                #print(gps_planar, gps_planar_filtered)

    def on_orientation(self, data):
        """
            Zpracuje dalsi udaj o orientaci robota.

            Args:
                data (list of float): kvaternion; seznam o 4 polozkach
        """
        self.localization.update_orientation(data)

    def draw(self):
        """
            In verbose mode and with --draw parameter: draw a plot.
        """
        import matplotlib.pyplot as plt
        x, y = list2xy(self.draw_poses)
        plt.plot(x, y, "k,", label="pose3d")
        x, y = list2xy(self.draw_odom)
        plt.plot(x, y, "r,", label="odom")
        x, y = list2xy(self.draw_gps_planar)
        plt.plot(x, y, "gx", label="gps_planar")
        x, y = list2xy(self.draw_gps_planar_filtered)
        plt.plot(x, y, "b.", label="gps_planar_filtered")
        x, y = list2xy(self.draw_poses_rotated)
        plt.plot(x, y, "m.", label="poses_rotated")
        plt.legend()

        plt.figure()
        t, a = list2xy(self.draw_angles)
        plt.plot(t, a, "b,", label="angles")
        plt.legend()

        plt.show()
