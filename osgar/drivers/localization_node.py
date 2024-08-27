"""
    TODO
"""

import math
import collections

from osgar.lib.localization import Localization
from osgar.node import Node

from osgar.lib.route import Convertor

import osgar.lib.quaternion as quaternion

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

        self.on_its_way = False # nastavi se na True ve chvili, kdy se robot (podle dat z odometrie) rozjede

        self.outF = open('out.data', 'w')

    def on_pose2d(self, data):
        """
            To be run if pose2d is received.

            Args:
                data (list of int): seznam o trech polozkach:
                    * x: souradnice, asi v milimetrech
                    * y: souradnice, asi v milimetrech
                    * heading: uhel v setinach stupne
        """
        #print('on_pose2d', self.time.total_seconds(), data, type(data))
        #for value in data:
        #    print(value, type(value))
        outStr = 'pose2d;{};{}'.format(self.time.total_seconds(), len(data))
        for value in data:
            outStr += ';{}'.format(value)
        self.outF.write(outStr + "\n")
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

        if dist > 0.0001:
            self.on_its_way = True

        pose3d = self.localization.update_distance(self.time, dist)

        if pose3d:
            self.publish("pose3d", pose3d)

    def on_position(self, data):
        """
            To be run if position (gps) is received.

            Args:
                data (list of int): zeměpisná souřadnice v milivteřinách;
                    seznam o dvou polozkach; asi to je [E, N].
                    Pozn: poloha katedry matematiky je 50.1291236N, 14.3736517E (podle mapy.cz).
        """
        outStr = 'position;{};{}'.format(self.time.total_seconds(), len(data))
        for value in data:
            outStr += ';{}'.format(value)
        self.outF.write(outStr + "\n")
        if self.on_its_way and len(data) == 2 and all(elem != None for elem in data):

            gps_in_miliseconds = data
            gps_in_degrees = [value / 3600000 for value in gps_in_miliseconds]
            gps_planar = self.gps_convertor.geo2planar(gps_in_degrees)

            # posunuti gps_planar tak, aby prvni gps_planar bylo `[0.0, 0.0]`
            if self.gps_planar_first == None:
                self.gps_planar_first = gps_planar 
            gps_planar = [a - b for a, b in zip(gps_planar, self.gps_planar_first)]

            self.localization.update_gps_planar(self.time, gps_planar)

    def on_orientation(self, data):
        """
            Zpracuje dalsi udaj o orientaci robota.

            Args:
                data (list of float): kvaternion; seznam o 4 polozkach
        """
        outStr = 'orientation;{};{}'.format(self.time.total_seconds(), len(data))
        for value in data:
            outStr += ';{}'.format(value)
        self.outF.write(outStr + "\n")
        self.localization.update_orientation(self.time, data)

    def draw(self):
        """
            Draw a plot with the --draw parameter.

            (No need to turn on the verbose mode with the --verbose parameter.)
        """

        self.outF.close()

        #from osgar.lib.localization import interpolate
        #x_a = 1
        #x_b = 3
        #y_a = 5
        #y_b = 2
        #x_i = x_a
        #while x_i <= x_b:
        #    print(x_i, interpolate(x_i, x_a, y_a, x_b, y_b))
        #    x_i += 0.1

        #self.localization.compute_angle()
        #return



        import matplotlib.pyplot as plt

        # draw 'gps planar'
        draw_gps_planar_x = []
        draw_gps_planar_y = []
        for time, gps_planar in self.localization.history['gps planar']:
            draw_gps_planar_x.append(gps_planar[0])
            draw_gps_planar_y.append(gps_planar[1])
        plt.plot(draw_gps_planar_x, draw_gps_planar_y, 'b.', label = 'gps planar')
        # draw 'gps planar filtered'
        draw_gps_planar_f_x = []
        draw_gps_planar_f_y = []
        for time, gps_planar_f in self.localization.history['gps planar filtered']:
            draw_gps_planar_f_x.append(gps_planar_f[0])
            draw_gps_planar_f_y.append(gps_planar_f[1])
        plt.plot(draw_gps_planar_f_x, draw_gps_planar_f_y, 'r.', label = 'gps planar filtered')
        # draw 'gps planar filtered v2'
        #draw_gps_planar_fa_x = []
        #draw_gps_planar_fa_y = []
        #for time, gps_planar_fa in self.localization.history['gps planar filtered a']:
        #    draw_gps_planar_fa_x.append(gps_planar_fa[0])
        #    draw_gps_planar_fa_y.append(gps_planar_fa[1])
        #plt.plot(draw_gps_planar_fa_x, draw_gps_planar_fa_y, 'mx', label = 'gps planar filtered v2')
        # draw 'position from odometry'
        draw_xyz_x = []
        draw_xyz_y = []
        draw_xyz_z = []
        for time, xyz in self.localization.history['position from odometry']:
            draw_xyz_x.append(xyz[0])
            draw_xyz_y.append(xyz[1])
            draw_xyz_z.append(xyz[2])
        plt.plot(draw_xyz_x, draw_xyz_y, 'g.', label = 'position from odometry')
        # draw 'position from odometry filtered'
        draw_xy_f_x = []
        draw_xy_f_y = []
        for time, xy_f in self.localization.history['position from odometry filtered']:
            draw_xy_f_x.append(xy_f[0])
            draw_xy_f_y.append(xy_f[1])
        plt.plot(draw_xy_f_x, draw_xy_f_y, 'm.', label = 'position from odometry filtered')
        # draw "adjusted" position from odometry
        #odo_adjusted = self.localization.get_odo_adjusted_to_gps()
        #odo_adjusted = self.localization.get_gps_adjusted_to_odo()
        #draw_xy_a_x = []
        #draw_xy_a_y = []
        #for time, xy_a in odo_adjusted:
        #    draw_xy_a_x.append(xy_a[0])
        #    draw_xy_a_y.append(xy_a[1])
        #plt.plot(draw_xy_a_x, draw_xy_a_y, 'c.', label = 'position from odometry adjusted')
        #
        plt.legend()
        plt.show()
