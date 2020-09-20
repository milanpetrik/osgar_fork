"""
  Parse Velodyne VLP-16 data
"""
import struct
import math

import numpy as np

from osgar.node import Node

LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16


def parse_packet(data, offset_step=100, blind_dist=1.0):
    assert len(data) == 1206, len(data)
    assert offset_step % 100 == 0, offset_step  # must be divisible by 100
    timestamp, factory = struct.unpack_from("<IH", data, offset=1200)
    assert factory == 0x2237, hex(factory)  # 0x22=VLP-16, 0x37=Strongest Return
#    time = timestamp/1000000.0
#    if self.time is not None:
#        lost_packets = int(round((time - self.time)/EXPECTED_PACKET_TIME)) - 1
#    else:
#        lost_packets = 0
#    self.time = time
#    if lost_packets > 0 and (self.last_blocked is None or self.time > self.last_blocked + EXPECTED_SCAN_DURATION):
#        self.last_blocked = self.time + EXPECTED_SCAN_DURATION
#        self.scan_index += 1
#        print("DROPPED index", self.scan_index)
#    if self.last_blocked is not None and self.time < self.last_blocked:
#        return  # to catch up-to-date packets again ...

    ret = []
    min_dist = None
    min_azi = None
    for offset in range(0, 1200, offset_step):  # 100 bytes per one reading
        flag, azi = struct.unpack_from("<HH", data, offset)
        assert flag == 0xEEFF, hex(flag)
        azimuth = azi/100.0
        # H-distance (2mm step), B-reflectivity (0
        arr = struct.unpack_from('<' + "HB"*32, data, offset + 4)
        dist = []
        for i in range(NUM_LASERS):
            dist.append(arr[i*2] * 0.002)

        # TODO refactor in numpy!!!
        a = np.array(dist)
        mask = a > blind_dist  # note, 0 is no reflection (probably)
        if max(mask):
            # i.e. at least one element is valid
            d = min(a[mask])
            if min_dist is None or min_dist > d:
                min_dist = d
                min_azi = azi  # 100th degree

        # so now we have azimuth and NUM_LASERS distance readings
        for d, beta_deg in zip(dist, LASER_ANGLES):
            beta = math.radians(beta_deg)
            x = d * math.cos(azimuth) * math.cos(beta)
            y = d * math.sin(azimuth) * math.cos(beta)
            z = d * math.sin(beta)
            ret.append([x, y, z])
    return (min_dist, min_azi), ret


class Velodyne(Node):
    def __init__(self, config, bus):
        bus.register('raw', 'xyz', 'min_dist')        
        super().__init__(config, bus)
        self.offset_step = config.get('offset_step', 200)  # skip every second packet (we need 1deg resolution input 0.4)
        self.should_publish_xyz = config.get('publish_xyz', True)
        assert self.offset_step % 100 == 0, self.offset_step  # must be divisible by 100

    def update(self):
        channel = super().update()
        assert channel == 'raw', channel
        if self.should_publish_xyz:
            self.publish('xyz', parse_packet(self.raw, offset_step=self.offset_step))
        else:
            min_pair, __ = parse_packet(self.raw, offset_step=self.offset_step)
            min_dist, min_azi = min_pair
            self.publish('min_dist', [min_dist, min_azi])
#            if min_dist is not None and min_dist < 3:
#                print(self.time, min_dist, 'at', min_azi)


# vim: expandtab sw=4 ts=4
