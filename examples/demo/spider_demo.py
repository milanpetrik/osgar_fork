"""
  Demo Spider3 capabilities
"""
import math
from datetime import timedelta

from osgar.node import Node


def distance(pose1, pose2):
    return math.hypot(pose1[0] - pose2[0], pose1[1] - pose2[1])


class ObstacleException(Exception):
    pass


class MyApp(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.max_speed = config.get('max_speed', 0.1)
        self.max_angular_speed = math.radians(50)  # TODO config
        self.verbose = False
        self.last_position = (0, 0, 0)
        self.is_moving = False
        self.pose2d = None  # TODO should be defined by super().__init__()
        self.safety_limit = 3.0  # terminate run when reached, None for no limit

    # TODO refactor to common "serializer"
    def send_speed_cmd(self, speed, angular_speed):
        return self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    # TODO refactor to common driver (used from sick2018 example)
    def go_straight(self, how_far):
        print(self.time, "go_straight %.1f" % how_far, self.last_position)
        start_pose = self.last_position
        if how_far >= 0:
            self.send_speed_cmd(self.max_speed, 0.0)
        else:
            self.send_speed_cmd(-self.max_speed, 0.0)
        while distance(start_pose, self.last_position) < abs(how_far):
            self.update()
        self.send_speed_cmd(0.0, 0.0)

    def turn(self, angle, with_stop=True):
        print(self.time, "turn %.1f" % math.degrees(angle))
        start_pose = self.last_position
        if angle >= 0:
            self.send_speed_cmd(0.0, self.max_angular_speed)
        else:
            self.send_speed_cmd(0.0, -self.max_angular_speed)
        while abs(start_pose[2] - self.last_position[2]) < abs(angle):
            self.update()
        if with_stop:
            self.send_speed_cmd(0.0, 0.0)
            start_time = self.time
            while self.time - start_time < timedelta(seconds=2):
                self.update()
                if not self.is_moving:
                    break
            print(self.time, 'stop at', self.time - start_time)

    def wait(self, dt):
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def on_pose2d(self, data):
        prev = self.pose2d
        self.pose2d = data
        x_mm, y_mm, heading_mdeg = self.pose2d
        self.last_position = (x_mm / 1000.0, y_mm / 1000.0,
                              math.radians(heading_mdeg / 100.0))
        self.is_moving = (prev != self.pose2d)

    def on_min_dist(self, data):
        self.min_dist = data
        dist, azi = self.min_dist
        if self.safety_limit is not None and dist < self.safety_limit:
            print(dist, 'at direction', azi/100)
            self.send_speed_cmd(0.0, 0.0)
            raise ObstacleException()

    def run(self):
        print("Spider demo back & forth!")
        step_size = 3  # meters

        try:
            for i in range(4):
                self.go_straight(step_size)
                self.wait(dt=timedelta(seconds=2.0))
                self.go_straight(-step_size)
                self.wait(dt=timedelta(seconds=2.0))
        except ObstacleException:
            print("ObstacleException")

        self.send_speed_cmd(0.0, 0.0)
        self.safety_limit = None  # i.e. disable exceptions
        self.wait(dt=timedelta(seconds=1.0))

        print("END")


if __name__ == "__main__":
    from osgar.launcher import launch

    launch(app=MyApp, description='Spider Demo', prefix='spider-demo-')

# vim: expandtab sw=4 ts=4