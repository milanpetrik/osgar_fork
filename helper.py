#!/usr/bin/python
"""
  Helper functions for attaching various sensors
"""
from gps import GPS
from gps import DummyGPS as DummySensor  # TODO move to apyros, as mock
from laser import LaserIP
from apyros.metalog import MetaLog, disableAsserts
from apyros.sourcelogger import SourceLogger


def gps_data_extension(robot, id, data):
    if id=='gps':
        robot.gps_data = data
        if robot.localization is not None:
            robot.localization.updateGPS(data) 


def laser_data_extension(robot, id, data):
    if id=='laser':
        robot.laser_data = data


def min_dist(data):
    data = np.array(data)
    mask = (data > 0)
    if np.any(mask):
        return np.min(data[mask]) * 0.001
    return None 


def attach_sensor(robot, sensor_name, metalog):
    assert sensor_name in ['gps', 'laser', 'camera'], sensor_name

    if sensor_name == 'gps':
        # GPS
        gps_log_name = metalog.getLog('gps')
        print gps_log_name
        if metalog.replay:
            robot.gps = DummySensor()
            function = SourceLogger(None, gps_log_name).get
        else:
            robot.gps = GPS(verbose=0)
            function = SourceLogger(robot.gps.coord, gps_log_name).get
        robot.gps_data = None
        robot.register_data_source('gps', function, gps_data_extension) 
        robot.gps.start()  # ASAP so we get GPS fix

    elif sensor_name == 'laser':
        # Laser
        laser_log_name = metalog.getLog('laser')
        print laser_log_name
        if metalog.replay:
            robot.laser = DummySensor()
            function = SourceLogger(None, laser_log_name).get
        else:
            robot.laser = LaserIP()
            function = SourceLogger(robot.laser.scan, laser_log_name).get
        robot.laser_data = None
        robot.register_data_source('laser', function, laser_data_extension)   
        robot.laser.start()

def detach_all_sensors(robot):
    # TODO unregister all modules
    # TODO conditional stopping
    robot.laser.requestStop()
    robot.gps.requestStop()

# vim: expandtab sw=4 ts=4 

