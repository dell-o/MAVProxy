#!/usr/bin/env python
'''
EFLS Module - Emergency Forced Landing System
Dello, Aug 2017

This module connects to the EFLS system through Proto3 to provide a communication pathway for MAVLink messages.

Note: This module is for communication and startup of the EFLS. 
        Please follow the documentation to compile and setup EFLS.
'''

import os
import os.path
import sys
import math
from pymavlink import mavutil
import errno
import time
from MAVProxy.modules.mavproxy_example import mavProxyLink_pb2

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class efls(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        self.mpstate = mpstate
        
        super(efls, self).__init__(mpstate, "efls", "")
        self.status_callcount = 0
        self.boredom_interval = 10 # seconds
        self.last_bored = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False
        
        self.time_previous = 0
        self.protocol_interval = 1
        
        # ToDo - Change paths
        self.protocolbuf_in = "/home/dello/workspace/EFL/aircraftLink_medium_waypoints"
        self.protocolbuf_out = "/home/dello/workspace/EFL/aircraftLink_medium_aircraft"

        self.efls_settings = mp_settings.MPSettings([ ('verbose', bool, False), ])
        self.add_command('efls', self.cmd_efls, "efls module", ['status','set (LOGSETTING)'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: efls <status|set>"

    def cmd_efls(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "status":
            print self.status()
        elif args[0] == "set":
            self.efls_settings.command(args[1:])
        else:
            print self.usage()

    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        self.last_bored = time.time() # status entertains us
        return("status called %(status_callcount)d times.  My target positions=%(packets_mytarget)u  Other target positions=%(packets_mytarget)u" %
               {"status_callcount": self.status_callcount,
                "packets_mytarget": self.packets_mytarget,
                "packets_othertarget": self.packets_othertarget,
               })

    def boredom_message(self):
        if self.efls_settings.verbose:
            return ("I'm very bored")
        return ("I'm bored")

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if now-self.last_bored > self.boredom_interval:
            self.last_bored = now
            message = self.boredom_message()
            self.say("%s: %s" % (self.name,message))
            # See if whatever we're connected to would like to play:
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, message)
            message = "A test of sanity"
            self.say("%s: %s" % (self.name,message))
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, message)                         
                                            
            w = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 1, 10, 16,
                                                          0, 1, 0, 0, 0, 0, -35.372532, 149.163345, 120.000000)

            self.module('wp').wploader.add(w)
            self.master.mav.send(self.module('wp').wploader.wp(w.seq))

            #tell the wp module to expect some waypoints
            self.module('wp').loading_waypoints = True

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            self.lat = m.lat * 1e-7
            self.lon = m.lon * 1e-7
            self.bearing = m.hdg * 1e-2
            self.speed = (m.vx + m.vy) * 1e-2
            self.altitude = m.relative_alt * 1e-2
        elif m.get_type() == 'SYS_STATUS':
            self.motor_current = m.current_battery * 1e-2
        elif m.get_type() == 'VFR_HUD':
            self.motor_throttle = m.throttle
        elif m.get_type() == 'WIND':
            self.wind_speed = m.speed
            self.wind_direction = m.direction
            
            
        now = time.time()
        if now - self.time_previous > self.protocol_interval:
            aircraft_link_out = mavProxyLink_pb2.AircraftLink()
            self.time_previous = now
            #try:
            #    f = open("cat.txt", "rb")
            #    aircraft_link.ParseFromString(f.read())
            #    f.close()
            #except IOError:
            #    print "Could not connect to Protocol Buf file. Creating new one."
            
            aircraft = aircraft_link_out.aircrafts.add()
            aircraft.lat = self.lat
            aircraft.lon = self.lon
            aircraft.bearing = self.bearing
            aircraft.speed = self.speed
            aircraft.altitude = self.altitude
            aircraft.wind_speed = self.wind_speed
            aircraft.wind_direction = self.wind_direction
            aircraft.motor_current = self.motor_current
            aircraft.motor_throttle = self.motor_throttle
            
            f = open(self.protocolbuf_out, "wb")
            f.write(aircraft_link_out.SerializeToString())
            f.flush()
            f.close
            
            del aircraft
            
            # Read waypoints
            #aircraft_link_in = mavProxyLink_pb2.AircraftLink()
            #f = open(protocolbuf_in, "rb")
            #aircraft_link_in.ParseFromString(f.read())
            #f.close()
            
            #for aircraft in aircraft_link_in.aircrafts:
            #    message = str(aircraft.speed)
            #self.say("%s: %s" % (self.name,message))
            ## See if whatever we're connected to would like to play:
            #self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, message)

def init(mpstate):
    '''initialise module'''
    return example(mpstate)
