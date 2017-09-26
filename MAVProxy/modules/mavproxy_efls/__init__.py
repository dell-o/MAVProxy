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
from pymavlink import mavutil, mavwp
import errno
import time
from MAVProxy.modules.mavproxy_efls import mavProxyLink_pb2

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class efls(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        self.mpstate = mpstate
        
        super(efls, self).__init__(mpstate, "efls", "")
        self.status_callcount = 0
        self.check_interval = 1 # seconds
        self.last_check = time.time()
        
        self.wploader = mavwp.MAVWPLoader()
        self.num_wps_expected = 0
        self.wps_received = {}
        self.expect_waypoints = 0
        self.timeout = 0
        self.triggerEFLS = False
        self.wp_pos = 0
        
        self.seq = 1
        self.frame = 10#mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        
        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False
        
        # ToDo - Change paths
        self.protocolbuf_in = "/home/dello/workspace/EFLS/build/res/protobuf/aircraftLink_medium_waypoints"
        self.protocolbuf_out = "/home/dello/workspace/EFLS/build/res/protobuf/aircraftLink_medium_aircraft"

        self.efls_settings = mp_settings.MPSettings([ ('verbose', bool, False), ])
        self.add_command('efls', self.cmd_efls, "efls module", ['status','set (LOGSETTING)'])

        # Clear waypoint data from Protobuf file initally
        f = open(self.protocolbuf_in, "wb")
        f.close()
        
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
        self.last_check = time.time() # status entertains us
        return("status called %(status_callcount)d times.  My target positions=%(packets_mytarget)u  Other target positions=%(packets_mytarget)u" %
               {"status_callcount": self.status_callcount,
                "packets_mytarget": self.packets_mytarget,
                "packets_othertarget": self.packets_othertarget,
               })

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if now-self.last_check > self.check_interval:
            self.last_check = now
            
            # Send Waypoint change command
            if self.triggerEFLS:
                self.master.waypoint_set_current_send(self.wp_pos)
                self.triggerEFLS = False
            
            self.aircraftFunc()
            self.waypointFunc()
            
            
    def aircraftFunc(self):
        '''writes aircraft data to Protobuf file'''
        
        # Defines aircarft data in Protobuf
        aircraft_link_out = mavProxyLink_pb2.AircraftLink()
            
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
        
        # Writes aircraft data to Protobuf file
        f = open(self.protocolbuf_out, "wb")
        f.write(aircraft_link_out.SerializeToString())
        f.flush()
        f.close
                    
            
    def waypointFunc(self):
        '''reads waypoints from Protobuf file'''
        
        # Read waypoints from protobuf
        aircraft_link_in = mavProxyLink_pb2.AircraftLink()
        f = open(self.protocolbuf_in, "rb")
        aircraft_link_in.ParseFromString(f.read())
        f.close()
        #self.master.waypoint_request_list_send()
        
        # Detects new waypoints
        if len(aircraft_link_in.waypoints) > 0:
            
            # Request updated waypoints
            if self.expect_waypoints == 0 or self.timeout > 3:
                self.timeout = 0
                self.wps_received = {}
                self.wploader.clear()
                self.expect_waypoints = 1
                self.mpstate.module('wp').cmd_wp(['list'])
                print 'EFLS: Getting waypoints'
                return
            if (len(self.wps_received) != self.num_wps_expected):
                self.timeout += 1
                print 'EFLS: Fetch not completed'
                return    
            print 'EFLS: Waypoints fetched'  
            self.expect_waypoints = 0    
            self.timeout = 0
            
            # Find EFLS do_land_start --> This is the last do_land_start
            self.wp_pos = 0
            for i in range(self.wploader.count()):
                w_current = self.wploader.wp(i)
                if w_current.command == mavutil.mavlink.MAV_CMD_DO_LAND_START:
                    self.wp_pos = w_current.seq
            if self.wp_pos == 0:
                print 'EFLS: Could not find (Do Land Start) mission item'
                return
            
            # Removes old waypoints --> Required to ensure if less waypoints are added, old ones do not survive
            print "start"
            print self.wp_pos
            print self.wploader.count()
            #for i in range(self.wp_pos,self.wploader.count()):
            #    print i
            #    self.wploader.remove(self.wploader.wp(i))
           
            print "finished"
            # Read waypoints 
            num = self.wp_pos
            for Waypoints in aircraft_link_in.waypoints:
                for Waypoint in Waypoints.waypoint:
                    lat = Waypoint.lat
                    lon = Waypoint.lon
                    alt = Waypoint.altitude
                    
                    # Set Do_Land_Start location
                    if num == self.wp_pos:
                        m = mavutil.mavlink.MAVLink_mission_item_message(self.target_system, self.target_component, num, self.frame, mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 1, 0, 0, 0, 0, lat, lon, alt)
                        
                    # Set Nav_Land location
                    elif num == self.wp_pos + len(Waypoints.waypoint) - 1:
                        m = mavutil.mavlink.MAVLink_mission_item_message(self.target_system, self.target_component, num, self.frame, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, lat, lon, alt)    
                        
                    # Set waypoints
                    else:
                        m = mavutil.mavlink.MAVLink_mission_item_message(self.target_system, self.target_component, num, self.frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, lat, lon, alt)
                    
                    self.mpstate.module('wp').wploader.set(m, m.seq)
                    self.master.mav.send(self.mpstate.module('wp').wploader.wp(m.seq))
                    self.mpstate.module('wp').loading_waypoints = True
                    num += 1
            
            # Send new mission to Aircraft        
            self.mpstate.module('wp').send_all_waypoints()
            
            # Clear waypoint data from Protobuf file
            f = open(self.protocolbuf_in, "wb")
            f.close()
            
            # Trigger EFLS landing
            self.triggerEFLS = True
            
            print 'EFLS: Success'            
                

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        
        # Waypoint handling
        if self.expect_waypoints == 1:
            if m.get_type() in ['WAYPOINT_COUNT','MISSION_COUNT']:
                self.num_wps_expected = m.count
            if m.get_type() in ['WAYPOINT', 'MISSION_ITEM']:
                if (len(self.wps_received) < self.num_wps_expected):
                    if (m.seq not in self.wps_received.keys()):
                        self.wps_received[m.seq] = True
                        self.wploader.set(m, m.seq)
                    else:
                        self.expect_waypoints = 2
        
        # Aircraft message handling    
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
            

def init(mpstate):
    '''initialise module'''
    return efls(mpstate)
