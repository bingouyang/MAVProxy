import time
import struct
import copy
import json
import os
from pymavlink import mavutil, mavwp
from MAVProxy.modules.lib import mp_module, mp_util, mp_settings

def has_location(hmodule, cmd_id):
    '''return True if a WP command has a location'''
    if cmd_id in mavutil.mavlink.enums['MAV_CMD'].keys():
        cmd_enum = mavutil.mavlink.enums['MAV_CMD'][cmd_id]
        # default to having location for older installs of pymavlink
        # which don't have the attribute
        return getattr(cmd_enum, 'has_location', True)
    return False

def wp_to_mission_item_int(hmodule, wp):
    '''convert a MISSION_ITEM to a MISSION_ITEM_INT. We always send as
        MISSION_ITEM_INT to give cm level accuracy
    '''
    if wp.get_type() == 'MISSION_ITEM_INT':
        return wp
    if has_location(hmodule, wp.command):
        p5 = int(wp.x*1.0e7)
        p6 = int(wp.y*1.0e7)
    else:
        p5 = int(wp.x)
        p6 = int(wp.y)
    wp_int = mavutil.mavlink.MAVLink_mission_item_int_message(
        wp.target_system,
        wp.target_component,
        wp.seq,
        wp.frame,
        wp.command,
        wp.current,
        wp.autocontinue,
        wp.param1,
        wp.param2,
        wp.param3,
        wp.param4,
        p5,
        p6,
        wp.z
    )
    return wp_int

def process_waypoint_request(hmodule, m, master):
    '''process a waypoint request from the master'''
    if (m.target_system != hmodule.settings.source_system or
            m.target_component != hmodule.settings.source_component):
        # hmodule.console.error("Mission request is not for me")
        return
    if (not hmodule.loading_waypoints or
            time.time() > hmodule.loading_waypoint_lasttime + 10.0):
        hmodule.loading_waypoints = False
        # hmodule.console.error("not loading waypoints")
        return
    if m.seq >= hmodule.wploader.count():
        hmodule.console.error("Request for bad waypoint %u (max %u)" % (m.seq, hmodule.wploader.count()))
        return
    wp = hmodule.wploader.wp(m.seq)
    wp.target_system = hmodule.target_system
    wp.target_component = hmodule.target_component
    if hmodule.settings.wp_use_mission_int:
        wp_send = wp_to_mission_item_int(hmodule, wp)
    else:
        wp_send = wp
    hmodule.master.mav.send(wp_send)
    hmodule.loading_waypoint_lasttime = time.time()
    hmodule.mpstate.console.set_status('Mission', 'Mission %u/%u' % (m.seq, hmodule.wploader.count()-1))
    if m.seq == hmodule.wploader.count() - 1:
        hmodule.loading_waypoints = False
        print("Loaded %u waypoint in %.2fs" % (hmodule.wploader.count(), time.time() - hmodule.upload_start))
        hmodule.console.writeln("Sent all %u waypoints" % hmodule.wploader.count())

def send_all_waypoints(hmodule):
    '''send all waypoints to vehicle'''
    hmodule.upload_start = time.time()
    hmodule.master.waypoint_clear_all_send()
    if hmodule.wploader.count() == 0:
        return
    hmodule.loading_waypoints = True
    hmodule.loading_waypoint_lasttime = time.time()
    hmodule.master.waypoint_count_send(hmodule.wploader.count())

def load_waypoints(hmodule, filename):
    '''load waypoints from a file'''
    hmodule.wploader.target_system = hmodule.target_system
    hmodule.wploader.target_component = hmodule.target_component
    try:
        # need to remove the leading and trailing quotes in filename
        hmodule.wploader.load(filename.strip('"'))
    except Exception as msg:
        print("Unable to load %s - %s" % (filename, msg))
        return
    print("Loaded %u waypoints from %s" % (hmodule.wploader.count(), filename))
    send_all_waypoints(hmodule)