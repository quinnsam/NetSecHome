#!/usr/bin/env python
# -*- coding: utf-8 -*-
################################################################################
#                                                                              #
#       _   _      _   ____            _   _                                   #
#      | \ | | ___| |_/ ___|  ___  ___| | | | ___  _ __ ___   ___              #
#      |  \| |/ _ \ __\___ \ / _ \/ __| |_| |/ _ \| '_ ` _ \ / _ \             #
#      | |\  |  __/ |_ ___) |  __/ (__|  _  | (_) | | | | | |  __/             #
#      |_| \_|\___|\__|____/ \___|\___|_| |_|\___/|_| |_| |_|\___|             #
#                                                                              #
# Created By: Sam Quinn                                                        #
# Date: 03/21/2017                                                             #
#                                                                              #
################################################################################

import time
import threading
import os
import sys
import json
import requests
import datetime

from pushbullet import Pushbullet
from requests.packages.urllib3.exceptions import InsecureRequestWarning

# Load config file
#------------------------------------------------------------------------------#
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
try:
    with open(os.path.join(__location__, 'netsechome.json')) as data_file:
        config = json.load(data_file)
except:
    log('Could not load configuration file %s' % os.path.join(__location__, 'netsechome.json'), 1)
    sys.exit(1)

# Globals
#------------------------------------------------------------------------------#
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)
pb = Pushbullet(config['pushbullet_key'])

verbose = False

#------------------------------------------------------------------------------#
# This function will print any message to the console and to a log file with a
# priority and the date + time.
#
# Input:  String to print, priority level
# Output: 1 = Error, 0 = Pass
#------------------------------------------------------------------------------#
def log(msg, pri):
    temp = ''
    time = datetime.datetime.now()

    if config['logfile']:
        logfile = config['logfile']
    else:
        print '%s:%s' % (pri,msg)
        return 1

    if pri == 0:
        temp = '[info]\t%s - %s' % (time,msg)
    elif pri == 1:
        temp = '[error]\t%s - %s' % (time,msg)
    elif pri == 2:
        temp = '[warn]\t%s - %s' % (time,msg)
    else:
        temp = '[%s]\t%s - %s' % (pri,time, msg)

    with open(logfile, 'a') as logf:
        logf.write('%s\n' % temp)

    if verbose: print temp
    return 0

#------------------------------------------------------------------------------#
# This function is used to print json in a nice format.
#
# Input:  JSON string
# Output: None
#------------------------------------------------------------------------------#
def pp_json(json_thing, sort=True, indents=4):
    if type(json_thing) is str:
        print(json.dumps(json.loads(json_thing), sort_keys=sort, indent=indents))
    else:
        print(json.dumps(json_thing, sort_keys=sort, indent=indents))
    return None

#------------------------------------------------------------------------------#
# This function will login to the Unifi API and generate a cookie that will be
# used for the rest of the transactions.
#
# Input:  None
# Output: Cookie
#------------------------------------------------------------------------------#
def api_login():
    log('Logging into API', 0)
    try:
        login = requests.post('%s/api/login' % config['unifi_server'], data="{ 'username': '%s', 'password': '%s' }" % (config['username'], config['password']), verify=False)
    except:
        return None
    if login.status_code == 200:
        log('Login Successfull', 0)
        cookie = login.cookies
    else:
        log('Login FAILED [%s]' % login.status_code, 1)
        return None

    return cookie

#------------------------------------------------------------------------------#
# This function will just return True if the current time is within the "Alert
# Time" or not.
#
# Input:  None
# Output: True = Alert time now, False = Not Alert time now
#------------------------------------------------------------------------------#
def alert_time():
    if datetime.datetime.now().hour >= 23 or datetime.datetime.now().hour < 7:
        return True
    else:
        return False

#------------------------------------------------------------------------------#
# This function will monitor the motion sensor to see if we are still in the
# living room. If we are then it will not arm the house yet. If there hasen't
# been any movement for 15 minutes then the house will be armed.
#
# Input:  None
# Output: 1 = Armed, 0 = Not Armed
#------------------------------------------------------------------------------#
def check_activity():
    log('Started check_activity', 0)
    motion_count = 0
    wait = time.time()
    while alert_time():
        if wait + 900 < time.time():
            log('Time is up Checking for movement', 0)
            if motion_count == 0:
                log('No movement seen', 0)
                arm_house()
                return 1 # Return so this doesn't keep monitoring PIR Sensor
            wait = time.time()
            motion_count = 0
        time.sleep(5)
        try:
            m = requests.get('%s/get/V3' % config['blynk_api'])
        except:
            return 0
        if (int(m.json()[0]) == 0):
            motion_count = motion_count + 1
        time.sleep(2)
    return 0

#------------------------------------------------------------------------------#
# This function will return the arm status from the Blynk API
#
# Input:  None
# Output: True = Armed, False = Disarmed, None = Error
#------------------------------------------------------------------------------#
def arm_status():
    try:
        arm = requests.get('%s/get/V0' % config['blynk_api'])
    except:
        return False
    if arm.status_code == 200:
        if int(arm.json()[0]) == 1:
            return True
        elif int(arm.json()[0]) == 0:
            return False
    else:
        log('Got a bad response code [%s] from Blynk API' % arm.status_code, 1)
        return None


#------------------------------------------------------------------------------#
# This function will arm the house.
#
# Input:  (Optional) Message to send in the Pushbullet message
# Output: 0 = Pass, 1 = Error
#------------------------------------------------------------------------------#
def arm_house(msg=''):
    #pb.push_note('[ARMING] home', '%s' % msg)
    try:
        arm = requests.get('%s/update/V0?value=1' % config['blynk_api'])
    except:
        return 1
    if arm.status_code == 200:
        log('Home ARMED', 0)
        pb.push_note('Home ARMED', '[%s]%s' % (arm.status_code, msg))
        return 0
    else:
        log('Got a bad response code [%s] from Blynk API' % arm.status_code, 1)
        return 1

#------------------------------------------------------------------------------#
# This function will disarm the house.
#
# Input:  (Optional) Message to send in the Pushbullet message
# Output: 0 = Pass, 1 = Error
#------------------------------------------------------------------------------#
def disarm_house(msg=''):
    #pb.push_note('[DISARMING] home', '%s' % msg)
    try:
        arm = requests.get('%s/update/V0?value=0' % config['blynk_api'])
    except:
        return 1
    if arm.status_code == 200:
        log('Home DISARMED', 0)
        pb.push_note('Home DISARMED', '[%s]%s' % (arm.status_code, msg))
        return 0
    else:
        log('Got a bad response code [%s] from Blynk API' % arm.status_code, 1)
        return 1


#------------------------------------------------------------------------------#
# This function monitors the Unifi API for users connecting or disconnecting.
# If no users are connected then it will arm the house.
#
# Input:  Unifi Cookie
# Output: None (Should not exit)
#------------------------------------------------------------------------------#
def api(cookie):
    data="{ 'username': '%s', 'password': '%s' }" % (config['username'], config['password'])
    log('Started API thread', 0)
    #pb.push_note('Started API watcher', '')
    monitor_activity = True
    connected = []

    while True:
        try:
            r = requests.post('%s/api/s/default/stat/sta' % config['unifi_server'], data=data, verify=False, cookies=cookie)
        except:
            pb.push_note('Got a bad response from Unifi', 'With in main API loop')
            time.sleep(30)
            continue
        if r.status_code == 200:
            #log('API called successfull', 0)
            data = r.json()
            #pp_json(data)
            found = False
            # All users in config file
            for user in config['user_id']:
                # All devices returned by Unifi
                for device in data['data']:
                    if config['user_id'][user] in device['user_id']:
                        found = True
                        if user not in connected:
                            log('Found %s' % user, 0)
                            connected.append(user)
                        # If last seen from Unifi is greater than 15 minutes assume  user left
                        if int(device['last_seen']) + 900 < time.time():
                            log('%s left' % user, 2)
                            pb.push_note('%s has left' % user, 'at %s' % user['last_seen'])
                if not found:
                    # Couldn't find user
                    if user in connected: connected.remove(user)
        else:
            log('Got a bad response code [%s] from Unifi API' % r.status_code, 1)
        if len(connected) == 0:
            if not arm_status():
                log('Nobody is home', 0)
                arm_house('Nobody is home')
        else:
            # Some one is home
            if (alert_time()):
                # During the Alert Time [23:00 - 07:00]
                if monitor_activity:
                    # Start Monitor activity Thread if NO activity is found then lock the house.
                    activity = threading.Thread(target=check_activity, args = [])
                    activity.daemon = True
                    activity.start()
                    monitor_activity = False # Set to False so this only starts once.
            else:
                # If someone is home and it is not within "Alert Time" then disarm the house.
                monitor_activity = True # Reset the monitor activity trigger.
                if arm_status():
                    log('Disarming since someone is home and not Alert Time', 0)
                    disarm_house()

        time.sleep(10)



#------------------------------------------------------------------------------#
# The main function will just call the API daemon.
#
# Input:  None
# Output: None
#------------------------------------------------------------------------------#
def main():
    global verbose
    verbose = True
    #api_thread = threading.Thread(target=api, args = [verbose])
    #api_thread.daemon = True
    #api_thread.start()
    cookie = api_login()
    if cookie:
        api(cookie)


if __name__ == "__main__":
    main()

