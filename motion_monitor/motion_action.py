#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import simpleflock
import json
import datetime
import requests
from pushbullet import Pushbullet
from time import time

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

    if config['log_file']:
        logfile = config['log_file']
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



# Load config file
#------------------------------------------------------------------------------#
#__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
__location__ = '/home/squinn/Documents/projects/NetSecHome/motion_monitor/'
try:
    with open(os.path.join(__location__, 'motion.json')) as data_file:
        config = json.load(data_file)
except:
    log('Could not load configuration file %s' % os.path.join(__location__, 'motion.json'), 1)
    sys.exit(1)

# Globals
#------------------------------------------------------------------------------#
pb = Pushbullet(config['pushbullet_api'])
if not pb:
    log('Could not create Pushbullet instace', 1)

verbose = True
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


def time_diff(old, new=time()):
    if float(old) + float(config['time_delta']) > float(new):
        return False
    else:
        return True

def main():

    if sys.argv[1] == 'motion':
        log('Action [%s]' % sys.argv[1], 0)
        with simpleflock.SimpleFlock(config['motion_flock']):
            log('%s flock aquired' % config['motion_flock'], 0)
            if not arm_status():
                log('Not armed exiting', 0)
                return 0

            with open(config['last_motion']) as f:
                old_time = f.read()

            if time_diff(old_time):
                log('Last motion notify is greater than 5 minutes', 0)
                pb.push_note('Motion Cam', 'Motion Detcted')
                out = open(config['last_motion'], 'w')
                out.write('%s' % time())
                out.close()
            else:
                log('Duplicate action motion', 2)

    elif sys.argv[1] == 'pic':
        log('Action [%s]' % sys.argv[1], 0)
        with simpleflock.SimpleFlock(config['pic_flock']):
            log('%s flock aquired' % config['pic_flock'], 0)
            if not arm_status():
                log('Not armed exiting', 0)
                return 0

            with open(config['last_pic']) as f:
                old_time = f.read()

            if time_diff(old_time):
                log('Last pic notify is greater than 5 minutes', 0)
                if not sys.argv[2]:
                    log('No picture argument suplied extiting', 1)
                    return 1

                with open(sys.argv[2], 'rb') as pic:
                    file_data = pb.upload_file(pic, config['pic_name'])

                push = pb.push_file(**file_data)

                out = open(config['last_pic'], 'w')
                out.write('%s' % time())
                out.close()
            else:
                log('Duplicate action pic', 2)
    return 0

if __name__ == "__main__":
    main()
