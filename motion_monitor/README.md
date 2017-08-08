**Motion Camera**

This python script needs to have motion installed.

sudo apt install motion 

Add the following to the config.

on_picture_save \<path to motion_action.py\> pic %f

on_motion_detect \<path to motion_action.py\> motion
