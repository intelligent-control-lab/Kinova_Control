import os, time


def launch_windows():
    os.system("gnome-terminal -e 'bash -c \"roscore; exec bash \"'")
    time.sleep(3)
    os.system("gnome-terminal -e 'bash -c \"rosrun kortex_driver kortex_driver 192.168.1.10 100; exec bash \"'")
    os.system("gnome-terminal -e 'bash -c \"roslaunch kinova_vision kinova_vision.launch; exec bash \"'")
    time.sleep(3)
    os.system("gnome-terminal -e 'bash -c \"rosrun image_view image_view image:=/camera/color/image_raw; exec bash \"'")
