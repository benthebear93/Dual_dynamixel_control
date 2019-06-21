#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
import serial
import time

st = serial.Serial("/dev/ttyUSB0",100000)


if __name__ == '__main__':
    try:
        st.write(b'\xff\xff\x01\x04\x03\x46\x01\xB0')
        time.sleep(0.1)
        st.write(b'\xff\xff\x01\x04\x03\x18\x01\xDE')
        time.sleep(0.1)
        st.write(b'\xff\xff\x01\x04\x03\x47\x01\x00\xAE')
        time.sleep(0.1)

        st.write(b'\xff\xff\x02\x04\x03\x46\x01\xAF')
        time.sleep(0.1)
        st.write(b'\xff\xff\x02\x04\x03\x18\x01\xDD')
        time.sleep(0.1)
        st.write(b'\xff\xff\x02\x05\x03\x47\x01\x00\xAD')

        st.close()
    except rospy.ROSInterruptException:
        pass