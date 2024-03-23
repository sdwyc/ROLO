#! /usr/bin/python
import rospy
from std_msgs.msg import Float64MultiArray
# import numpy as np
import matplotlib.pyplot as plt

def dataHandler(data):
    plot_data = data.data
    print(len(data.data))
    deltT = range(len(plot_data))
    
    fig = plt.figure(0)
    fig.clf()
    plt.plot(deltT, plot_data)
    plt.xlabel('time')
    plt.ylabel('resual')
    plt.draw()
    plt.pause(0.1)
    if(rospy.is_shutdown()):
        rospy.signal_shutdown()

if __name__ == '__main__':
    rospy.init_node("rolo_sam_plot", anonymous=False)
    sub = rospy.Subscriber("rolo/data_test", Float64MultiArray, callback=dataHandler, queue_size=10)
    plt.ion()
    rospy.spin()