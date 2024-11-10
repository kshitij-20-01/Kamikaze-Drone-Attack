#Code to classify object accurately using YOLO Model in Gazebo using ROS

import cv2
import cvzone
import geopy
import geopy.distance
import json
import numpy as np
import pandas
import rospy
import time
import torch

#from calc_gps import gps
from math import *
from sensor_msgs.msg import Image


net = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/rithwick11111/Desktop/GlobalHawks/Simulations/Kamikaze/Round_3/best.pt')


def conv(dataframe):
    cord = pandas.DataFrame(dataframe)
    cord_x = (cord['xmin'] + cord['xmax'])/2
    cord_y = (cord['ymin'] + cord['ymax'])/2
    cordinates = cord[['xmin','ymin','xmax','ymax']].values.tolist()
    name = cord['name'].values.tolist()
    conf = cord['confidence'].values.tolist()
    return cord_x.to_list()+cord_y.to_list(),cordinates, name, conf


def update_cordinates(cordinates):
    fileObject = open('/home/rithwick11111/Desktop/GlobalHawks/Simulations/Kamikaze/Round_3/coordinates.json', 'w+')
    jsonData = json.dumps(cordinates)
    fileObject.write(jsonData)
    fileObject.close()


def seejsonoutput():
    fileObject = open('/home/rithwick11111/Desktop/GlobalHawks/Simulations/Kamikaze/Round_3/coordinates.json', 'r+')
    output = json.load(fileObject)
    print(output)
    fileObject.close()


def object_detection(img):
    global results 
    results = net(img)
    centroid, cordinates, name, conf = conv(results.pandas().xyxy[0])
    try:
        for num in range(len(name)):
            c = np.array(cordinates[num])
            cv2.rectangle(img, (int(c[0]),int(c[1])), (int(c[2]),int(c[3])), (255, 0, 155), 2)
            cv2.putText(img, f' {name[num]} : {round(conf[num]*100,3)}', (int(c[num]-60), int(c[num]-140)), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 2)
            print("Object being detected is: ", name)
        cv2.imshow("Image", img)
        cv2.waitKey(1)
        if centroid==[]:
            update_cordinates([0,0])
            return [0,0]
        else:
            update_cordinates(centroid)
            return centroid
    except:
        cv2.imshow("Image", img)
        cv2.waitKey(1)
        return [0,0]


def callback(data):
    img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    a = object_detection(img)
    print('centroid coordinates:',a[0],a[1])


def display():
    rospy.init_node('display', anonymous=True)
    rospy.Subscriber('webcam/image_raw', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    display()
