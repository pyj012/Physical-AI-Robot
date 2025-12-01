import pyrealsense2 as rs
import numpy as np
import time
import cv2
import random
import sys
from .  create_marker_lib import * 
import os
class RealsenseCamera():
    def __init__(self):
        ############ REALSENSE#####
        # Configure depth and color streams
        self.images = None
        self.makernode = MarkerPublisher()
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def getFrame(self):
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            return color_image, depth_frame
        
        except Exception as e:
            print("realsense err", e)
            self.release()
        
    def getDistance(self, depth_frame, object_dict):
        try:
            object_pos_dict = {}
            x =0
            y =0
            d_z =0
            x_offset=-40.24/1000.0
           # -30.24
            y_offset=190/1000.0 
            z_offset=912.52/1000.0
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            if len(object_dict):
                for class_name in object_dict:
                    x=object_dict[class_name][0]
                    y=object_dict[class_name][1]
                    depth= depth_frame.get_distance(x,y) # depth 값을 얻는 과정
                    d_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x,y], depth)
                    _d_x,_d_y,_d_z=d_point
                    d_x = _d_x*100
                    d_y = _d_y*100  
                    d_z = round(_d_z*100,1)
                    cut_distance= 70#cm
                    if d_z <cut_distance:
                        maker_x= ((-d_x/100.0) - (x_offset))
                        maker_y= ((-d_z/100.0) + (y_offset))
                        maker_z= ((-d_y/100.0) + (z_offset))
                        object_pos_dict[class_name]=[round(maker_x,3),round(maker_y,3),round(maker_z,3)]
                        self.makernode.create_maker(maker_x, maker_y, maker_z)
            else :
                pass
                
            return object_pos_dict
        
        except Exception as e:
            print("getdistance err",e)
            self.release()

    def release(self):
        self.pipeline.stop()
