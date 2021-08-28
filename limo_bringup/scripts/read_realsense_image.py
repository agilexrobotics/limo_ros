#!/bin/python3

import pyrealsense2 as rs
import numpy as np
import cv2


class ReadImage():
    def __init__(self):
        self.pipeline_ = rs.pipeline()
        self.config_ = rs.config()
        self.pipeline_wrapper_ = rs.pipeline_wrapper(self.pipeline_)
        self.pipeline_profile_ = self.config_.resolve(self.pipeline_wrapper_)
        self.device_ = self.pipeline_profile_.get_device()
        self.device_product_line_ = str(
            self.device_.get_info(rs.camera_info.product_line))
        self.found_rgb_ = False

    def read(self):
        for s in self.device_.sensors:
            if(s.get_info(rs.camera_info.name)) == 'RGB Camera':
                self.found_rgb_ = True
                break

        if not self.found_rgb_:
            print('require rgb image sensor')
            return None

        self.config_.enable_stream(
            rs.stream.depth, 640, 480, rs.format.z16, 30)

        if self.device_product_line_ == 'L500':
            self.config_.enable_stream(
                rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config_.enable_stream(
                rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline_.start(self.config_)

        try:
            for i in range(30):
                frames = self.pipeline_.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                if not depth_frame or not color_frame:
                    return None

                color_image = np.asanyarray(color_frame.get_data())
        finally:
            self.pipeline_.stop()

        return color_image


if __name__ == "__main__":
    read_img = ReadImage()
    image = read_img.read()

    if image is not None:
        cv2.namedWindow('Realsense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Realsense', image)
        cv2.waitKey(0)
    else:
        print("can not read image")

# reference: https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py
