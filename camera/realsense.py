import os
import cv2
import pyrealsense2 as rs
import numpy as np
import time

class RealSense:
    def __init__(self, save_folder):
        self.save_folder = save_folder
        self.count = 0
        
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

        self.rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        
    def get_image(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        self.rgb = np.asanyarray(color_frame.get_data())

        save_path = os.path.join(self.save_folder, f'{self.count}.png')
        self.count += 1
        cv2.imwrite(save_path,  self.rgb)

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', self.rgb)
        cv2.waitKey(1)
        
        return save_path
        
if __name__ == '__main__':
    camera = RealSense()
    
    while True:
        time.sleep(0.1)
        camera.get_image()