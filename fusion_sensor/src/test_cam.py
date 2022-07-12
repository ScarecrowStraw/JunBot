import pyrealsense2 as rs
import numpy as np
import cv2

width = 640
height = 360

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

print("Depth Scale is: ", depth_scale)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth = depth_image[320,240].astype(float)*depth_scale

        cv2.imshow('rgb', color_image)
        cv2.imshow('depth', depth_colormap)
        print(f'Depth: {depth} m')

        if cv2.waitKey(1) == ord("q"):
            break
finally:
    pipeline.stop()