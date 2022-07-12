import pyrealsense2 as rs
import numpy as np
import cv2

width = 640
height = 360

config = rs.config()
config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
pipeline = rs.pipeline()

pipe_profile = pipeline.start(config)

depth_sensor = pipe_profile.get_device().first_depth_sensor()
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

        cv2.imshow('rgb', color_image)
        cv2.imshow('depth', depth_image)

        # Intrinsics & Extrinsics
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
        color_to_depth_extrin = color_frame.profile.get_extrinsics_to(depth_frame.profile)
        # print("\n Depth intrinsics: " + str(depth_intrin))
        # print("\n Color intrinsics: " + str(color_intrin))
        # print("\n Depth to color extrinsics: " + str(depth_to_color_extrin))

        depth_sensor = pipe_profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        # print("\n\t depth_scale: " + str(depth_scale))
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_pixel = [200, 200] # Random pixel
        depth_value = depth_image[200][200]*depth_scale
        # print("\n\t depth_pixel@" + str(depth_pixel) + " value: " + str(depth_value) + " meter")

        depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_value)
        print("\n\t 3D depth_point: " + str(depth_point))

        color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
        print("\n\t 3D color_point: " + str(color_point))


        color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
        print("\n\t color_pixel: " + str(color_pixel))
        # color_ = np.asanyarray(color_pixel.get_data())
        # cv2.imshow('rgbd', color_)

        if cv2.waitKey(1) == ord("q"):
            break

finally:
    pipeline.stop()
