import cv2
import numpy as np
import pyrealsense2 as rs
from multiprocessing import Process

class CameraHandler:


    metersClippingDistance = 1 #1 meter


    def __init__(self, name = "Camera 1", type = "D435", width = 1280, height = 720, frameRate = 30):
        self.name = name
        self.type = type
        self.width = width
        self.height = height
        self.frameRate = frameRate
        self.pipeline = None
        self.profile = None
        self.depthSensor = None
        self.baseFrame = None
        self.aligner = None
        self.colorFramesBuffer = []
        self.depthFramesBuffer = []

    def start(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.frameRate)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.frameRate)

        # Start streaming
        self.profile = self.pipeline.start(config)

        self.depthSensor = self.profile.get_device().first_depth_sensor()

        if self.depthSensor.supports(rs.option.emitter_enabled):
            self.depthSensor.set_option(rs.option.emitter_enabled, 1.0)
            self.depthSensor.set_option(rs.option.depth_units, 0.00001)
            self.depthSensor.set_option(rs.option.enable_auto_exposure, False)

        #Recieve the frames and verify that they are of the correct type
        frames = self.pipeline.wait_for_frames()
        colorFrame = frames.get_color_frame()
        colorImage = np.asanyarray(colorFrame.get_data())
        print('Color Frame information')
        print(colorImage.shape)



        frames = self.pipeline.wait_for_frames()
        depthFrame = frames.get_depth_frame()
        depthImage = np.asanyarray(depthFrame.get_data())
        print('Depth Frame information')
        print(depthImage.shape)

        self.baseFrame = rs.stream.color
        self.aligner = rs.align(self.baseFrame)




    def getColorFrame(self):

        # frames = self.pipeline.wait_for_frames()
        if(len(self.colorFramesBuffer) > 0):
            print(',')
            colorFrame = self.colorFramesBuffer.__getitem__(0)
            colorImage = np.asanyarray(colorFrame.get_data())
            del self.colorFramesBuffer[0]
        else:
            frames = self.pipeline.wait_for_frames()
            self.depthFramesBuffer.append(frames.get_depth_frame())
            colorFrame = frames.get_color_frame()
            colorImage = np.asanyarray(colorFrame.get_data())

            print(len(self.depthFramesBuffer))
            # del self.framesBuffer[-1]

            # del self.framesBuffer[0]

        # colorFrame = self.framesBuffer.get_color_frame()




        return colorImage

    def getDepthFrame(self):
        # frames = self.pipeline.wait_for_frames()

        if(len(self.depthFramesBuffer) > 0):
            print(',')
            depthFrame = self.depthFramesBuffer.__getitem__(0)

            spatial = rs.spatial_filter()
            spatial.set_option(rs.option.filter_magnitude, 2)
            spatial.set_option(rs.option.filter_smooth_alpha, 1)
            spatial.set_option(rs.option.filter_smooth_delta, 50)
            spatial.set_option(rs.option.holes_fill, 2)

            # hole_filling = rs.hole_filling_filter()
            # hole_filling.set_option(rs.option.filter_magnitude, 2)

            thresh = rs.threshold_filter(0.1, 2)

            #
            # depthImage = cv2.bilateralFilter(depthImage,9,75,75)
            depthFrame = thresh.process(depthFrame)
            depthFrame = spatial.process(depthFrame)
            # depthFrame = hole_filling.process(depthFrame)
            depthImage = np.asanyarray(depthFrame.get_data())
            # depthImage = depthImage * 255

            # depthImage = cv2.medianBlur(depthImage,1)
            # depthImage = cv2.GaussianBlur(depthImage,(3,3),3)



            del self.depthFramesBuffer[0]
        else:
            frames = self.pipeline.wait_for_frames()
            self.colorFramesBuffer.append(frames.get_color_frame())

            depthFrame = frames.get_depth_frame()
            depthframe = rs.hole_filling_filter.process(depthFrame)
            depthImage = np.asanyarray(depthFrame.get_data())

            depthImage = cv2.GaussianBlur(depthImage,(11,11),5)
            depthImage = cv2.medianBlur(depthImage,51)
            depthImage = cv2.bilateralFilter(depthImage,9,75,75)






            print(len(self.depthFramesBuffer))
            # del self.framesBuffer[-1]

        depthColorMap = cv2.applyColorMap(cv2.convertScaleAbs(depthImage, alpha=0.003 ), cv2.COLORMAP_JET)

        return depthColorMap

    #Uses the depth frame in order to map the 3D coordinates into a readable 2D array
    def getPointsArray(self):

        #Distance at which a pixel is ignored
        clipping_distance = self.metersClippingDistance / self.depthSensor.get_depth_scale

        pointCloud = rs.pointcloud()
        points = rs.points


        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        alignedFrames = self.aligner.process(frames)

        # Get aligned frames
        alignedDepthFrame = alignedFrames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        colorFrame = alignedFrames.get_color_frame()

        depth_image = np.asanyarray(alignedDepthFrame.get_data())
        color_image = np.asanyarray(colorFrame.get_data())

        cv2.GaussianBlur(alignedDepthFrame,(5,5), 9, alignedDepthFrame)

        depth_image_3d = np.dstack((depth_image,depth_image,depth_image))

        # # Intrinsics & Extrinsics
        # depth_intrin = depthFrame.profile.as_video_stream_profile().intrinsics
        # color_intrin = colorFrame.profile.as_video_stream_profile().intrinsics
        # depth_to_color_extrin = depthFrame.profile.get_extrinsics_to(colorFrame.profile)
        #
        # # Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
        # depth_scale = self.depthSensor.get_depth_scale()
        #
        # #Create an instance of a point cloud and see what happens

        # points = pointCloud.calculate(depthFrame)
        # # pc.map_to(color_frame)
        # pointArray = np.asanyarray(points.get_vertices())
        # print('Point Frame information')
        # print(pointArray.shape)
        # print(pointArray[0])
        #
        #
        #
        # # Map depth to color
        #
        # depth_pixel = [240, 320]   # Random pixel
        # depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_scale)
        # color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
        # color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
        # pipeline.stop()




