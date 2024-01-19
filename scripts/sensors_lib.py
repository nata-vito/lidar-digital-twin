import cv2 as cv
import numpy as np
import open3d as o3d
from matplotlib import cm

class Camera:
    def __init__(self) -> None:
        self.cam_img_width = 640
        self.cam_img_height = 480
    
    def set_camera(self, carla, actor_list, blueprint_library, world, vehicle):
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x", "{}".format(self.cam_img_width))
        cam_bp.set_attribute("image_size_y", "{}".format(self.cam_img_height))
        cam_bp.set_attribute("fov", "110")
        
        spawn_point = carla.Transform(carla.Location(x = 2.5, z = 0.7))

        sensor = world.spawn_actor(cam_bp, spawn_point, attach_to = vehicle)
        actor_list.append(sensor)
        sensor.listen(lambda data: self.process_img(data))
        
        return cam_bp
    
    def process_img(self, image):
        i = np.array(image.raw_data)
        print(i.shape)
        
        i2 = i.reshape((self.cam_img_height, self.cam_img_width, 4))
        i3 = i2[:, :, :3]
        
        cv.imshow("", i3)
        cv.waitKey(1)
        
        return i3/255

class Lidar:
    def __init__(self) -> None:
        self.semantic = False
        self.no_noise = True
        self.noise_stddev = 0.2
        self.channels = 32
        self.range = 1000
        self.points_per_second = 56000
        self.rotation_frequency = 10.0
        self.upper_fov = 10.0
        self.lower_fov = -30.0
        self.sensor_trick = 0.0
        self.dropoff_general_rate = 0.0
        self.dropoff_intensity_limit = 1.0
        self.dropoff_zero_intensity = 0.0
        self.viridis = np.array(cm.get_cmap('plasma').colors)
        self.vid_range = np.linspace(0.0, 1.0, self.viridis.shape[0])
        
        self.label_colors = np.array([
                            (255, 255, 255), # None
                            (70, 70, 70),    # Building
                            (100, 40, 40),   # Fences
                            (55, 90, 80),    # Other
                            (220, 20, 60),   # Pedestrian
                            (153, 153, 153), # Pole
                            (157, 234, 50),  # RoadLines
                            (128, 64, 128),  # Road
                            (244, 35, 232),  # Sidewalk
                            (107, 142, 35),  # Vegetation
                            (0, 0, 142),     # Vehicle
                            (102, 102, 156), # Wall
                            (220, 220, 0),   # TrafficSign
                            (70, 130, 180),  # Sky
                            (81, 0, 81),     # Ground
                            (150, 100, 100), # Bridge
                            (230, 150, 140), # RailTrack
                            (180, 165, 180), # GuardRail
                            (250, 170, 30),  # TrafficLight
                            (110, 190, 160), # Static
                            (170, 120, 50),  # Dynamic
                            (45, 60, 150),   # Water
                            (145, 170, 100), # Terrain
                        ]) / 255.0 # normalize each channel [0-1] since is what Open3D uses
        
    # Create to generate Lidar    
    def generate_lidar_bp(self, blueprint_library, delta):
        
        if self.semantic:
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')  
        else:
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')    

            if self.no_noise:
                lidar_bp.set_attribute('dropoff_general_rate', f'{self.dropoff_general_rate}')
                lidar_bp.set_attribute('dropoff_intensity_limit', f'{self.dropoff_intensity_limit}')
                lidar_bp.set_attribute('dropoff_zero_intensity', f'{self.dropoff_zero_intensity}')
            else:
                lidar_bp.set_attribute('noise_stddev', f'{self.self.noise_stddev}')

        lidar_bp.set_attribute('upper_fov', str(self.upper_fov))
        lidar_bp.set_attribute('lower_fov', str(self.lower_fov))
        lidar_bp.set_attribute('channels', str(self.channels))
        lidar_bp.set_attribute('range', str(self.range))
        lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
        lidar_bp.set_attribute('points_per_second', str(self.points_per_second))

        return lidar_bp
    
    
    def lidar_callback(self, point_cloud, point_list):
        """Prepares a point cloud with intensity
        colors ready to be consumed by Open3D"""
        data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        # Isolate the intensity and compute a color for it
        intensity = data[:, -1]
        intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
        int_color = np.c_[
            np.interp(intensity_col, self.vid_range, self.viridis[:, 0]),
            np.interp(intensity_col, self.vid_range, self.viridis[:, 1]),
            np.interp(intensity_col, self.vid_range, self.viridis[:, 2])]

        # Isolate the 3D data
        points = data[:, :-1]

        # We're negating the y to correclty visualize a world that matches
        # what we see in Unreal since Open3D uses a right-handed coordinate system
        points[:, :1] = -points[:, :1]

        # # An example of converting points from sensor to vehicle space if we had
        # # a carla.Transform variable named "tran":
        # points = np.append(points, np.ones((points.shape[0], 1)), axis=1)
        # points = np.dot(tran.get_matrix(), points.T).T
        # points = points[:, :-1]

        point_list.points = o3d.utility.Vector3dVector(points)
        point_list.colors = o3d.utility.Vector3dVector(int_color)
        
        
    def semantic_lidar_callback(self, point_cloud, point_list):
        data = np.frombuffer(point_cloud.raw_data, dtype = np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('cosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)]))
        
        points = np.array([data['x'], -data['y'], data['z']]).T

        labels = np.array(data['ObjTag'])
        int_color = self.label_colors[labels]    
        
        point_list.points = o3d.utility.Vector3dVector(points)
        point_list.colors = o3d.utility.vector3dVector(int_color)
        
    def add_open3d_axis(self, vis):
        axis = o3d.geometry.LineSet()

        axis.points = o3d.utility.Vector3dVector(np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]))

        axis.lines = o3d.utility.Vector2iVector(np.array([
            [0, 1],
            [0, 2],
        ]))

        axis.colors = o3d.utility.Vector3dVector(np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]))

        vis.add_geometry(axis)