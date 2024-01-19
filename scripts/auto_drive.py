import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

import random
import time
import numpy as np
from datetime import datetime
import cv2 as cv
from matplotlib import cm
import open3d as o3d

actor_list = []
cam_img_width = 640
cam_img_height = 480

lidar_args = {
    'semantic': False,
    'no_noise': True,
    'noise_stddev': '0.2',
    'channels': 128,
    'range': 1000,
    'points_per_second': 56000,
    'rotation_frequency': 10.0,
    'upper_fov': 10.0,
    'lower_fov': -30.0,
    'sensor_tick': 0.0,
    'dropoff_general_rate': '0.0',
    'dropoff_intensity_limit': '1.0',
    'dropoff_zero_intensity': '0.0'
}

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

LABEL_COLORS = np.array([
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

def process_img(image):
    i = np.array(image.raw_data)
    print(i.shape)
    i2 = i.reshape((cam_img_height, cam_img_width, 4))
    i3 = i2[:, :, :3]
    cv.imshow("", i3)
    cv.waitKey(1)
    return i3/255


def set_camera(blueprint_library, world, vehicle):
    cam_bp = blueprint_library.find('sensor.camera.rgb')
    cam_bp.set_attribute("image_size_x", "{}".format(cam_img_width))
    cam_bp.set_attribute("image_size_y", "{}".format(cam_img_height))
    cam_bp.set_attribute("fov", "110")
    
    spawn_point = carla.Transform(carla.Location(x = 2.5, z = 0.7))

    sensor = world.spawn_actor(cam_bp, spawn_point, attach_to = vehicle)
    actor_list.append(sensor)
    sensor.listen(lambda data: process_img(data))
    
    return cam_bp
    
# Create to generate Lidar    
def generate_lidar_bp(lidar_args, blueprint_library, world, vehicle, delta):
    
    if lidar_args['semantic']:
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')  
    else:
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')    

        if lidar_args['no_noise']:
            lidar_bp.set_attribute('dropoff_general_rate', lidar_args['dropoff_general_rate'])
            lidar_bp.set_attribute('dropoff_intensity_limit', lidar_args['dropoff_intensity_limit'])
            lidar_bp.set_attribute('dropoff_zero_intensity', lidar_args['dropoff_zero_intensity'])
        else:
            lidar_bp.set_attribute('noise_stddev', lidar_args['noise_stddev'])

    lidar_bp.set_attribute('upper_fov', str(lidar_args["upper_fov"]))
    lidar_bp.set_attribute('lower_fov', str(lidar_args["lower_fov"]))
    lidar_bp.set_attribute('channels', str(lidar_args["channels"]))
    lidar_bp.set_attribute('range', str(lidar_args["range"]))
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(lidar_args["points_per_second"]))

    return lidar_bp


def lidar_callback(point_cloud, point_list):
    """Prepares a point cloud with intensity
    colors ready to be consumed by Open3D"""
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    # Isolate the intensity and compute a color for it
    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

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


def semantic_lidar_callback(point_cloud, point_list):
    data = np.frombuffer(point_cloud.raw_data, dtype = np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('cosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)]))
    
    points = np.array([data['x'], -data['y'], data['z']]).T

    labels = np.array(data['ObjTag'])
    int_color = LABEL_COLORS[labels]    
    
    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.vector3dVector(int_color)
    
    
def add_open3d_axis(vis):
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


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    
    try:
        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        delta = 0.05
        x, y, z = 0.0, 0.0, 0.0
        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        world.apply_settings(settings)
        
        blueprint_library = world.get_blueprint_library()

        bp = blueprint_library.filter("model3")[0]
        print(bp)

        spawn_point = random.choice(world.get_map().get_spawn_points())

        vehicle = world.spawn_actor(bp, spawn_point)
        vehicle.set_autopilot(True)

        vehicle.apply_control(carla.VehicleControl(throttle = 1.0, steer = 0.0))
        actor_list.append(vehicle)

        #cam_bp = set_camera(blueprint_library, world, vehicle)
        
        # Lidar Setup
        lidar_bp = generate_lidar_bp(lidar_args, blueprint_library, world, vehicle, delta)
        user_offset = carla.Location(x, y, z)
        lidar_transform = carla.Transform(carla.Location(x = -0.5, z = 1.8) + user_offset)
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to = vehicle)
        actor_list.append(lidar)
        
        point_list = o3d.geometry.PointCloud()
        
        if lidar_args['semantic']:
            lidar.listen(lambda data: semantic_lidar_callback(data, point_list))
        else:
            lidar.listen(lambda data: lidar_callback(data, point_list))
        
        
        # To do: 3d Point Cloud visualizer
        vis = o3d.visualization.Visualizer()
        
        vis.create_window(
            window_name = 'Carla Lidar',
            width = 960,
            height = 540,
            left = 480,
            top = 270
        )
        
        vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        vis.get_render_option().point_size = 1
        vis.get_render_option().show_coordinate_frame = True
        
        show_axis = True     
        
        if show_axis:
            add_open3d_axis(vis)
        
        frame = 0
        dt0 = datetime.now()
        
        while True:
            if frame == 2:
                vis.add_geometry(point_list)
            vis.update_renderer()
            
            vis.poll_events()
            vis.update_renderer()
            
            time.sleep(0.005)
            world.tick()
            
            process_time = datetime.now() - dt0
            sys.stdout.write('r' + 'FPS: ' + str(1.0 / process_time.total_seconds()))
            sys.stdout.flush()
            dt0 = datetime.now()
            frame += 1
            
    finally:
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        for actor in actor_list:
            actor.destroy()

        vis.destroy_window()
        
        print("All cleaned up!")


if __name__ == '__main__':
    main()