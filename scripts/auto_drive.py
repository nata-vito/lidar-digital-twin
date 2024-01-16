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
import cv2 as cv


img_width = 640
img_height = 480


def process_img(image):
    i = np.array(image.raw_data)
    print(i.shape)
    i2 = i.reshape((img_height, img_width, 4))
    i3 = i2[:, :, :3]
    cv.imshow("", i3)
    cv.waitKey(1)
    return i3/255

actor_list = []
    
try:
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    
    bp = blueprint_library.filter("model3")[0]
    print(bp)
    
    spawn_point = random.choice(world.get_map().get_spawn_points())
    
    vehicle = world.spawn_actor(bp, spawn_point)
    #vehicle.set_autopilot(True)
    
    vehicle.apply_control(carla.VehicleControl(throttle = 1.0, steer = 0.0))
    actor_list.append(vehicle)
    
    cam_bp = blueprint_library.find('sensor.camera.rgb')
    cam_bp.set_attribute("image_size_x", "{}".format(img_width))
    cam_bp.set_attribute("image_size_y", "{}".format(img_height))
    cam_bp.set_attribute("fov", "110")
    
    
    spawn_point = carla.Transform(carla.Location(x = 2.5, z = 0.7))
    
    sensor = world.spawn_actor(cam_bp, spawn_point, attach_to = vehicle)
    actor_list.append(sensor)
    sensor.listen(lambda data: process_img(data))
    
    time.sleep(5)
finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")
