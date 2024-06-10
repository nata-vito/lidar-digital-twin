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
import weakref
import random

actor_list = []


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    world_ref = weakref.ref(world)
    
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
        safe_sensor_bp = blueprint_library.find('sensor.other.safe_distance')
        user_offset = carla.Location(x, y, z)
        safe_sensor_sensor_transform = carla.Transform(carla.Location(x = -0.5, z = 1.8) + user_offset)
        safe_sensor = world.spawn_actor(safe_sensor_bp, safe_sensor_sensor_transform, attach_to = vehicle)
        actor_list.append(safe_sensor)                
        safe_sensor.listen(lambda data: callback(data, vehicle))
                   
    finally:
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        for actor in actor_list:
            actor.destroy()

        print("All cleaned up!")



def callback(event, vehicle):
    for actor_id in event:
        print("Vehicle too Close: %s" % vehicle.type_id)
    
    print('hi')


if __name__ == '__main__':
    main()
 
 