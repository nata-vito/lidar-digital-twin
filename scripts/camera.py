import carla
import random
import sensors_lib as sensors

def main():
    actor_list = []
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)
        
    world = client.get_world()
        
    try:
        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        
        delta = 0.05

        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        settings.no_rendering_mode = True
        world.apply_settings(settings)
        
        blueprint_library = world.get_blueprint_library()
        
        # Car
        vehicle_bp = blueprint_library.filter('model3')[0]
        vehicle_transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        actor_list.append(vehicle)
        
        vehicle.set_autopilot(True)
        
        camera_bp = blueprint_library.find('sensor.camera.depth')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        actor_list.append(camera)
        print('created %s' % camera.type_id)

        # Now we register the function that will be called each time the sensor
        # receives an image. In this example we are saving the image to disk
        # converting the pixels to gray-scale.
        cc = carla.ColorConverter.LogarithmicDepth
        camera.listen(lambda image: image.save_to_disk('_out/%06d.png' % image.frame, cc))
        
    finally:
        print('destroying actors')

        camera.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('done.')

if __name__=="__main__":
    main()