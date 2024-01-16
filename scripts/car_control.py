def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    
    spawn_points = world.get_map().get_spawn_points()
    
    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
    
    spectator = world.get_spectator()
    transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x = -4, z = 2.5)), vehicle.get_transform().rotation)
    spectator.set_transform(transform)
    
    # Add random vehicles in map
    for i in range(30):
        vehicle_bp = random.choice(bp_lib.filter('vehicle'))
        npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
        
        if npc:
            npc.set_autopilot(True)
            print('created %s' % npc.type_id)
        
    camera_sensor(bp_lib, world, vehicle)
        

def camera_sensor(bp_lib, world, vehicle):
    camera_bp  = bp_lib.find("sensor.camera.rgb")
    camera_init_trans = carla.Transform(carla.Location(z = 2))
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to = vehicle)
    camera.listen(lambda image: image.save_to_disk('./out/%06d.png'% image.frame))
    print('here')
    
if __name__ == '__main__':
    main()