import carla

def parked_in_intersection(sim_bridge):
    CLIENT_WORLD = 'Town07'
    EGO_SPAWN_X = -180
    EGO_SPAWN_Y = -163
    EGO_SPAWN_Z = 20.0
    EGO_SPAWN_YAW = 90

    sim_bridge.world = sim_bridge.client.load_world(CLIENT_WORLD)
    setup_ego(sim_bridge, EGO_SPAWN_X, EGO_SPAWN_Y, EGO_SPAWN_Z, EGO_SPAWN_YAW)
    add_vehicles(sim_bridge, autopilot=False, spawn_points=[carla.Transform(carla.Location(x=-77.5, y=-158, z=20), carla.Rotation(yaw=90))])

def normal(sim_bridge):
    CLIENT_WORLD = 'Town07'
    EGO_SPAWN_X = -180
    EGO_SPAWN_Y = -163
    EGO_SPAWN_Z = 20.0
    EGO_SPAWN_YAW = 90

    sim_bridge.world = sim_bridge.client.load_world(CLIENT_WORLD)
    setup_ego(sim_bridge, EGO_SPAWN_X, EGO_SPAWN_Y, EGO_SPAWN_Z, EGO_SPAWN_YAW)
    add_vehicles(sim_bridge, [None]*10)
    add_pedestrians(sim_bridge, [None]*10)

def add_vehicles(sim_bridge, spawn_points=[None], autopilot=True):
    sim_bridge.get_logger().info("Spawning {} vehicles".format(vehicle_count))
    for spawn in spawn_points:
        # Choose a vehicle blueprint at random.
        vehicle_bp = random.choice(sim_bridge.blueprint_library.filter('vehicle.*.*'))
        
        if not spawn:
            #spawn randomly if no point is provided
            spawn = random.choice(sim_bridge.world.get_map().get_spawn_points())
        
        # spawn vehicle
        sim_bridge.get_logger().info("Spawning vehicle ({}) @ {}".format(vehicle_bp.id, spawn))
        vehicle = sim_bridge.world.try_spawn_actor(vehicle_bp, spawn)
        
        # autopilot off for now (junction code testing)
        if vehicle is not None:
            vehicle.set_autopilot(enabled=autopilot)

def add_pedestrians(sim_bridge, spawn_points = None, autopilot=True):
    sim_bridge.get_logger().info("Spawning {} pedestrians".format(count))

    for spawn in spawn_points:
        # Choose a vehicle blueprint at random.

        ped_bp = random.choice(sim_bridge.blueprint_library.filter('walker.*.*'))
        sim_bridge.get_logger().info("Spawning ped ({}) @ {}".format(ped_bp.id, spawn))
        
        if not spawn:
            #spawn randomly if no point is provided
            spawn = random.choice(sim_bridge.world.get_map().get_spawn_points())

        ped = sim_bridge.world.try_spawn_actor(ped_bp, spawn)
        if ped is not None:
             ped.set_autopilot(enabled=autopilot)

def setup_ego(sim_bridge, ego_x, ego_y, ego_z, ego_yaw, carla_autopilot = False, model='vehicle.audi.etron'):
    
    sim_bridge.blueprint_library = sim_bridge.world.get_blueprint_library()
    # Get random spawn point
    spawn_loc = carla.Location()
    spawn_loc.x = ego_x
    spawn_loc.y = ego_y
    spawn_loc.z = ego_z
    spawn_rot = carla.Rotation()
    spawn_rot.pitch = 0
    spawn_rot.roll = 0
    spawn_rot.yaw = ego_yaw
    spawn = carla.Transform()
    spawn.location = spawn_loc
    spawn.rotation = spawn_rot
    vehicle_bp = sim_bridge.blueprint_library.find(model)
    sim_bridge.ego: carla.Vehicle = sim_bridge.world.spawn_actor(vehicle_bp, spawn)
    # TODO: Destroy ego actor when node exits or crashes. Currently keeps actor alive in CARLA,
    # which eventually leads to memory overflow. WSH.
    sim_bridge.ego.set_autopilot(enabled=carla_autopilot)
    sim_bridge.add_ego_sensors()



    
    
