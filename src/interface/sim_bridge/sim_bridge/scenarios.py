import carla
import random

class ScenarioManager:

    def __init__(self, sim_bridge):
        self.sim_bridge = sim_bridge
        self.world = 'Town07'
        self.ego_spawn = (-180, -163, 20)
        self.ego_yaw = 90

    def reset_vars(self):
        self.ego_spawn = (-180, -163, 20)
        self.ego_yaw = 90

    def add_vehicles(self, spawn_points=[None], autopilot=True):
        self.sim_bridge.get_logger().info("Spawning {} vehicles".format(len(spawn_points)))
        for spawn in spawn_points:
            # Choose a vehicle blueprint at random.
            vehicle_bp = random.choice(self.sim_bridge.blueprint_library.filter('vehicle.*.*'))
            
            if not spawn:
                #spawn randomly if no point is provided
                spawn = random.choice(self.sim_bridge.world.get_map().get_spawn_points())
            
            # spawn vehicle
            self.sim_bridge.get_logger().info("Spawning vehicle ({}) @ {}".format(vehicle_bp.id, spawn))
            vehicle = self.sim_bridge.world.try_spawn_actor(vehicle_bp, spawn)
            
            # autopilot off for now (junction code testing)
            if vehicle is not None:
                vehicle.set_autopilot(enabled=autopilot)        

    def add_pedestrians(self, spawn_points = None, autopilot=True):
        self.sim_bridge.get_logger().info("Spawning {} pedestrians".format(len(spawn_points)))

        for spawn in spawn_points:
            # Choose a vehicle blueprint at random.

            ped_bp = random.choice(self.sim_bridge.blueprint_library.filter('walker.*.*'))
            self.sim_bridge.get_logger().info("Spawning ped ({}) @ {}".format(ped_bp.id, spawn))
            
            if not spawn:
                #spawn randomly if no point is provided
                spawn = random.choice(self.sim_bridge.world.get_map().get_spawn_points())

            ped = self.sim_bridge.world.try_spawn_actor(ped_bp, spawn)

    def setup_ego(self, ego_x, ego_y, ego_z, ego_yaw, carla_autopilot = False, model='vehicle.audi.etron'):
        
        self.sim_bridge.blueprint_library = self.sim_bridge.world.get_blueprint_library()
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
        vehicle_bp = self.sim_bridge.blueprint_library.find(model)
        self.sim_bridge.ego: carla.Vehicle = self.sim_bridge.world.spawn_actor(vehicle_bp, spawn)
        # TODO: Destroy ego actor when node exits or crashes. Currently keeps actor alive in CARLA,
        # which eventually leads to memory overflow. WSH.
        self.sim_bridge.ego.set_autopilot(enabled=carla_autopilot)
        self.sim_bridge.add_ego_sensors()

    # ALL SCENARIOS

    def normal(self, num_ped=0, num_cars=0):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)
        self.add_vehicles([None] * num_cars)
        self.add_pedestrians([None] * num_ped)

    '''
    MAIN BEHAVIORS
    '''

    # Works (Commit 503)
    # Car should slow down before continuing
    # LK -> Y -> IJ -> LK
    def upcoming_yield_sign(self):
        self.reset_vars()
        
        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    # Works (Commit 503)
    # Car should stop before continuing
    # LK -> SG -> SD -> IJ -> LK
    def upcoming_stop_sign(self):
        self.reset_vars()
        self.ego_spawn = (-152, -79, 20)
        
        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    # Works (Commit 503)
    # Car should slow down then stop completely
    # LK -> Y -> IJ
    def car_in_yield_junction(self):
        self.reset_vars()
        
        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)
        
        # other car position in junction
        car_loc = carla.Transform(carla.Location(x=-72, y=-158, z=20), carla.Rotation(yaw=90))
        self.add_vehicles(autopilot=False, spawn_points=[car_loc])

    # Works (Commit 503)
    # Car should stop for junction but never proceed
    # LK -> SG -> SD
    def car_in_stop_junction(self):
        self.reset_vars()
        self.ego_spawn = (-152, -79, 20)
        
        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)
        
        # other car position in junction
        car_loc = carla.Transform(carla.Location(x=-150, y=-35, z=20), carla.Rotation(yaw=90))
        self.add_vehicles(autopilot=False, spawn_points=[car_loc])

    # Works (Commit 503)
    # Car should behave normally
    # LK -> ...
    def car_in_front(self):
        self.reset_vars()
         
        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)
        
        # other car position in front
        car_loc = carla.Transform(carla.Location(x=-170, y=-158, z=20), carla.Rotation(yaw=0))
        self.add_vehicles(autopilot=True, spawn_points=[car_loc])

    # Works (Commit 503)
    # Car should behave normally
    # LK -> ...
    def car_in_back(self):
        self.reset_vars()
         
        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)
        
        # other car position in back
        car_loc = carla.Transform(carla.Location(x=-185, y=-158, z=20), carla.Rotation(yaw=0))
        self.add_vehicles(autopilot=True, spawn_points=[car_loc])

    # Works (Commit 503)
    # IDK
    # IDK    
    def car_stopped_at_yield_junction(self):
        self.reset_vars()
        self.ego_spawn = (-101, -158, 20)

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)
        
        # other car position at junction
        car_loc = carla.Transform(carla.Location(x=-50, y=-158, z=20), carla.Rotation(yaw=180))
        self.add_vehicles(autopilot=True, spawn_points=[car_loc])

    # Works (Commit 503)
    # Car should stop at junction, wait for ROW cars to clear junction, then go
    # LK -> SG -> SD -> W -> IJ -> LK
    def car_stopped_at_stop_junction(self):
        self.reset_vars()
        # -60.2
        self.ego_spawn = (-152, -60.2, 20)

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)
        
        # other car position at junction FILLIN
        car_loc = carla.Transform(carla.Location(x=-150, y=-2, z=20), carla.Rotation(yaw=180))
        car_loc2 = carla.Transform(carla.Location(x=-120, y=-36, z=20), carla.Rotation(yaw=180))
        self.add_vehicles(autopilot=True, spawn_points=[car_loc, car_loc2])


    '''
    EGO STARTING BEHAVIORS
    '''

    def ego_starts_in_yield_junction(self):
        self.reset_vars()
        self.ego_spawn = (-72, -158, 20)

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)
        
    def ego_starts_in_stop_junction(self):
        self.reset_vars()
        self.ego_spawn = (-72, -158, 20)

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)
    
    def ego_starts_right_before_yield_junction(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def ego_starts_right_before_stop_junction(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def ego_starts_right_behind_car(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def ego_starts_right_in_front_of_car(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def ego_starts_in_parking_lot(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)


    '''
    PEDESTRIAN BEHAVIOR
    '''

    def pedestrian_stationary_on_sidewalk(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def pedestrian_moving_on_sidewalk(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def pedestrian_stationary_in_junction(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def pedestrian_moving_in_junction(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def pedestrian_stationary_near_ego(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def pedestrian_moving_near_ego(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def pedestrian_moving_into_ego_lane(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def pedestrian_moving_out_of_ego_lane(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)

    def pedestrian_moving_in_ego_lane(self):
        self.reset_vars()

        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.setup_ego(self.ego_spawn[0], self.ego_spawn[1], self.ego_spawn[2], self.ego_yaw)


    '''
    CAR BEHAVIOR
    '''
    