import carla
import random

class ScenarioManager:

    def __init__(self, sim_bridge):
        self.sim_bridge = sim_bridge
        self.world = 'Town01'
        # spawn_points = self.sim_bridge.world.get_map().get_spawn_points()
        # spawn_xyz = spawn_points[0].location
        # self.ego_spawn = (spawn_xyz.x, spawn_xyz.y, spawn_xyz.z)
        self.ego_yaw = 90

    def reset_vars(self):
        self.ego_spawn = (0.0, 0.0, 0.0)
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

    def add_pedestrians(self, ped_qty = 30, autopilot=True):
        self.sim_bridge.get_logger().info("Spawning {} pedestrians".format(ped_qty))

        world = self.sim_bridge.world

        blueprintsWalkers = world.get_blueprint_library().filter("walker.pedestrian.*")

        spawn_points = []
        for i in range(ped_qty):
            spawn_point = carla.Transform()
            spawn_point.location = world.get_random_location_from_navigation()
            if (spawn_point.location != None):
                spawn_points.append(spawn_point)

        # 2. build the batch of commands to spawn the pedestrians
        batch = []
        walkers_list = []
        all_id = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

        # apply the batch
        results = self.sim_bridge.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                # logging.error(results[i].error)
                self.sim_bridge.get_logger().info(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})

        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))

        # apply the batch
        
        results = self.sim_bridge.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                # logging.error(results[i].error)
                self.sim_bridge.get_logger().info(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id

        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        world.wait_for_tick()

        # 5. initialize each controller and set target to walk to (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_actors), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # random max speed
            all_actors[i].set_max_speed(1 + random.random())    # max speed between 1 and 2 (default is 1.4 m/s)

    def setup_ego(self, pose: carla.Transform, carla_autopilot = False, model='vehicle.audi.etron'):
        
        self.sim_bridge.blueprint_library = self.sim_bridge.world.get_blueprint_library()

        vehicle_bp = self.sim_bridge.blueprint_library.find(model)
        self.sim_bridge.ego: carla.Vehicle = self.sim_bridge.world.spawn_actor(vehicle_bp, pose)
        # TODO: Destroy ego actor when node exits or crashes. Currently keeps actor alive in CARLA,
        # which eventually leads to memory overflow. WSH.
        self.sim_bridge.ego.set_autopilot(enabled=carla_autopilot)
        self.sim_bridge.add_ego_sensors()

    # ALL SCENARIOS

    def normal(self, num_ped=0, num_cars=0, carla_autopilot = False):
        self.reset_vars()
        self.sim_bridge.world = self.sim_bridge.client.load_world(self.world)
        self.sim_bridge.get_logger().info("LOADING WORLD")
        self.setup_ego(self.sim_bridge.get_random_spawn(), carla_autopilot = carla_autopilot)
        self.add_vehicles([None] * num_cars)
        self.add_pedestrians(ped_qty=num_ped)

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
    