from omni.isaac.examples.base_sample import BaseSample
from pxr import Sdf, Gf, UsdPhysics, Usd, UsdGeom
import omni.kit.commands



ENVIRONMENT_BASE_PATH = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/"
ENVIRONMENT_PRIM_PATH = "/World/Environment"
# ROBOT_PATH = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Robots/O3dyn/o3dyn_controller.usd"
ROBOT_PATH = "omniverse://localhost/Library/o3dynsimmodel/o3dyn_ros2.usd"  # From https://git.openlogisticsfoundation.org/silicon-economy/simulation-model/o3dynsimmodel/-/tree/main/documentation
# ROBOT_PATH = ""

class PoseLogger(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.robot = "KMR"
        self.environment = "Grid/default_environment"
        return


    def on_select_environment(self, env):
        self.environment = env
        print(self.environment, "selected")


    def on_select_robot(self, robot):
        self.robot = robot
        print(self.robot, "selected")


    def setup_scene(self):
        """ used to setup anything in the world, adding tasks happen here for instance.
        """
        self._stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()

        environment_path = ENVIRONMENT_BASE_PATH + self.environment + ".usd"
        omni.kit.commands.execute("CreateReference",
            usd_context=omni.usd.get_context(),
            path_to=f"/World/Environment",
            asset_path=environment_path,   
        )

        omni.kit.commands.execute("CreateReference",
            usd_context=omni.usd.get_context(),
            path_to=f"/World/Robot",
            asset_path=ROBOT_PATH,   
        )
        
        # omni.kit.commands.execute("ChangePropertyCommand",
        #     prop_path="/World/Robot.xformOp:translate",
        #     value=(0, 0, 0.15),
        #     prev=(0, 0, 0)
        # )
        return



    async def setup_post_load(self):
        """ called after first reset of the world when pressing load, 
            intializing provate variables happen here.
        """
        return


    async def setup_pre_reset(self):
        """ called in reset button before resetting the world
            to remove a physics callback for instance or a controller reset
        """
        return


    async def setup_post_reset(self):
        """ called in reset button after resetting the world which includes one step with rendering
        """
        return


    def _on_change_config_event(self):
        """ Re-initialize scene with new environment and robot
        """
        print("Initialize")
        return
    
    
    def _on_save_data_event(self, log_path):
        world = self.get_world()
        data_logger = world.get_data_logger()
        data_logger.save(log_path=log_path)
        print("Datalogger saved")
        data_logger.reset()
        print("Datalogger reset")
        return
    

    def _on_logging_event(self, val):
        world = self.get_world()
        print("WORLD", world.current_time)
        data_logger = world.get_data_logger()
        print("DATALOGGER", data_logger)
        if not world.get_data_logger().is_started():
            def frame_logging_func(tasks, scene):
                print("Logging")
                return {
                    "joint_positions": 0
                }

            data_logger.add_data_frame_logging_func(frame_logging_func)
            print("Added frame_logging_func to datalogger")
            data_logger.add_data({"joint_positions": 0}, 1.0, 1.0)
            print("Added data")
        if val:
            data_logger.start()
            print("Datalogger started")
        else:
            data_logger.pause()
            print("Datalogger paused")
        return

    def print_pos_example(self):
        usd_context = omni.usd.get_context()
        # Get list of selected primitives
        selected_prims = usd_context.get_selection().get_selected_prim_paths()
        # Get the current timecode
        # timeline = omni.timeline.get_timeline_interface()
        timecode = self.timeline.get_current_time() * self.timeline.get_time_codes_per_seconds()
        # Loop through all prims and print their transforms
        # for s in selected_prims:
        s = "/World/Robot/o3dyn/base_link"
        curr_prim = self._stage.GetPrimAtPath(s)
        print("Selected", s)
        pose = omni.usd.utils.get_world_transform_matrix(curr_prim, timecode)
        print("Matrix Form:", pose)
        print("Translation: ", pose.ExtractTranslation())
        q = pose.ExtractRotation().GetQuaternion()
        print(
            "Rotation: ", q.GetReal(), ",", q.GetImaginary()[0], ",", q.GetImaginary()[1], ",", q.GetImaginary()[2]
        )
    
    def print_pos(self):
        base_link_path = "/World/Robot/o3dyn/base_link"
        curr_prim = self._stage.GetPrimAtPath(base_link_path)
        timecode = self.timeline.get_current_time() * self.timeline.get_time_codes_per_seconds()
        pose = omni.usd.utils.get_world_transform_matrix(curr_prim, timecode)
        print("Matrix:", pose)
        print("Translation:", pose.ExtractTranslation())

    def world_cleanup(self):
        """Function called when extension shutdowns and starts again, (hot reloading feature)
        """
        return

