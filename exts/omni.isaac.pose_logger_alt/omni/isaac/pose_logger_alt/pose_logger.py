import omni
import asyncio
from omni.isaac.core import World

class PoseLogger:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()
        self.pose = {}

        
        return
    
    def get_world(self):
        return World.instance()
    
    def set_target_prim_path(self, target_prim_path):
        self.target_prim_path = target_prim_path
        return

    def on_restart_data_logger(self):
        world = self.get_world()
        data_logger = world.get_data_logger()
        data_logger.reset()
        print('Datalogger restarted')
        print('Datalogger is started', data_logger.is_started())

    def on_logging_event(self, val):
        # TODO: Reason why datalogger does not work is that world does not step...
        # Clicking the "step" button steps the world manually and this logs
        world = self.get_world()
        data_logger = world.get_data_logger()
        if not world.get_data_logger().is_started():
            def frame_logging_func_pose(tasks, scene):
                print("DL logged")
                return {
                    "joint_positions": 0
                }
            data_logger.add_data_frame_logging_func(frame_logging_func_pose)
        if val:
            data_logger.start()
            print('DL Started')
        else:
            data_logger.pause()
            print('DL Paused')
        return
    
    def step_manually(self):
        self.get_world().step_async()
    
    def play_sim(self):
        asyncio.ensure_future(self.get_world().play_async())

    def on_save_data_event(self, log_path):
        world = self.get_world()
        data_logger = world.get_data_logger()
        data_logger.save(log_path=log_path)
        print("Datalogger saved to", log_path)
        data_logger.reset()
        print("Datalogger reset")
        return

    def add_callback(self):
        def dummy_callback():
            a=10
            return
        world = self.get_world()
        # world.add_physics_callback("dummy_callback", dummy_callback)
        world.add_physics_callback("dummy_callback", world.step_async)

    def print_pose(self):
        curr_prim = self.stage.GetPrimAtPath(self.target_prim_path)
        timecode = self.timeline.get_current_time() * self.timeline.get_time_codes_per_seconds()
        pose = omni.usd.utils.get_world_transform_matrix(curr_prim, timecode)
        print("Matrix:", pose)
        print("Translation:", pose.ExtractTranslation())