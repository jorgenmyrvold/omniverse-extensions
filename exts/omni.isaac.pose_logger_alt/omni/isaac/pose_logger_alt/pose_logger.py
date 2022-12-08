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

    def on_save_data_event(self, log_path):
        world = self.get_world()
        data_logger = world.get_data_logger()
        data_logger.save(log_path=log_path)
        print("Datalogger saved to", log_path)
        data_logger.reset()
        print("Datalogger reset")
        return
