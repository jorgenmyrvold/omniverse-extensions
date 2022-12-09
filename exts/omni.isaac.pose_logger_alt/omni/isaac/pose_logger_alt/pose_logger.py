import omni
import asyncio
from omni.isaac.core import World
from pxr import Usd
import numpy as np

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
        self.wheel_fl_velocity_attr = self.stage.GetPrimAtPath('/Root/wheel_drive/wheel_fl_joint').GetAttribute('state:angular:physics:velocity')
        self.wheel_fr_velocity_attr = self.stage.GetPrimAtPath('/Root/wheel_drive/wheel_fr_joint').GetAttribute('state:angular:physics:velocity')
        self.wheel_rl_velocity_attr = self.stage.GetPrimAtPath('/Root/wheel_drive/wheel_rl_joint').GetAttribute('state:angular:physics:velocity')
        self.wheel_rr_velocity_attr = self.stage.GetPrimAtPath('/Root/wheel_drive/wheel_rr_joint').GetAttribute('state:angular:physics:velocity')

        data_logger = world.get_data_logger()
        if not world.get_data_logger().is_started():
            def frame_logging_func_pose(tasks, scene):
                print("DL logged")
                curr_prim = self.stage.GetPrimAtPath(self.target_prim_path)
                timecode = self.timeline.get_current_time() * self.timeline.get_time_codes_per_seconds()
                pose = omni.usd.utils.get_world_transform_matrix(curr_prim, timecode)
                return {
                    "base_link_transform_matrix": np.array(pose).tolist(),
                    "wheel_velocity_fl": self.wheel_fl_velocity_attr.Get(),
                    "wheel_velocity_fr": self.wheel_fr_velocity_attr.Get(),
                    "wheel_velocity_rl": self.wheel_rl_velocity_attr.Get(),
                    "wheel_velocity_rr": self.wheel_rr_velocity_attr.Get(),
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
        world = self.get_world()
        world.clear_physics_callbacks()
        world.add_physics_callback("sim_step", world.step_async)
        print("ADDED callback")

    def print_pose(self):
        curr_prim = self.stage.GetPrimAtPath(self.target_prim_path)
        timecode = self.timeline.get_current_time() * self.timeline.get_time_codes_per_seconds()
        pose = omni.usd.utils.get_world_transform_matrix(curr_prim, timecode)
        print("Matrix:", pose)
        print("Translation:", pose.ExtractTranslation())