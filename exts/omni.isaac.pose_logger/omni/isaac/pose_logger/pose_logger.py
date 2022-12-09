import omni
import asyncio
from omni.isaac.core import World
import numpy as np

class PoseLogger:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()
        self.pose = {}
        return
    

    def load_world(self):
        async def load_world_async():
            world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
            if World.instance() is None:
                self._world = World(**world_settings)
                await self._world.initialize_simulation_context_async()
            else:
                self._world = World.instance()
        asyncio.ensure_future(load_world_async())


    def get_world(self):
        return World.instance()


    def set_target_prim_path(self, target_prim_path):
        self.target_prim_path = target_prim_path


    def on_start_logging_event(self):
        world = self.get_world()
        world.clear_physics_callbacks()  # Has to be done as world is not re-initialized and callback id has to be unique
        world.add_physics_callback("sim_step", world.step_async)

        self.wheel_fl_velocity_attr = self.stage.GetPrimAtPath('/Root/wheel_drive/wheel_fl_joint').GetAttribute('state:angular:physics:velocity')
        self.wheel_fr_velocity_attr = self.stage.GetPrimAtPath('/Root/wheel_drive/wheel_fr_joint').GetAttribute('state:angular:physics:velocity')
        self.wheel_rl_velocity_attr = self.stage.GetPrimAtPath('/Root/wheel_drive/wheel_rl_joint').GetAttribute('state:angular:physics:velocity')
        self.wheel_rr_velocity_attr = self.stage.GetPrimAtPath('/Root/wheel_drive/wheel_rr_joint').GetAttribute('state:angular:physics:velocity')

        data_logger = world.get_data_logger()
        data_logger.pause()
        data_logger.reset()

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
        data_logger.start()


    def on_save_data_event(self, log_path):
        world = self.get_world()
        data_logger = world.get_data_logger()
        data_logger.save(log_path=log_path)
        print("Datalogger saved to", log_path)
        data_logger.reset()
        print("Datalogger reset")
        return


    def print_pose(self):
        curr_prim = self.stage.GetPrimAtPath(self.target_prim_path)
        timecode = self.timeline.get_current_time() * self.timeline.get_time_codes_per_seconds()
        pose = omni.usd.utils.get_world_transform_matrix(curr_prim, timecode)
        print("Matrix:", pose)
        print("Translation:", pose.ExtractTranslation())