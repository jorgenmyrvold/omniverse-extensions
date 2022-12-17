import omni
import asyncio
from omni.isaac.core import World
import numpy as np


O3DYN_BASE_LINK_PRIM_PATH = '/Root/base_link'
O3DYN_WHEEL_PRIM_PATH = '/Root/wheel_drive/'

KMR_BASE_LINK_PRIM_PATH = '/kmr/base_link'
KMR_WHEEL_PRIM_PATH = '/kmr/omniwheel_joints/'


def get_arm_joint_prim_path(joint_id):
    return f'/kmr/kmr_link_{joint_id}/kmr_joint_{joint_id+1}'

class PoseLogger:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()
        self.pose = {}
        self.log_arm = False
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

    def set_log_arm(self, enable):
        self.log_arm = enable

    def set_selected_robot(self, robot):
        self.robot = robot
        if robot == 'KMR':
            self.wheel_prim_path = KMR_WHEEL_PRIM_PATH
            self.base_link_prim_path = KMR_BASE_LINK_PRIM_PATH
            
        elif robot == 'O3dyn':
            self.wheel_prim_path = O3DYN_WHEEL_PRIM_PATH
            self.base_link_prim_path = O3DYN_BASE_LINK_PRIM_PATH
        
        if self.log_arm:
            self.arm_pos_attr = {
                'kmr_joint_1': self.stage.GetPrimAtPath(get_arm_joint_prim_path(0)).GetAttribute('state:angular:physics:position'),
                'kmr_joint_2': self.stage.GetPrimAtPath(get_arm_joint_prim_path(1)).GetAttribute('state:angular:physics:position'),
                'kmr_joint_3': self.stage.GetPrimAtPath(get_arm_joint_prim_path(2)).GetAttribute('state:angular:physics:position'),
                'kmr_joint_4': self.stage.GetPrimAtPath(get_arm_joint_prim_path(3)).GetAttribute('state:angular:physics:position'),
                'kmr_joint_5': self.stage.GetPrimAtPath(get_arm_joint_prim_path(4)).GetAttribute('state:angular:physics:position'),
                'kmr_joint_6': self.stage.GetPrimAtPath(get_arm_joint_prim_path(5)).GetAttribute('state:angular:physics:position'),
                'kmr_joint_7': self.stage.GetPrimAtPath(get_arm_joint_prim_path(6)).GetAttribute('state:angular:physics:position'),
            }

        self.wheel_vel_attr = {
            'wheel_fl_joint': self.stage.GetPrimAtPath(f'{self.wheel_prim_path}wheel_fl_joint').GetAttribute('state:angular:physics:velocity'),
            'wheel_fr_joint': self.stage.GetPrimAtPath(f'{self.wheel_prim_path}wheel_fr_joint').GetAttribute('state:angular:physics:velocity'),
            'wheel_rl_joint': self.stage.GetPrimAtPath(f'{self.wheel_prim_path}wheel_rl_joint').GetAttribute('state:angular:physics:velocity'),
            'wheel_rr_joint': self.stage.GetPrimAtPath(f'{self.wheel_prim_path}wheel_rr_joint').GetAttribute('state:angular:physics:velocity'),
        }


    def on_start_logging_event(self):
        world = World.instance()
        world.clear_physics_callbacks()  # Has to be done as world is not re-initialized and callback id has to be unique
        world.add_physics_callback("sim_step", world.step_async)
        
        data_logger = world.get_data_logger()
        data_logger.pause()
        data_logger.reset()

        def frame_logging_func_pose(tasks, scene):
            curr_prim = self.stage.GetPrimAtPath(self.base_link_prim_path)
            pose = omni.usd.get_world_transform_matrix(curr_prim)
            data = {
                "base_link_transform_matrix": np.array(pose).tolist(),
                "wheel_velocity_fl": self.wheel_vel_attr['wheel_fl_joint'].Get(),
                "wheel_velocity_fr": self.wheel_vel_attr['wheel_fr_joint'].Get(),
                "wheel_velocity_rl": self.wheel_vel_attr['wheel_rl_joint'].Get(),
                "wheel_velocity_rr": self.wheel_vel_attr['wheel_rr_joint'].Get(),
            }
            if self.log_arm:
                extra_data = {
                    'kmr_joint_1_pos': self.arm_pos_attr['kmr_joint_1'].Get(),
                    'kmr_joint_2_pos': self.arm_pos_attr['kmr_joint_2'].Get(),
                    'kmr_joint_3_pos': self.arm_pos_attr['kmr_joint_3'].Get(),
                    'kmr_joint_4_pos': self.arm_pos_attr['kmr_joint_4'].Get(),
                    'kmr_joint_5_pos': self.arm_pos_attr['kmr_joint_5'].Get(),
                    'kmr_joint_6_pos': self.arm_pos_attr['kmr_joint_6'].Get(),
                    'kmr_joint_7_pos': self.arm_pos_attr['kmr_joint_7'].Get(),
                }
                data.update(extra_data)
            return data

        data_logger.add_data_frame_logging_func(frame_logging_func_pose)
        data_logger.start()
        print('[+] DL started')


    def on_save_data_event(self, log_path):
        world = World.instance()
        data_logger = world.get_data_logger()
        data_logger.save(log_path=log_path)
        print("[+] Log saved to", log_path)
        data_logger.reset()
        return


    def print_pose(self):
        curr_prim = self.stage.GetPrimAtPath(self.base_link_prim_path)
        pose = omni.usd.get_world_transform_matrix(curr_prim)
        print("Translation:", pose.ExtractTranslation())