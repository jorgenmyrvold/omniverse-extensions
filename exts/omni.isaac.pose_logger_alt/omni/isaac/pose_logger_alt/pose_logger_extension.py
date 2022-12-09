import os
import omni.ext
import omni.ui as ui
from omni.isaac.pose_logger_alt.pose_logger import PoseLogger
from omni.isaac.ui.ui_utils import setup_ui_headers, str_builder, btn_builder, state_btn_builder
from omni.isaac.core import World
import asyncio


DATAFILE_PATH = "/home/jorgen/omniverse-extensions/data_processing/data/"

def generate_datafile_name(filename):
    path = f"{DATAFILE_PATH}{filename}"
    path_str_len = len(path)
    id = 0
    while os.path.exists(path + ".json"):
        id += 1
        path = path[:path_str_len] + str(id)
    return path + ".json"

class PoseLoggerExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.load_world()
        self.pose_logger = PoseLogger()  # Logic part of extension
        self.ui_elements = {}

        self._window = ui.Window("Pose logger", width=300, height=300)
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                setup_ui_headers(
                    ext_id=ext_id,
                    file_path=os.path.abspath(__file__),
                    title="Pose logger",
                )

                self.ui_elements['Target prim path'] = str_builder(
                    label='Target prim path',
                    default_val="/Root/base_link",
                )

                self.ui_elements['Log output dir'] = str_builder(
                    label="Output Directory",
                    default_val="output_data",
                )

                self.ui_elements['Restart logger button'] = btn_builder(
                    label='Restart logger',
                    text="Restart logger",
                    on_clicked_fn=self.pose_logger.on_restart_data_logger,
                )

                self.ui_elements['Start logging button'] = state_btn_builder(
                    label='Start logging',
                    a_text="Start logging",
                    b_text="Pause logging",
                    on_clicked_fn=self.on_logging_event,
                )

                self.ui_elements['Save log button'] = btn_builder(
                    label='Save log',
                    text="Save Log",
                    on_clicked_fn=self.on_save_log_event,
                )
                
                self.ui_elements['Print button'] = btn_builder(
                    label='Print Pose',
                    text='Print Pose',
                    on_clicked_fn=self.pose_logger.print_pose,
                )

                self.ui_elements['Add callback'] = btn_builder(
                    label='Add callback',
                    text='Add callback',
                    on_clicked_fn=self.pose_logger.add_callback,
                )
        
    def load_world(self):
        async def load_world_async():
            world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
            if World.instance() is None:
                self._world = World(**world_settings)
                await self._world.initialize_simulation_context_async()
            else:
                self._world = World.instance()
            
        asyncio.ensure_future(load_world_async())
        return

    def on_logging_event(self, val):
        self.pose_logger.set_target_prim_path(self.ui_elements['Target prim path'].get_value_as_string())
        self.pose_logger.on_logging_event(val)
    
    def on_save_log_event(self):
        path = generate_datafile_name(self.ui_elements['Log output dir'].get_value_as_string())
        self.pose_logger.on_save_data_event(path)

    def on_shutdown(self):
        return
