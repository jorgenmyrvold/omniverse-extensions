import os
import omni.ext
import omni.ui as ui
from omni.isaac.pose_logger.pose_logger import PoseLogger
from omni.isaac.ui.ui_utils import setup_ui_headers, str_builder, btn_builder, dropdown_builder, cb_builder
from omni.isaac.core import World
import asyncio


DATAFILE_PATH = "/home/jorgen/omniverse-extensions/data_processing/data/"
SELECTABLE_ROBOTS = ['KMR', 'O3dyn']

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
                
                self.ui_elements['Robot selector'] = dropdown_builder(
                    label="Select robot",
                    items=SELECTABLE_ROBOTS,
                    default_val=0,
                )
                self.ui_elements['Log arm cb'] = cb_builder(
                    label='Log arm positions',
                    default_val=False,
                    on_clicked_fn=lambda enable: self.pose_logger.set_log_arm(enable),
                )
                self.ui_elements['Log output dir'] = str_builder(
                    label="Output filename",
                    default_val="output_data",
                )
                self.ui_elements['Initialize button'] = btn_builder(
                    label='Initialize datalogger',
                    text='Initialize',
                    on_clicked_fn=self.on_initialize_event,
                )
                self.ui_elements['Logging button'] = btn_builder(
                    label='Start logging',
                    text="Start",
                    on_clicked_fn=self.on_start_logging_event,
                )
                self.ui_elements['Logging button'].enabled = False

                self.ui_elements['Save log button'] = btn_builder(
                    label='Save log',
                    text="Save Log",
                    on_clicked_fn=self.on_save_log_event,
                )
                self.ui_elements['Save log button'].enabled = False

        
    def on_initialize_event(self):
        self.pose_logger.load_world()
        self.pose_logger.set_selected_robot(SELECTABLE_ROBOTS[self.ui_elements['Robot selector'].get_item_value_model().get_value_as_int()])
        self.ui_elements['Logging button'].enabled = True
        print('[+] Initialized')
        

    def on_start_logging_event(self):
        self.pose_logger.on_start_logging_event()
        self.ui_elements['Save log button'].enabled = True
    

    def on_save_log_event(self):
        path = generate_datafile_name(self.ui_elements['Log output dir'].get_value_as_string())
        self.pose_logger.on_save_data_event(path)

    def on_shutdown(self):
        return
