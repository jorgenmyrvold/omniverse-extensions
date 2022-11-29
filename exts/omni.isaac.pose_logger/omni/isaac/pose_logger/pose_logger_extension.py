import os
import omni.ui as ui
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.pose_logger import PoseLogger
from omni.isaac.ui.ui_utils import get_style, dropdown_builder, btn_builder, str_builder, state_btn_builder


class PoseLoggerExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="Pose logger",
            title="Pose logger example standalone",
            doc_link="",
            overview="Pose logger extension",
            file_path=os.path.abspath(__file__),
            sample=PoseLogger(),
            number_of_extra_frames=2,
        )
        self.environment = "Grid/default_environment"
        self.robot = "KMR"
        
        self.ui_elements = {}
        frame = self.get_frame(index=0)
        self._build_config_ui(frame)
        frame = self.get_frame(index=1)
        # self.build_data_logging_ui(frame)
        self.build_data_logging_ui_new(frame)
        return


    def _build_config_ui(self, frame):
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                frame.title = "Configuration"
                frame.visible = True

                self.ui_elements["Environment"] = dropdown_builder(
                    label="Environment",
                    items=["Grid/default_environment", "Simple_Warehouse/warehouse_with_forklifts", "Simple_Warehouse/warehouse_multiple_shelves"],
                    default_val=0,
                    on_clicked_fn=lambda env: self.sample.on_select_environment(env),
                    tooltip="Select environment",
                )

                self.ui_elements["Robot"] = dropdown_builder(
                    label="Robot",
                    items=["KMR", "O3dyn", "Holonomic Dummy"],
                    default_val=0,
                    on_clicked_fn=lambda robot: self.sample.on_select_robot(robot),
                    tooltip="Select Robot",
                )

                dict = {
                    "label": "Change config",
                    "type": "button",
                    "text": "Change config",
                    "tooltip": "Change configuration",
                    "on_clicked_fn": self.sample._on_change_config_event,
                }
                self.ui_elements["Change config"] = btn_builder(**dict)
                self.ui_elements["Change config"].enabled = True


    def _on_logging_button_event(self, val):
        self.sample._on_logging_event(val)
        self.ui_elements["Save Data"].enabled = True
        return
    
    def _on_save_data_button_event(self):
        self.sample._on_save_data_event(self.ui_elements["Output Directory"].get_value_as_string())
        return

    def build_data_logging_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                frame.title = "Data Logging"
                frame.visible = True
                dict = {
                    "label": "Output Directory",
                    "type": "stringfield",
                    "default_val": os.path.join(os.getcwd(), "output_data.json"),
                    "tooltip": "Output Directory",
                    "on_clicked_fn": None,
                    "use_folder_picker": False,
                    "read_only": False,
                }
                self.ui_elements["Output Directory"] = str_builder(**dict)

                dict = {
                    "label": "Start Logging",
                    "type": "button",
                    "a_text": "START",
                    "b_text": "PAUSE",
                    "tooltip": "Start Logging",
                    "on_clicked_fn": self._on_logging_button_event,
                }
                self.ui_elements["Start Logging"] = state_btn_builder(**dict)
                self.ui_elements["Start Logging"].enabled = False

                dict = {
                    "label": "Save Data",
                    "type": "button",
                    "text": "Save Data",
                    "tooltip": "Save Data",
                    "on_clicked_fn": self._on_save_data_button_event,
                }
                self.ui_elements["Save Data"] = btn_builder(**dict)
                self.ui_elements["Save Data"].enabled = False

                dict = {
                    "label": "Print pos",
                    "type": "button",
                    "text": "Print pos",
                    "on_clicked_fn": self.sample.print_pos_scene,
                }
                self.ui_elements["Print pos"] = btn_builder(**dict)
                # self.ui_elements["Print pos"].enabled = True

    def build_data_logging_ui_new(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                frame.title = "Data Logging"
                frame.visible = True
                dict = {
                    "label": "Output Directory",
                    "type": "stringfield",
                    "default_val": os.path.join(os.getcwd(), "output_data.json"),
                    "tooltip": "Output Directory",
                    "on_clicked_fn": None,
                    "use_folder_picker": False,
                    "read_only": False,
                }
                self.ui_elements["Output Directory"] = str_builder(**dict)

                dict = {
                    "label": "Start Logging",
                    "type": "button",
                    "a_text": "START",
                    "b_text": "PAUSE",
                    "tooltip": "Start Logging",
                    "on_clicked_fn": self._on_logging_button_event,
                }
                self.ui_elements["Start Logging"] = state_btn_builder(**dict)
                self.ui_elements["Start Logging"].enabled = False

                dict = {
                    "label": "Save Data",
                    "type": "button",
                    "text": "Save Data",
                    "tooltip": "Save Data",
                    "on_clicked_fn": self._on_save_data_button_event,
                }

                self.ui_elements["Save Data"] = btn_builder(**dict)
                self.ui_elements["Save Data"].enabled = False
        return

    def post_load_button_event(self):
        self.ui_elements["Start Logging"].enabled = True
        self.ui_elements["Save Data"].enabled = False
    
    def post_reset_button_event(self):
        return