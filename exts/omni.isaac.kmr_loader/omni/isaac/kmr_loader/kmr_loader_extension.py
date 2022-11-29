import os
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.kmr_loader.kmr_loader import KMRLoader
from omni.isaac.ui.ui_utils import get_style, dropdown_builder, btn_builder, str_builder, state_btn_builder
import omni.ui as ui


class KMRLoaderExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="KMR Loader",
            title="KMR Loader",
            doc_link="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_required_hello_world.html",
            overview="This extension imports KUKA KMR iiwa",
            file_path=os.path.abspath(__file__),
            sample=KMRLoader(),
            number_of_extra_frames=1,
        )

        self.environment = "Grid/default_environment"
        self.robot = "KMR"
        
        self.ui_elements = {}
        frame = self.get_frame(index=0)
        self._build_config_ui(frame)
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
