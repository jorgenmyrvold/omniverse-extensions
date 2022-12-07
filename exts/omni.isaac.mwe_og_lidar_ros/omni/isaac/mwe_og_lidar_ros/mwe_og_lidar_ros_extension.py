import os
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.mwe_og_lidar_ros.mwe_og_lidar_ros import MweOgLidarRos


class MweOgLidarRosExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="MWE Omnigraph Lidar ROS problem",
            title="MWE Omnigraph Lidar ROS problem",
            doc_link="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_required_hello_world.html",
            overview="MWE Omnigraph Lidar ROS problem",
            file_path=os.path.abspath(__file__),
            sample=MweOgLidarRos(),
            number_of_extra_frames=1,
        )
        return
