import omni.kit.commands
import omni.graph.core as og
from omni.isaac.core.utils.extensions import disable_extension, enable_extension
from omni.isaac.examples.base_sample import BaseSample


ENVIRONMENT_PATH = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

class MweOgLidarRos(BaseSample):
    def __init__(self) -> None:
        super().__init__()

    def setup_scene(self):
        disable_extension("omni.isaac.ros_bridge")
        print("[+] ROS Bridge disabled")
        enable_extension("omni.isaac.ros2_bridge")
        print("[+] ROS 2 Bridge enabled")

        self._stage = omni.usd.get_context().get_stage()
        omni.kit.commands.execute("CreateReference",
            usd_context=omni.usd.get_context(),
            path_to=f"/World/Environment",
            asset_path=ENVIRONMENT_PATH,   
        )
        self._create_lidar_sensor()
        self._setup_minimum_lidar_graph()
        return
    
    def _create_lidar_sensor(self):      
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/Lidar",
            parent="/World",
            min_range=0.4,
            max_range=100.0,
            draw_points=False,
            draw_lines=False,
            horizontal_fov=360,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=20.0,
            high_lod=False,
            yaw_offset=0,
            enable_semantics=False,
        )
        return result, prim

    def _setup_minimum_lidar_graph(self):
        keys = og.Controller.Keys
        lidar_prim_path = "/World/Lidar"
        graph_path = "/lidar_graph"
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("isaac_read_lidar_beam_node", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("constant_string_frame_id", "omni.graph.nodes.ConstantString"),
                    ("isaac_read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ros2_pub_laser_scan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                ],
                keys.SET_VALUES: [
                    ("ros2_pub_laser_scan.inputs:topicName", "/laser_scan"),
                    ("constant_string_frame_id.inputs:value", "world"),
                    ("ros2_context.outputs:context", 0),
                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "isaac_read_lidar_beam_node.inputs:execIn"),
                    ("isaac_read_lidar_beam_node.outputs:execOut", "ros2_pub_laser_scan.inputs:execIn"),
                    ("isaac_read_lidar_beam_node.outputs:azimuthRange", "ros2_pub_laser_scan.inputs:azimuthRange"),
                    ("isaac_read_lidar_beam_node.outputs:depthRange", "ros2_pub_laser_scan.inputs:depthRange"),
                    ("isaac_read_lidar_beam_node.outputs:horizontalFov", "ros2_pub_laser_scan.inputs:horizontalFov"),
                    ("isaac_read_lidar_beam_node.outputs:horizontalResolution", "ros2_pub_laser_scan.inputs:horizontalResolution"),
                    ("isaac_read_lidar_beam_node.outputs:intensitiesData", "ros2_pub_laser_scan.inputs:intensitiesData"),
                    ("isaac_read_lidar_beam_node.outputs:linearDepthData", "ros2_pub_laser_scan.inputs:linearDepthData"),
                    ("isaac_read_lidar_beam_node.outputs:numCols", "ros2_pub_laser_scan.inputs:numCols"),
                    ("isaac_read_lidar_beam_node.outputs:numRows", "ros2_pub_laser_scan.inputs:numRows"),
                    ("isaac_read_lidar_beam_node.outputs:rotationRate", "ros2_pub_laser_scan.inputs:rotationRate"),
                    ("ros2_context.outputs:context", "ros2_pub_laser_scan.inputs:context"),
                    ("constant_string_frame_id.inputs:value", "ros2_pub_laser_scan.inputs:frameId"),
                    ("isaac_read_sim_time.outputs:simulationTime", "ros2_pub_laser_scan.inputs:timeStamp"),
                ],
            }
        )

        read_lidar_og_path = f"{graph_path}/isaac_read_lidar_beam_node"
        usd_prim = self._stage.GetPrimAtPath(read_lidar_og_path)
        usd_prim.GetRelationship("inputs:lidarPrim").AddTarget(lidar_prim_path)

 
