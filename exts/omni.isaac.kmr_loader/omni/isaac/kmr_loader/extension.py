import asyncio
import omni
import omni.ui as ui
import omni.kit.commands
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core.utils.extensions import disable_extension, enable_extension
from omni.isaac.dynamic_control import _dynamic_control
import omni.graph.core as og
from omni.isaac.range_sensor._range_sensor import acquire_lidar_sensor_interface
from omni.isaac.urdf import _urdf
from pxr import Sdf, Gf, UsdPhysics, Usd, UsdGeom


EXTENSION_NAME = "KMR iiwa importer"
# ENVIRONMENT_PATH = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
# ENVIRONMENT_PATH = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
ENVIRONMENT_PATH = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/Grid/gridroom_curved.usd"
# ENVIRONMENT_PATH = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/Grid/default_environment.usd"
# ENVIRONMENT_PATH = "/home/jorgen/.cache/ov/client/omniverse/localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/Grid/default_environment.usd"
ENVIRONMENT_PRIM_PATH = "/World/environment"
# KMR_PATH = "/home/jorgen/ros-ws/src/kmriiwa_description/urdf/robot/kmriiwa_no_wheels_new.urdf"
# KMR_PATH = "/home/jorgen/misc_repos/kmriiwa_ws_devel/src/kmr_bringup/urdf/kmriiwa_withcameraframes.urdf"
KMR_PATH = "/home/jorgen/ros2-ws/src/kmr_description/urdf/robot/kmr.urdf"
OMNIWHEELS_PATH = "/home/jorgen/isaac_ws/omniwheels/"
ROS2_FRAME_ID = "world"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        print("[omni.kmr] MyExtension startup")
        disable_extension("omni.isaac.ros_bridge")
        print("[+] ROS Bridge disabled")
        enable_extension("omni.isaac.ros2_bridge")
        print("[+] ROS 2 Bridge enabled")
        
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._ext_id = ext_id
        self._extension_path = ext_manager.get_extension_path(ext_id)
        
        self._window = ui.Window("KMR iiwa importer", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Label("KUKA KMR iiwa importer and ROS2 initializer")
                ui.Button("Start", clicked_fn=self._on_load_scene)

    def on_shutdown(self):
        self._window = None

    def _on_load_scene(self):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        load_stage = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._create_world(load_stage))

    async def _create_world(self, task):
        done, pending = await asyncio.wait({task})
        if task in done:
            # Create default world prim
            self._stage = omni.usd.get_context().get_stage()
            world_prim = self._stage.DefinePrim("/World")
            self._stage.SetDefaultPrim(world_prim)
            
            # Create a pyhysics scene
            scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/physicsScene"))
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(9.81)
            
            # Create warehouse reference
            ref_warehouse = self._stage.DefinePrim(ENVIRONMENT_PRIM_PATH)
            omni.kit.commands.execute("AddReference",
                stage=self._stage,
                prim_path=Sdf.Path(ENVIRONMENT_PRIM_PATH),  # an existing prim to add the reference to.
                reference=Sdf.Reference(ENVIRONMENT_PATH)
            )
            print("[+] Created environment")
            
            # Load robot urdf
            res, self._kmr_prim = self._load_kmr()
            self._rig_robot()
            
            # Set up different omnigraphs
            keys = og.Controller.Keys
            # self._setup_graph_kmp()
            self._setup_graph_iiwa(keys)
            self._setup_lidar_graph(keys, is_front_lidar=True)
            self._setup_lidar_graph(keys, is_front_lidar=False)
            # self._setup_tf_graph(keys)
            # self._setup_odom_graph(keys)
            # self._setup_camera_graph(keys)
                        

    def _load_kmr(self, urdf_filepath=KMR_PATH):
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = False
        import_config.make_default_prim = False
        import_config.self_collision = False
        import_config.create_physics_scene = False
        import_config.import_inertia_tensor = False
        import_config.default_drive_strength = 1047.19751
        import_config.default_position_drive_damping = 52.35988
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION  # JOINT_DRIVE_VELOCITY of JOINT_DRIVE_POSITION
        import_config.distance_scale = 1
        import_config.density = 0.0
        
        result, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile", 
            urdf_path=urdf_filepath,
            import_config=import_config
        )
        return result, prim_path
    

    def _load_omniwheels(self, omniwheels_path=OMNIWHEELS_PATH):
        """
        omnisheels_path should point to the directory where the four omniwheels with names
        omniwheel_fl.usd, omniwheel_fr.usd, omniwheel_rl.usd, omniwheel_rr.usd
        """
        omniwheels = {"omniwheel_fl": "front_left", 
                      "omniwheel_fr": "front_right", 
                      "omniwheel_rl": "back_left",
                      "omniwheel_rr": "back_right"}

        for prim_name, wheel in omniwheels.items():
            omniwheel_path = f"{self._kmr_prim}/{prim_name}"
            omniwheel_prim = self._stage.DefinePrim(omniwheel_path)
            # omniwheel_prim = self._stage.GetPrimAtPath(f"{self._kmr_prim}/kmriiwa_{wheel}_wheel_link")
            omni.kit.commands.execute("AddReference",
                stage=self._stage,
                prim_path=Sdf.Path(omniwheel_path),
                reference=Sdf.Reference(f"{omniwheels_path}/{prim_name}.usd")
            )
            # omni.kit.commands.execute("AddPayload",
            #     stage=self._stage,
            #     prim_path=Sdf.Path(omniwheel_path),
            #     payload=Sdf.Payload(f"{omniwheels_path}/{prim_name}.usd")
            # )
            if prim_name[-2:] == "fl":
                UsdGeom.Xformable(omniwheel_prim).AddTranslateOp().Set((0.28, 0.4825, 0.17))  # Should be 0.28, 0.1825, 0.125
                UsdGeom.Xformable(omniwheel_prim).AddRotateXYZOp().Set((0, 0, -90))
            elif prim_name[-2:] == "fr":
                UsdGeom.Xformable(omniwheel_prim).AddTranslateOp().Set((0.28, -0.4825, 0.17))
                UsdGeom.Xformable(omniwheel_prim).AddRotateXYZOp().Set((0, 0, 90))
            elif prim_name[-2:] == "rl":
                UsdGeom.Xformable(omniwheel_prim).AddTranslateOp().Set((-0.28, 0.4825, 0.17))
                UsdGeom.Xformable(omniwheel_prim).AddRotateXYZOp().Set((0, 0, -90))
            elif prim_name[-2:] == "rr":
                UsdGeom.Xformable(omniwheel_prim).AddTranslateOp().Set((-0.28, -0.4825, 0.17))
                UsdGeom.Xformable(omniwheel_prim).AddRotateXYZOp().Set((0, 0, 90))

            
            # TODO: Create 4 joints and add bodies to their relationships
            
            # d6_props = _dynamic_control.DynamicControl.D6JointProperties()

            # d6_props.GetRelationship("physic:body0").AddTarget(self._scene.GetPrimAtPath(f"{self._kmr_prim}/base_link"))

            # joint = self._dc.create_d6_joint(d6_props)
            # joint = self._stage.GetPrimAtPath(f"{self._kmr_prim}/kmriiwa_base_link/kmriiwa_{wheel}_wheel_joint")
            # joint.GetRelationship("physics:body0").ClearTargets(removeSpec=False)
            # joint.GetRelationship("physics:body1").ClearTargets(removeSpec=False)
            # joint.GetRelationship("physics:body1").AddTarget(f"{self._kmr_prim}/kmriiwa_base_link")
            # joint.GetRelationship("physics:body0").AddTarget(f"{omniwheel_path}/{wheel}/omniwheel")
            

        print("[+] Created omniwheels")
    

    def _rig_robot(self):
        result1, self._lidar1_prim = self._create_lidar_sensor(is_front_lidar=True)
        result2, self._lidar2_prim = self._create_lidar_sensor(is_front_lidar=False)
        # TODO: Add cameras and other relevant sensors
        # self._create_camera("/World", "/Camera_1")
        self._load_omniwheels()


    def _create_lidar_sensor(self, is_front_lidar: bool):
        if is_front_lidar:
            parent_prim = f"{self._kmr_prim}/kmriiwa_laser_B1_link"
            yaw_offset = 0.0  # 45.0
            horizontal_fov = 246.0  # Experimental values
        else:
            parent_prim = f"{self._kmr_prim}/kmriiwa_laser_B4_link"
            yaw_offset = 0.0  # 225.0
            horizontal_fov = 249.0  # Experimental values
            
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/Lidar",
            parent=parent_prim,
            min_range=0.4,
            max_range=100.0,
            draw_points=False,
            draw_lines=False,
            horizontal_fov=horizontal_fov,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=20.0,
            high_lod=False,
            yaw_offset=yaw_offset,
            enable_semantics=False,
        )
        
        if result:
            print(f"[+] Created {parent_prim}/Lidar")
        else:
            print(f"[!] Failed creating {parent_prim}/Lidar")
        return result, prim


    def _create_camera(self, parent_prim: str, camera_prim_name: str):
        # TODO: Connect the camera to an existing prim on the robot
        camera_prim = self._stage.DefinePrim(f"{parent_prim}{camera_prim_name}", "Camera")
        UsdGeom.Xformable(camera_prim).AddTranslateOp().Set((0., 0., 1.))
        UsdGeom.Xformable(camera_prim).AddRotateXYZOp().Set((90., 0., -90.))
        print(f"[+] Created {parent_prim}{camera_prim_name}")
    
    
    def _setup_lidar_graph(self, keys, is_front_lidar: bool):
        if is_front_lidar:
            lidar_num = 1
            lidar_frame_id = "kmriiwa_laser_B1_link"
        else:
            lidar_num = 2
            lidar_frame_id = "kmriiwa_laser_B4_link"

        lidar_prim_path = f"{self._kmr_prim}/{lidar_frame_id}/Lidar"
        graph_path = f"/lidar{lidar_num}_graph"
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
                    ("ros2_pub_laser_scan.inputs:topicName", f"/laser_scan{lidar_num}"),
                    ("constant_string_frame_id.inputs:value", lidar_frame_id),
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
        print(f"[+] Created {graph_path}")
    
    
    def _setup_graph_kmp(self, keys):
        graph_prim_path = "/kmp_controller_graph"
        og.Controller.edit(
            {"graph_path": graph_prim_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("constant_token_FR", "omni.graph.nodes.ConstantToken"), # Front Right etc...
                    ("constant_token_FL", "omni.graph.nodes.ConstantToken"),
                    ("constant_token_BR", "omni.graph.nodes.ConstantToken"),
                    ("constant_token_BL", "omni.graph.nodes.ConstantToken"),
                    ("constant_double_FR", "omni.graph.nodes.ConstantDouble"),
                    ("constant_double_FL", "omni.graph.nodes.ConstantDouble"),
                    ("constant_double_BR", "omni.graph.nodes.ConstantDouble"),
                    ("constant_double_BL", "omni.graph.nodes.ConstantDouble"),
                    ("make_array_joint_names", "omni.graph.nodes.MakeArray"),
                    ("make_array_joint_vel", "omni.graph.nodes.MakeArray"),
                    ("articulation_controller", "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    ("constant_token_FR.inputs:value", "kmriiwa_back_left_wheel_joint"),
                    ("constant_token_FL.inputs:value", "kmriiwa_back_right_wheel_joint"),
                    ("constant_token_BR.inputs:value", "kmriiwa_front_left_wheel_joint"),
                    ("constant_token_BL.inputs:value", "kmriiwa_front_right_wheel_joint"),  # forward | sideways
                    ("constant_double_FR.inputs:value", -5.0),                               # 5       | -5
                    ("constant_double_FL.inputs:value", 5.0),                               # 5       | 5
                    ("constant_double_BR.inputs:value", 5.0),                               # 5       | 5
                    ("constant_double_BL.inputs:value", -5.0),                               # 5       | -5
                    ("make_array_joint_names.inputs:arraySize", 4),
                    ("make_array_joint_vel.inputs:arraySize", 4),
                    ("articulation_controller.inputs:robotPath", self._kmr_prim),
                    ("articulation_controller.inputs:usePath", True),
                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "articulation_controller.inputs:execIn"),
                    ("constant_token_FR.inputs:value", "make_array_joint_names.inputs:a"),
                    ("constant_token_FL.inputs:value", "make_array_joint_names.inputs:b"),
                    ("constant_token_BR.inputs:value", "make_array_joint_names.inputs:c"),
                    ("constant_token_BL.inputs:value", "make_array_joint_names.inputs:d"),
                    ("constant_double_FR.inputs:value", "make_array_joint_vel.inputs:a"),
                    ("constant_double_FL.inputs:value", "make_array_joint_vel.inputs:b"),
                    ("constant_double_BR.inputs:value", "make_array_joint_vel.inputs:c"),
                    ("constant_double_BL.inputs:value", "make_array_joint_vel.inputs:d"),
                    ("make_array_joint_names.outputs:array", "articulation_controller.inputs:jointNames"),
                    ("make_array_joint_vel.outputs:array", "articulation_controller.inputs:velocityCommand"),
                ]
            }
        )
        print(f"[+] Created {graph_prim_path}")
    
    
    def _setup_graph_iiwa(self, keys):
        # TODO: example from https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_manipulation.html#add-joint-states-in-extension
        # Change to iiwa from Franka
        graph_prim_path = "/iiwa_controller_graph"
        og.Controller.edit(
            {"graph_path": graph_prim_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                    ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                    ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                    ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Providing path to /panda robot to Articulation Controller node
                    # Providing the robot path is equivalent to setting the targetPrim in Articulation Controller node
                    ("ArticulationController.inputs:usePath", True),
                    ("ArticulationController.inputs:robotPath", self._kmr_prim),
                ],
            },
        )
        # Setting the /panda target prim to Publish JointState node
        set_target_prims(primPath=f"{graph_prim_path}/PublishJointState", targetPrimPaths=[self._kmr_prim])
        print(f"[+] Created {graph_prim_path}")
    
    
    def _setup_tf_graph(self, keys):
        graph_path = "/tf_pub_graph"
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("isaac_read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ros2_pub_tf", "omni.isaac.ros2_bridge.ROS2PublishTransformTree")
                ],
                keys.SET_VALUES: [
                    ("ros2_context.outputs:context", 0),
                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "ros2_pub_tf.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_pub_tf.inputs:context"),
                    ("isaac_read_sim_time.outputs:simulationTime", "ros2_pub_tf.inputs:timeStamp"),
                ]
            }
        )
        
        tf_publisher_og_path = f"{graph_path}/ros2_pub_tf"
        usd_prim = self._stage.GetPrimAtPath(tf_publisher_og_path)
        # usd_prim.GetRelationship("inputs:parentPrim").AddTarget("/World")
        # usd_prim.GetRelationship("inputs:parentPrim").AddTarget(f"{self._kmr_prim}/kmriiwa_base_link")
        # 
        # TODO: Problem with multiple targets
        # usd_prim.GetRelationship("inputs:targetPrims").AddTarget(f"{self._kmr_prim}/kmriiwa_laser_B1_link/Lidar")
        # usd_prim.GetRelationship("inputs:targetPrims").AddTarget(f"{self._kmr_prim}/kmriiwa_laser_B4_link/Lidar")
        usd_prim.GetRelationship("inputs:targetPrims").AddTarget(self._kmr_prim)
        # TODO: Add cameras and other relevant sensors
        
        print(f"[+] Created {graph_path}")
    
    
    def _setup_odom_graph(self, keys):
        # TODO!: Not connected to wheels!!!
        graph_path = "/odom_pub_graph"
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("isaac_read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("isaac_compute_odom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                    ("ros2_pub_odom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                    ("ros2_pub_raw_tf", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("ros2_context.outputs:context", 0),
                    ("ros2_pub_raw_tf.inputs:topicName", "tf_raw"),
                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "isaac_compute_odom.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "ros2_pub_odom.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "ros2_pub_raw_tf.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_pub_odom.inputs:context"),
                    ("ros2_context.outputs:context", "ros2_pub_raw_tf.inputs:context"),
                    ("isaac_read_sim_time.outputs:simulationTime", "ros2_pub_odom.inputs:timeStamp"),
                    ("isaac_read_sim_time.outputs:simulationTime", "ros2_pub_raw_tf.inputs:timeStamp"),
                    ("isaac_compute_odom.outputs:angularVelocity", "ros2_pub_odom.inputs:angularVelocity"),
                    ("isaac_compute_odom.outputs:linearVelocity", "ros2_pub_odom.inputs:linearVelocity"),
                    ("isaac_compute_odom.outputs:orientation", "ros2_pub_odom.inputs:orientation"),
                    ("isaac_compute_odom.outputs:position", "ros2_pub_odom.inputs:position"),
                    ("isaac_compute_odom.outputs:orientation", "ros2_pub_raw_tf.inputs:rotation"),
                    ("isaac_compute_odom.outputs:position", "ros2_pub_raw_tf.inputs:translation"),
                ]
            }
        )
        
        compute_odom_og_path = f"{graph_path}/isaac_compute_odom"
        usd_prim = self._stage.GetPrimAtPath(compute_odom_og_path)
        usd_prim.GetRelationship("inputs:chassisPrim").AddTarget(self._kmr_prim)
        
        print(f"[+] Created {graph_path}")


    def _setup_camera_graph(self, keys):
        graph_path = "/camera_pub_graph"
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("viewport_id", "omni.graph.nodes.ConstantUInt"),
                    ("isaac_create_viewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                    ("get_prim_path", "omni.graph.nodes.GetPrimPath"),
                    ("set_active_camera", "omni.graph.ui.SetActiveViewportCamera"),
                    ("frame_id", "omni.graph.nodes.ConstantString"),
                    ("ros2_camera_helper_rgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("ros2_camera_helper_info", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("ros2_camera_helper_depth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                ],
                keys.SET_VALUES: [
                    ("viewport_id.inputs:value", 0),  # UNIQUE
                    ("ros2_context.outputs:context", 0),
                    ("frame_id.inputs:value", ROS2_FRAME_ID),
                    ("set_active_camera.inputs:primPath", "/World/Camera_1"),
                    ("ros2_camera_helper_rgb.inputs:type", "rgb"),
                    ("ros2_camera_helper_rgb.inputs:topicName", "rgb"),
                    ("ros2_camera_helper_info.inputs:type", "camera_info"),
                    ("ros2_camera_helper_info.inputs:topicName", "camera_info"),
                    ("ros2_camera_helper_depth.inputs:type", "depth"),
                    ("ros2_camera_helper_depth.inputs:topicName", "depth"),
                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "isaac_create_viewport.inputs:execIn"),
                    ("viewport_id.inputs:value", "isaac_create_viewport.inputs:viewportId"),
                    ("isaac_create_viewport.outputs:execOut", "set_active_camera.inputs:execIn"),
                    ("isaac_create_viewport.outputs:viewport", "set_active_camera.inputs:viewport"),
                    ("isaac_create_viewport.outputs:viewport", "ros2_camera_helper_rgb.inputs:viewport"),
                    ("isaac_create_viewport.outputs:viewport", "ros2_camera_helper_info.inputs:viewport"),
                    ("isaac_create_viewport.outputs:viewport", "ros2_camera_helper_depth.inputs:viewport"),
                    ("set_active_camera.outputs:execOut", "ros2_camera_helper_rgb.inputs:execIn"),
                    ("set_active_camera.outputs:execOut", "ros2_camera_helper_info.inputs:execIn"),
                    ("set_active_camera.outputs:execOut", "ros2_camera_helper_depth.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_camera_helper_rgb.inputs:context"),
                    ("ros2_context.outputs:context", "ros2_camera_helper_info.inputs:context"),
                    ("ros2_context.outputs:context", "ros2_camera_helper_depth.inputs:context"),
                    ("frame_id.inputs:value", "ros2_camera_helper_rgb.inputs:frameId"),
                    ("frame_id.inputs:value", "ros2_camera_helper_info.inputs:frameId"),
                    ("frame_id.inputs:value", "ros2_camera_helper_depth.inputs:frameId"),
                ]
            }
        )
        print(f"[+] Created {graph_path}")