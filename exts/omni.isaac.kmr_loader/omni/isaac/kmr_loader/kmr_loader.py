import omni.kit.commands
import omni.graph.core as og
from pxr import Sdf, Gf, UsdPhysics, Usd, UsdGeom
from omni.isaac.core.utils.extensions import disable_extension, enable_extension
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.urdf import _urdf
import os
# from omni.isaac.core.utils.utils import get_prim_path

ENVIRONMENT_BASE_PATH = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/"
# KMR_PATH = "/home/jorgen/ros2-ws/src/kmr_description/urdf/robot/kmr.urdf"
# KMR_PATH = "/home/jorgen/ros2-ws/src/kmr_description/urdf/robot/kmr_wo_wheels.urdf"
KMR_PATH = "/home/jorgen/kmr_ws/src/kmr_description/urdf/robot/kmr_simple_camera_wo_wheels.urdf"
# OMNIWHEELS_PATH = "/home/jorgen/isaac_ws/omniwheels/"
# OMNIWHEELS_PATH = "/home/jorgen/misc_repos/o3dynsimmodel/Parts/"
OMNIWHEELS_PATH = f"{os.path.dirname(os.path.abspath(__file__))}/../../../data"
OMNIWHEELS_SCALING_FACTOR = 0.95
ROS2_CONTEXT_DOMAIN_ID = 0



class KMRLoader(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.environment = "Simple_Warehouse/warehouse_with_forklifts"
        self.create_camera_graph = False
        return

    def on_select_environment(self, env):
        self.environment = env
        print(self.environment, "selected")
    
    def on_enable_cameras(self, enable):
        print('+++ create cameras', enable)
        self.create_camera_graph = enable

    def setup_scene(self):
        disable_extension("omni.isaac.ros_bridge")
        print("[+] ROS Bridge disabled")
        enable_extension("omni.isaac.ros2_bridge")
        print("[+] ROS 2 Bridge enabled")

        self._stage = omni.usd.get_context().get_stage()
        environment_path = ENVIRONMENT_BASE_PATH + self.environment + ".usd"
        omni.kit.commands.execute("CreateReference",
            usd_context=omni.usd.get_context(),
            path_to=f"/Environment",
            asset_path=environment_path,   
        )

        result = self._load_kmr()
        self._rig_robot()
        self._setup_omnigraphs()
        self._organize_stage()
        return
    
    
    def _load_kmr(self, urdf_filepath=KMR_PATH):
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = False
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.create_physics_scene = False
        import_config.import_inertia_tensor = False
        import_config.default_drive_strength = 1047.19751
        import_config.default_position_drive_damping = 52.35988
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION  # JOINT_DRIVE_VELOCITY of JOINT_DRIVE_POSITION
        import_config.distance_scale = 1
        import_config.density = 0.0
        
        result, self._kmr_prim = omni.kit.commands.execute(
            "URDFParseAndImportFile", 
            urdf_path=urdf_filepath,
            import_config=import_config
        )
        self._base_link_frame_id = "base_link"
        self._base_link_prim_path = f'{self._kmr_prim}/{self._base_link_frame_id}'  # Later set as the articulation root

        omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{self._kmr_prim}.xformOp:translate",
                value=(0,0,0.025),
                prev=(0,0,0)
            )

        # [0, 0, 0] is underneath some shelves so when using environment "warehouse_multiple_shelves" the robot has to be moved
        if self.environment == "Simple_Warehouse/warehouse_multiple_shelves":
            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{self._kmr_prim}.xformOp:translate",
                value=(2,0,0),
                prev=(0,0,0)
            )

        # Set the correct articulation root so that tf is correct
        omni.kit.commands.execute("RemovePhysicsComponent",
            usd_prim=self._stage.GetPrimAtPath(self._kmr_prim),
            component="PhysicsArticulationRootAPI",
            multiple_api_token=None
        )
        omni.kit.commands.execute("AddPhysicsComponent",
            usd_prim=self._stage.GetPrimAtPath(self._base_link_prim_path),
            component="PhysicsArticulationRootAPI",
        )
        omni.kit.commands.execute("ChangeProperty",
            prop_path=f'{self._base_link_prim_path}.physxArticulation:enabledSelfCollisions',
            value=False,
            prev=None
        )

        return result

    def _rig_robot(self):
        res, self._lidar1_prim = self._create_lidar_sensor(is_front_lidar=True)
        res, self._lidar2_prim = self._create_lidar_sensor(is_front_lidar=False)
        self._load_omniwheels()
        self._create_cameras()
        return

    def _create_lidar_sensor(self, is_front_lidar: bool):
        if is_front_lidar:
            parent_prim = f"{self._kmr_prim}/kmr_laser_B1_link"
            yaw_offset = 0.0  # 45.0
            horizontal_fov = 246.0  # Experimental values
        else:
            parent_prim = f"{self._kmr_prim}/kmr_laser_B4_link"
            yaw_offset = 0.0  # 225.0
            horizontal_fov = 249.0  # Experimental values

        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/Lidar",
            parent=parent_prim,
            min_range=0.4,
            max_range=30.0,  # From specs
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

    def _load_omniwheels(self, omniwheels_path=OMNIWHEELS_PATH):
        """
        omnisheels_path should point to the directory where the four omniwheels with names
        omniwheel_fl.usd, omniwheel_fr.usd, omniwheel_rl.usd, omniwheel_rr.usd
        """
        # Create scope to place the omniwheels in
        omniwheel_scope_prim_path = f"{self._kmr_prim}/omniwheels"
        omni.kit.commands.execute("CreatePrimWithDefaultXformCommand",
            prim_type="Scope",
            prim_path=omniwheel_scope_prim_path,
        )
        # Create scope to place the omniwheel joints in
        joint_scope_prim_path = f"{self._kmr_prim}/omniwheel_joints"
        omni.kit.commands.execute("CreatePrimWithDefaultXformCommand",
            prim_type="Scope",
            prim_path=joint_scope_prim_path,
        )

        omniwheels = {"omniwheel_fl": "wheel_fl",  # wheel_left 
                      "omniwheel_fr": "wheel_fr",  # wheel_right
                      "omniwheel_rl": "wheel_rl",  # wheel_right
                      "omniwheel_rr": "wheel_rr"}  # wheel_left

        for prim_name, file_name in omniwheels.items():
            omniwheel_prim_path = f"{omniwheel_scope_prim_path}/{prim_name}"
            print('+++', f"{omniwheels_path}/{file_name}.usd")
            omni.kit.commands.execute("CreateReference",
                usd_context=omni.usd.get_context(),
                path_to=omniwheel_prim_path,
                asset_path=f"{omniwheels_path}/{file_name}.usd",   
            )
            omniwheel_prim = self._stage.GetPrimAtPath(omniwheel_prim_path)

            xform = UsdGeom.Xformable(omniwheel_prim)
            xform.ClearXformOpOrder()
            if prim_name[-2:] == "fl":
                UsdGeom.Xformable(omniwheel_prim).AddTranslateOp().Set((0.28, 0.1825, 0.125))  # Should be 0.28, 0.1825, 0.125
                UsdGeom.Xformable(omniwheel_prim).AddRotateXYZOp().Set((0, 0, 90))
            elif prim_name[-2:] == "fr":
                UsdGeom.Xformable(omniwheel_prim).AddTranslateOp().Set((0.28, -0.1825, 0.125))
                UsdGeom.Xformable(omniwheel_prim).AddRotateXYZOp().Set((0, 0, -90))
            elif prim_name[-2:] == "rl":
                UsdGeom.Xformable(omniwheel_prim).AddTranslateOp().Set((-0.28, 0.1825, 0.125))
                UsdGeom.Xformable(omniwheel_prim).AddRotateXYZOp().Set((0, 0, 90))
            elif prim_name[-2:] == "rr":
                UsdGeom.Xformable(omniwheel_prim).AddTranslateOp().Set((-0.28, -0.1825, 0.125))
                UsdGeom.Xformable(omniwheel_prim).AddRotateXYZOp().Set((0, 0, -90))
            UsdGeom.Xformable(omniwheel_prim).AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set((OMNIWHEELS_SCALING_FACTOR, OMNIWHEELS_SCALING_FACTOR, OMNIWHEELS_SCALING_FACTOR))

            success, joint_prim = omni.kit.commands.execute("CreateJointCommand",
                stage=self._stage,
                joint_type="Revolute",
                from_prim=self._stage.GetPrimAtPath(self._base_link_prim_path),
                to_prim=self._stage.GetPrimAtPath(f"{omniwheel_prim_path}/omniwheel_{prim_name[-2:]}")
            )
            # Joints on right side have to be rotated 180 deg around z-axis so that positive angular drive corresponds to positive velocity
            if prim_name[-1:] == "r": 
                omni.kit.commands.execute("ChangeProperty",
                    prop_path=f"{joint_prim.GetPrimPath()}.physics:localRot0",
                    value=(0.70710677, 0, 0, 0.70710677),
                    prev=(0.70710677, 0, 0, -0.70710677)
                )

            omni.kit.commands.execute("AddPhysicsComponent",
                usd_prim=joint_prim,
                component="PhysicsDrive:angular"
            )
            omni.kit.commands.execute("AddPhysicsComponent",
                usd_prim=joint_prim,
                component="PhysicsJointState:angular"
            )
            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{joint_prim.GetPrimPath()}.drive:angular:physics:damping",
                value=10_000,  # Random value used in multiple isaac tutorials. Has to be non-zero to drive
                prev=0
            )
            omni.kit.commands.execute("MovePrim",
                path_from=joint_prim.GetPrimPath(),
                path_to=f"{joint_scope_prim_path}/{prim_name[4:]}_joint"
            )
        print("[+] Created omniwheels")

    def _create_cameras(self):
        camera_suffixes = ["front", "manipulator", "right", "left"]
        self._camera_prim_paths = {}
        for camera in camera_suffixes:
            frame = "link"  # Which frame to mount the camera to
            prim_path = f"{self._kmr_prim}/camera_{camera}_{frame}/camera_{camera}"
            omni.kit.commands.execute("CreatePrimWithDefaultXform",
                prim_type="Camera",
                prim_path=prim_path,
                attributes={'focusDistance': 400, 'focalLength': 24},  # TODO: Default parameters, should be matched real cameras
            )
            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{prim_path}.xformOp:orient",
                value=(0.5, 0.5, -0.5, -0.5),
                prev=(0.5, 0.5, 0.5, 0.5)
            )
            self._camera_prim_paths[prim_path] = camera
            print(f"[+] Created {prim_path}")


    def _setup_omnigraphs(self):
        self._og_scope_prim_path = f"{self._kmr_prim}/omnigraphs"
        omni.kit.commands.execute("CreatePrimWithDefaultXformCommand",
            prim_type="Scope",
            prim_path=self._og_scope_prim_path,
        )
  
        keys = og.Controller.Keys
        self._setup_publish_clock_graph(keys)
        self._setup_twist_cmd_graph(keys)
        self._setup_iiwa_graph(keys)
        self._setup_lidar_graph(keys, is_front_lidar=True)
        self._setup_lidar_graph(keys, is_front_lidar=False)
        self._setup_tf_odom_graph(keys)

        # Concider disabeling cameras due to performance bottelnecs
        if self.create_camera_graph:
            for viewport_id, (camera_prim_path, topic_suffix) in enumerate(self._camera_prim_paths.items()):
                self._setup_camera_graph(keys, camera_prim_path, topic_suffix, viewport_id)
        return

    def _setup_tf_odom_graph(self, keys):

        # Original tf graph
        graph_path = f"{self._og_scope_prim_path}/tf_odom_graph"
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("isaac_read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ros2_pub_tf_1", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("ros2_pub_tf_2", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("isaac_compute_odom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                    ("ros2_pub_odom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                    ("ros2_pub_raw_tf", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("ros2_context.outputs:context", ROS2_CONTEXT_DOMAIN_ID),
                    ("ros2_pub_raw_tf.inputs:topicName", "tf"),
                    ("ros2_pub_odom.inputs:chassisFrameId", self._base_link_frame_id)
                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "ros2_pub_tf_1.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "ros2_pub_tf_2.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "ros2_pub_raw_tf.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "isaac_compute_odom.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "ros2_pub_odom.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_pub_tf_1.inputs:context"),
                    ("ros2_context.outputs:context", "ros2_pub_tf_2.inputs:context"),
                    ("isaac_read_sim_time.outputs:simulationTime", "ros2_pub_tf_1.inputs:timeStamp"),
                    ("isaac_read_sim_time.outputs:simulationTime", "ros2_pub_tf_2.inputs:timeStamp"),
                    ("ros2_context.outputs:context", "ros2_pub_raw_tf.inputs:context"),
                    ("isaac_compute_odom.outputs:orientation", "ros2_pub_raw_tf.inputs:rotation"),
                    ("isaac_compute_odom.outputs:position", "ros2_pub_raw_tf.inputs:translation"),
                    ("isaac_read_sim_time.outputs:simulationTime", "ros2_pub_raw_tf.inputs:timeStamp"),
                    ("ros2_context.outputs:context", "ros2_pub_odom.inputs:context"),
                    ("isaac_read_sim_time.outputs:simulationTime", "ros2_pub_odom.inputs:timeStamp"),
                    ("isaac_compute_odom.outputs:angularVelocity", "ros2_pub_odom.inputs:angularVelocity"),
                    ("isaac_compute_odom.outputs:linearVelocity", "ros2_pub_odom.inputs:linearVelocity"),
                    ("isaac_compute_odom.outputs:orientation", "ros2_pub_odom.inputs:orientation"),
                    ("isaac_compute_odom.outputs:position", "ros2_pub_odom.inputs:position"),
                ]
            }
        )
        ros2_pub_tf_1_prim = self._stage.GetPrimAtPath(f"{graph_path}/ros2_pub_tf_1")
        ros2_pub_tf_1_prim.GetRelationship("inputs:targetPrims").AddTarget(self._base_link_prim_path)        
        ros2_pub_tf_1_prim.GetRelationship("inputs:parentPrim").AddTarget(self._base_link_prim_path)     
        
        ros2_pub_tf_2_prim = self._stage.GetPrimAtPath(f"{graph_path}/ros2_pub_tf_2")
        ros2_pub_tf_2_prim.GetRelationship("inputs:parentPrim").AddTarget(self._base_link_prim_path)
        ros2_pub_tf_2_prim.GetRelationship("inputs:targetPrims").AddTarget(f'{self._kmr_prim}/kmr_laser_B1_link')        
        ros2_pub_tf_2_prim.GetRelationship("inputs:targetPrims").AddTarget(f'{self._kmr_prim}/kmr_laser_B4_link')

        compute_odom_prim = self._stage.GetPrimAtPath(f"{graph_path}/isaac_compute_odom")
        compute_odom_prim.GetRelationship("inputs:chassisPrim").AddTarget(self._base_link_prim_path)
        print(f"[+] Created {graph_path}")

    def _setup_publish_clock_graph(self, keys):
        graph_prim_path = f"{self._og_scope_prim_path}/publish_clock_graph"
        og.Controller.edit(
            {"graph_path": graph_prim_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("isaac_read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ros2_publish_clock", "omni.isaac.ros2_bridge.ROS2PublishClock")
                ],
                keys.SET_VALUES: [
                    ("ros2_context.outputs:context", ROS2_CONTEXT_DOMAIN_ID),

                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "ros2_publish_clock.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_publish_clock.inputs:context"),
                    ("isaac_read_sim_time.outputs:simulationTime", "ros2_publish_clock.inputs:timeStamp"),
                ]
            }
        )
    
    def _setup_twist_cmd_graph(self, keys):
        graph_prim_path = f"{self._og_scope_prim_path}/twist_cmd_graph"
        og.Controller.edit(
            {"graph_path": graph_prim_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ros2_subscribe_twist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                    ("break_3_vector_rot", "omni.graph.nodes.BreakVector3"),
                    ("break_3_vector_vel", "omni.graph.nodes.BreakVector3"),
                    ("convert_to_stage_units", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                    ("make_3_vector", "omni.graph.nodes.MakeVector3"),
                    ("holonomic_controller", "omni.isaac.wheeled_robots.HolonomicController"),
                    ("articulation_controller", "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    ("holonomic_controller.inputs:upAxis", [0, 0, 1]),
                    ("holonomic_controller.inputs:wheelAxis", [0, 1, 0]),
                    ("holonomic_controller.inputs:wheelRadius", [0.1289999932050705 * OMNIWHEELS_SCALING_FACTOR] * 4),  # 0.1289999932050705 from O3dyn model scaled down with 0.875. Same as wheels
                    ("holonomic_controller.inputs:wheelOrientations", [[-0.5, 0.5, 0.5, -0.5], 
                                                                       [ 0.5, 0.5, 0.5,  0.5], 
                                                                       [-0.5, 0.5, 0.5, -0.5], 
                                                                       [ 0.5, 0.5, 0.5,  0.5]]),
                    ("holonomic_controller.inputs:wheelPositions", [( 0.28,  0.1825, -0.2), 
                                                                    ( 0.28, -0.1825, -0.2), 
                                                                    (-0.28,  0.1825, -0.2), 
                                                                    (-0.28, -0.1825, -0.2)]),
                    ("holonomic_controller.inputs:mecanumAngles", [-135, -45, -45, -135]),
                    ("holonomic_controller.inputs:maxLinearSpeed", 1.0),  # Max speed according to specifications
                    ("holonomic_controller.inputs:maxAngularSpeed", 0.51),  # Max speed according to specifications
                    ("holonomic_controller.inputs:angularGain", -1.0),  # Roration is oposite
                    ("articulation_controller.inputs:jointNames", ["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"]),
                    ("articulation_controller.inputs:usePath", False),
                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "articulation_controller.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "ros2_subscribe_twist.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_subscribe_twist.inputs:context"),
                    ("ros2_subscribe_twist.outputs:execOut", "holonomic_controller.inputs:execIn"),
                    ("ros2_subscribe_twist.outputs:angularVelocity", "break_3_vector_rot.inputs:tuple"),
                    ("ros2_subscribe_twist.outputs:linearVelocity", "convert_to_stage_units.inputs:value"),
                    ("convert_to_stage_units.outputs:result", "break_3_vector_vel.inputs:tuple"),
                    ("break_3_vector_rot.outputs:z", "make_3_vector.inputs:z"),
                    ("break_3_vector_vel.outputs:x", "make_3_vector.inputs:x"),
                    ("break_3_vector_vel.outputs:y", "make_3_vector.inputs:y"),
                    ("make_3_vector.outputs:tuple", "holonomic_controller.inputs:velocityCommands"),
                    ("holonomic_controller.outputs:jointVelocityCommand", "articulation_controller.inputs:velocityCommand"),
                ]
            }
        )
        usd_prim = self._stage.GetPrimAtPath(f"{graph_prim_path}/articulation_controller")
        usd_prim.GetRelationship("inputs:targetPrim").AddTarget(self._base_link_prim_path)
        print(f"[+] Created {graph_prim_path}")

    def _setup_iiwa_graph(self, keys):
        # Example from https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_manipulation.html#add-joint-states-in-extension
        # TODO: Check that it publishes and subscribes as intended
        graph_prim_path = f"{self._og_scope_prim_path}/iiwa_controller_graph"
        og.Controller.edit(
            {"graph_path": graph_prim_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                    ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                    ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                    ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ],
                keys.SET_VALUES: [
                    # Providing path to /panda robot to Articulation Controller node
                    # Providing the robot path is equivalent to setting the targetPrim in Articulation Controller node
                    ("ArticulationController.inputs:usePath", True),
                    ("ArticulationController.inputs:robotPath", self._base_link_prim_path),
                ],
            },
        )
        # Setting the /panda target prim to Publish JointState node
        set_target_prims(primPath=f"{graph_prim_path}/PublishJointState", targetPrimPaths=[f"{self._base_link_prim_path}"])
        print(f"[+] Created {graph_prim_path}")

    def _setup_lidar_graph(self, keys, is_front_lidar: bool):
        if is_front_lidar:
            lidar_num = 1
            lidar_frame_id = "kmr_laser_B1_link"
        else:
            lidar_num = 2
            lidar_frame_id = "kmr_laser_B4_link"

        lidar_prim_path = f"{self._kmr_prim}/{lidar_frame_id}/Lidar"
        graph_path = f"{self._og_scope_prim_path}/lidar{lidar_num}_graph"
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("isaac_read_lidar_beam_node", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("isaac_read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ros2_pub_laser_scan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                ],
                keys.SET_VALUES: [
                    ("ros2_pub_laser_scan.inputs:topicName", f"/laser_scan{lidar_num}"),
                    ("ros2_context.outputs:context", ROS2_CONTEXT_DOMAIN_ID),
                    ("ros2_pub_laser_scan.inputs:frameId", lidar_frame_id),
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
                    ("isaac_read_sim_time.outputs:simulationTime", "ros2_pub_laser_scan.inputs:timeStamp"),
                ],
            }
        )
        usd_prim = self._stage.GetPrimAtPath(f"{graph_path}/isaac_read_lidar_beam_node")
        usd_prim.GetRelationship("inputs:lidarPrim").AddTarget(lidar_prim_path)
        print(f"[+] Created {graph_path}")

    def _setup_camera_graph(self, keys, camera_prim_path, topic_suffix, viewport_id):
        graph_path = f"{self._og_scope_prim_path}/camera_{topic_suffix}_pub_graph"
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
                    ("viewport_id.inputs:value", viewport_id),  # UNIQUE
                    ("ros2_context.outputs:context", ROS2_CONTEXT_DOMAIN_ID),
                    ("frame_id.inputs:value", f"camera_{topic_suffix}_link"),  # TODO: "..._link" can be any other frame connected to the camera. Should be able to customize
                    ("set_active_camera.inputs:primPath", camera_prim_path),
                    ("ros2_camera_helper_rgb.inputs:type", "rgb"),
                    ("ros2_camera_helper_rgb.inputs:topicName", f"rgb_{topic_suffix}"),
                    ("ros2_camera_helper_info.inputs:type", "camera_info"),
                    ("ros2_camera_helper_info.inputs:topicName", f"camera_info"),
                    ("ros2_camera_helper_depth.inputs:type", "depth"),
                    ("ros2_camera_helper_depth.inputs:topicName", f"depth_{topic_suffix}"),
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


    def _organize_stage(self):
        # TODO: See if time to implement this
        omni.kit.commands.execute("CreatePrimWithDefaultXformCommand",
            prim_type="Scope",
            prim_path=f"{self._kmr_prim}/cameras",
        )
        omni.kit.commands.execute("CreatePrimWithDefaultXformCommand",
            prim_type="Scope",
            prim_path=f"{self._kmr_prim}/gripper",
        )
        omni.kit.commands.execute("CreatePrimWithDefaultXformCommand",
            prim_type="Scope",
            prim_path=f"{self._kmr_prim}/arm",
        )

        


    async def setup_post_load(self):
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
