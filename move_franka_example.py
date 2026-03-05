import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.prims import SingleArticulation
import numpy as np
from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.robot.manipulators.examples.franka import Franka

class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name: str, robot_articulation: SingleArticulation, physics_dt: float = 1.0 / 60.0) -> None:
        self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**self.rmp_flow_config)
        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)
        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        (
            self._default_position,
            self._default_orientation,
        ) = self._articulation_motion_policy._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )

class Move_Franka(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return
    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        franka = world.scene.add(
            Franka(
                prim_path="/World/Fancy_Franka",
                name="fancy_franka"
            )
        )
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("fancy_franka")
        self.cspace_controller = RMPFlowController(name="pick_place_controller" + "_cspace_controller", robot_articulation=self._franka)
        
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)

        self._goal_points = [
            np.array([0.3, 0.3, 0.3]),
            np.array([0.3, -0.3, 0.3]),
            np.array([0.6, 0.3, 0.6]),
            np.array([0.1, -0.7, 0.2])  
        ]
        self.task_phase = 0
        self._goal_reached = False

        await self._world.play_async()
        return
    
    def move_point(self, goal_position: np.ndarray, end_effector_orientation: np.ndarray=np.array([0, np.pi, 0])) -> bool:

        end_effector_orientation = euler_angles_to_quat(end_effector_orientation)
        
        target_joint_positions = self.cspace_controller.forward(
            target_end_effector_position=goal_position, 
            target_end_effector_orientation=end_effector_orientation
        )
        
        self._franka.apply_action(target_joint_positions)
        
        current_joint_positions = self._franka.get_joint_positions()
        is_reached = np.all(np.abs(current_joint_positions[:7] - target_joint_positions.joint_positions) < 0.001)
        
        return is_reached

    def physics_step(self, step_size):
        if self.task_phase == 0:
            current_goal = self._goal_points[self.task_phase]
            self._goal_reached = self.move_point(current_goal)
            if self._goal_reached:
                self.cspace_controller.reset()
                self.task_phase = 1
        elif self.task_phase == 1:
            current_goal = self._goal_points[self.task_phase]
            self._goal_reached = self.move_point(current_goal, end_effector_orientation=np.array([np.pi/2, np.pi/2, 0]))
            if self._goal_reached:
                self.cspace_controller.reset()
                self.task_phase = 2
        elif self.task_phase == 2:
            current_goal = self._goal_points[self.task_phase]
            self._goal_reached = self.move_point(current_goal, end_effector_orientation=np.array([np.pi/2, np.pi/2, 0]))
            if self._goal_reached:
                self.cspace_controller.reset()
                self.task_phase = 3
        elif self.task_phase == 3:
            current_goal = self._goal_points[self.task_phase]
            self._goal_reached = self.move_point(current_goal, end_effector_orientation=np.array([np.pi/2, np.pi/2, 0]))
            if self._goal_reached:
                self.cspace_controller.reset()
                self.task_phase = 3
        return
