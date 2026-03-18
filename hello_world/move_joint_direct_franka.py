import numpy as np
from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.core.utils.types import ArticulationAction

class Move_Joint_Direct_Franka(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._target_joint_positions_1 = None
        self._target_joint_positions_2 = None
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
        self.task_phase = 1

        arm_target_deg_1 = np.array([0.0, -20.0, 0.0, -120.0, 0.0, 100.0, 45.0])
        gripper_target_pos_1 = np.array([0.04, 0.04])  

        arm_target_deg_2 = np.array([0.0, -20.0, 50.0, -80.0, 0.0, 50.0, 40.0])
        gripper_target_pos_2 = np.array([0.00, 0.00])  

        arm_target_rad_1 = np.deg2rad(arm_target_deg_1)
        arm_target_rad_2 = np.deg2rad(arm_target_deg_2)

        self._target_joint_positions_1 = np.concatenate([arm_target_rad_1, gripper_target_pos_1])
        self._target_joint_positions_2 = np.concatenate([arm_target_rad_2, gripper_target_pos_2])

        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        await self._world.play_async()
        return

    def move_joint_direct(self, target_joint_positions: np.ndarray) -> bool:
        action = ArticulationAction(joint_positions=target_joint_positions)
        self._franka.apply_action(action)

        current_joint_positions = self._franka.get_joint_positions()

        is_reached = np.all(
            np.abs(current_joint_positions[:7] - target_joint_positions[:7]) < 0.01
        )

        return is_reached

    def physics_step(self, step_size):
        if self.task_phase == 1:
            if self.move_joint_direct(self._target_joint_positions_1):
                self.task_phase = 2
        elif self.task_phase == 2:
            if self.move_joint_direct(self._target_joint_positions_2):
                self.task_phase = 3 
        elif self.task_phase == 3:
            pass
        return