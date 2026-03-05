from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import RMPFlowController
from isaacsim.robot.manipulators.examples.franka.tasks import FollowTarget
import omni.kit.commands
import omni.usd


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        # 1. 시뮬레이션 환경 설정을 위해 world 객체를 가져옵니다.
        world = self.get_world()

        # 2. 로봇이 상호작용할 기본 지면(Ground Plane)을 씬(Scene)에 추가합니다.
        world.scene.add_default_ground_plane()

        # 3. URDF 파일을 파싱하고 임포트하기 위한 URDF 인터페이스를 획득합니다.
        urdf_interface = _urdf.acquire_urdf_interface()

        # 4. URDF 파일 임포트를 위한 설정을 구성합니다.
        import_config = _urdf.ImportConfig()
        import_config.convex_decomp = False  # 단순화를 위해 convex decomposition 비활성화
        import_config.fix_base = True       # 로봇의 베이스를 바닥에 고정
        import_config.make_default_prim = True  # 로봇을 씬의 기본 프림으로 설정
        import_config.self_collision = False  # 성능을 위해 자체 충돌 비활성화
        import_config.distance_scale = 1     # 로봇의 거리 스케일 설정
        import_config.density = 0.0          # 밀도를 0으로 설정 (기본값 사용)

        # 5. URDF 파일의 경로를 익스텐션으로부터 가져옵니다.
        extension_path = get_extension_path_from_name("isaacsim.asset.importer.urdf")
        root_path = extension_path + "/data/urdf/robots/franka_description/robots"
        file_name = "panda_arm_hand.urdf"

        # 6. 로봇의 URDF 파일을 파싱(Parse)하여 로봇 모델을 생성합니다.
        result, robot_model = omni.kit.commands.execute(
            "URDFParseFile",
            urdf_path="{}/{}".format(root_path, file_name),
            import_config=import_config
        )

        # 7. (중요) 더 나은 제어를 위해 관절 드라이브 파라미터(Stiffness, Damping)를 업데이트합니다.
        for joint in robot_model.joints:
            robot_model.joints[joint].drive.strength = 1047.19751  # 높은 Stiffness 값
            robot_model.joints[joint].drive.damping = 52.35988    # 적절한 Damping 값

        # 8. 수정된 로봇 모델을 현재 스테이지로 임포트(Import)하고 프림 경로를 가져옵니다.
        result, prim_path = omni.kit.commands.execute(
            "URDFImportRobot",
            urdf_robot=robot_model,
            import_config=import_config,
        )

        # 9. 로봇을 위한 사전 정의된 태스크(Task)를 초기화합니다 (예: 타겟 따라가기).
        my_task = FollowTarget(
            name="follow_target_task",
            franka_prim_path=prim_path,  # 씬(Scene)에 있는 로봇의 프림 경로
            franka_robot_name="fancy_franka",  # 로봇 인스턴스의 이름
            target_name="target"  # 로봇이 따라갈 타겟 객체의 이름
        )

        # 10. 생성된 태스크(Task)를 시뮬레이션 월드에 추가합니다.
        world.add_task(my_task)
        return

    async def setup_post_load(self):
        # 11. 씬 로드가 완료된 후, 컨트롤러 등 후속 설정을 합니다.
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("fancy_franka")

        # 12. 로봇을 위한 RMPFlow 컨트롤러를 초기화합니다.
        self._controller = RMPFlowController(
            name="target_follower_controller",
            robot_articulation=self._franka
        )

        # 13. 매 시뮬레이션 스텝마다 'physics_step' 함수가 호출되도록 물리 콜백(Callback)을 추가합니다.
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
        return

    async def setup_post_reset(self):
        # 14. 시뮬레이션이 리셋될 때마다 컨트롤러를 초기 상태로 리셋합니다.
        self._controller.reset()
        await self._world.play_async()
        return

    def physics_step(self, step_size):
        # 15. (매 스텝 실행) 시뮬레이션 스텝을 수행하고 로봇을 위한 액션을 계산합니다.
        world = self.get_world()
        observations = world.get_observations()
        
        # 16. 관측값(observations)에서 'target'의 위치와 방향 정보를 가져와 컨트롤러의 입력으로 사용합니다.
        actions = self._controller.forward(
            target_end_effector_position=observations["target"]["position"],
            target_end_effector_orientation=observations["target"]["orientation"]
        )
        
        # 17. 계산된 액션(Actions)을 로봇에 적용합니다.
        self._franka.apply_action(actions)
        return
