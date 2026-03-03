import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
from scipy.spatial.transform import Rotation as R

# 가상 세계 설정 및 씬 초기화
if World.instance() is None:
    world = World()
else:
    world = World.instance()

world.scene.clear()
world.scene.add_default_ground_plane()

# 파라미터 설정
domino_count = 20
radius = 2.0
height = 0.6
thickness = 0.1
width = 0.3

# 도미노 배치 계산
angles = np.linspace(0, np.pi, domino_count)

for i, angle in enumerate(angles):
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = height / 2.0
    
    # 넓은 면이 궤적에 수직이 되도록 90도 회전
    scipy_quat = R.from_euler('z', angle + np.pi / 2).as_quat()
    isaac_quat = np.array([scipy_quat[3], scipy_quat[0], scipy_quat[1], scipy_quat[2]])
    
    DynamicCuboid(
        prim_path=f"/World/Domino_{i}",
        name=f"domino_{i}",
        position=np.array([x, y, z]),
        orientation=isaac_quat,
        scale=np.array([thickness, width, height]), 
        color=np.array([1.0, 0.0, 0.0]),
        mass=1.0
    )

# 수정된 부분: 공을 첫 도미노(x=2.0) 정면의 Y축 선상에 배치
start_x = radius
start_y = -2.0 

ball = DynamicSphere(
        prim_path="/World/Ball",
        name="ball",
        position=np.array([start_x, start_y, height / 2.0]),
        radius=0.15,
        color=np.array([0.0, 1.0, 0.0]),
        mass=10.0,
        # 물리 엔진 씹힘 방지 및 도미노 면에 수직(Y축)으로 강제 발사
        linear_velocity=np.array([0.0,3.0, 0.0]) 
    )

# 물리 엔진 시작
world.reset()
world.play()

print("완벽한 타격 위치 설정 완료! 공이 수직으로 날아가 도미노를 강타합니다!")
