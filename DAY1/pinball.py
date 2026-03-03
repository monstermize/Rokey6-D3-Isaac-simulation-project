# --- [1. 필요한 도구 가져오기] ---
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid, DynamicSphere
from scipy.spatial.transform import Rotation as R

# --- [2. 완벽한 초기화 (Script Editor 에러 방지 쉴드)] ---
# 켜져있는 가상 세계의 꼬인 메모리를 완전히 날려버립니다.
if World.instance() is not None:
    World.clear_instance()
world = World()

# 화면에 남아있는 이전 슬라이드와 공을 찾아서 깔끔하게 삭제합니다.
if prim_utils.is_prim_path_valid("/World/Ramps"):
    prim_utils.delete_prim("/World/Ramps")
if prim_utils.is_prim_path_valid("/World/Ball"):
    prim_utils.delete_prim("/World/Ball")

# 슬라이드들을 하나로 묶어둘 빈 폴더(/World/Ramps)를 만듭니다.
prim_utils.create_prim("/World/Ramps", "Xform")

# 무대 바닥 생성
world.scene.add_default_ground_plane()

# --- [3. 거대 지그재그 슬라이드 설계도] ---
num_ramps = 6        # 장애물 판의 총 개수
z_start = 18.0       # 첫 번째 판의 시작 높이
z_step = 3.0         # 위아래 판 사이의 높이 간격
ramp_length = 10.0   # 판의 가로 길이 (충분히 길게)
x_offset = 3.0       # 판들이 좌우로 엇갈리는 중심 거리

# --- [4. 슬라이드 생성 및 배치 루프] ---
for i in range(num_ramps):
    # 방향 결정: 짝수는 1(오른쪽), 홀수는 -1(왼쪽)
    direction = 1 if i % 2 == 0 else -1
    
    # 기울기: 30도(pi/6)로 설정하여 속도 제어
    angle = direction * (np.pi / 6)
    
    # 위치 계산: X축을 엇갈리게 배치하고, Z축(높이)은 점점 낮춥니다.
    x = -direction * x_offset 
    y = 0.0
    z = z_start - (i * z_step)
    
    # Y축 기준으로 30도 회전하는 값을 Isaac Sim의 쿼터니언 배열로 변환
    scipy_quat = R.from_euler('y', angle).as_quat()
    isaac_quat = np.array([scipy_quat[3], scipy_quat[0], scipy_quat[1], scipy_quat[2]])
    
    # 허공에 고정된 네모 판(FixedCuboid) 생성
    world.scene.add(
        FixedCuboid(
            prim_path=f"/World/Ramps/Ramp_{i}",
            name=f"ramp_{i}",
            position=np.array([x, y, z]),
            orientation=isaac_quat,
            scale=np.array([ramp_length, 1.0, 0.2]), # 두께는 얇고 길이는 길게
            color=np.array([1.0, 0.6, 0.6])          # 연분홍색 지정
        )
    )

# --- [5. 떨어지는 공 투하] ---
world.scene.add(
    DynamicSphere(
        prim_path="/World/Ball",
        name="ball",
        position=np.array([0, 0.0, z_start + 1]), # 첫 번째 판 가운데의 살짝 위에서 투하
        radius=0.4,                               # 공을 크고 시원하게 키움
        color=np.array([0.0, 1.0, 0.0]),
        mass=0.1                                  # 가벼운 질량으로 튕김 현상 조절
    )
)

# --- [6. 물리 엔진 실행] ---
world.reset()
for j in range(1000):
    world.step(render=True)
world.play()

print("초대형 슬라이드 완성! 넓고 긴 장애물을 타고 공이 안전하게 내려갑니다!")
