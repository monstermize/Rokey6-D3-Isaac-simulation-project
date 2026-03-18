#!/usr/bin/env python3
import sys

# 1. Isaac Sim 엔진 실행 (ROS 2 초기화보다 무조건 먼저 와야 합니다)
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import rclpy
from rclpy.node import Node
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim  # 💡 조명을 만들기 위한 라이브러리 추가

def main(args=None):
    # 2. ROS 2 노드 초기화
    rclpy.init(args=args)
    node = Node('isaac_sim_map_node')
    
    # 3. 월드 생성 및 맵 불러오기
    world = World()
    map_usd_path = "/home/rokey/IsaacSim-ros_workspaces/humble_ws/src/project/resource/map.usd"
    
    node.get_logger().info(f"맵을 불러오는 중입니다: {map_usd_path}")
    add_reference_to_stage(usd_path=map_usd_path, prim_path="/World/Map")

    # ★ 4. 전체를 환하게 밝히는 조명(DomeLight) 추가 ★
    node.get_logger().info("하늘에 조명을 켭니다 💡")
    create_prim(
        prim_path="/World/SkyLight",
        prim_type="DomeLight",
        attributes={
            "inputs:intensity": 1000.0,  # 빛의 강도 (너무 어두우면 2000, 3000으로 올려보세요)
            "inputs:color": (1.0, 1.0, 1.0) # 빛의 색상 (흰색)
        }
    )
    
    world.reset()
    node.get_logger().info("맵 불러오기 완료! 시뮬레이션을 시작합니다.")

    # 5. 시뮬레이션 & ROS 2 동시 실행 루프
    while simulation_app.is_running() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        world.step(render=True)

    # 6. 종료 시 안전하게 닫기
    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == '__main__':
    main()