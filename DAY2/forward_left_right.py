# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # start the simulation app, with GUI open

import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

# preparing the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()  # add ground plane
set_camera_view(
    eye=[5.0, 0.0, 1.5], target=[0.00, 0.00, 1.00], camera_prim_path="/OmniverseKit_Persp"
)  # set camera view

# Add Franka
asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Arm")  # add robot to stage
arm = Articulation(prim_paths_expr="/World/Arm", name="my_arm")  # create an articulation object

# Add Carter
asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Car")
car = Articulation(prim_paths_expr="/World/Car", name="my_car")

# set the initial poses of the arm and the car so they don't collide BEFORE the simulation starts
arm.set_world_poses(positions=np.array([[0.0, 1.0, 0.0]]) / get_stage_units())
car.set_world_poses(positions=np.array([[0.0, -1.0, 0.0]]) / get_stage_units())

# initialize the world
my_world.reset()

for i in range(4):
    print("running cycle: ", i)
    
    if i == 0:
        print("moving forward")
        arm.set_joint_positions([[-1.5, 0.0, 0.0, -1.5, 0.0, 1.5, 0.5, 0.04, 0.04]])
        car.set_joint_velocities([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])
        
    elif i == 1:
        print("turning left")
        arm.set_joint_positions([[0.0, -1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.04, 0.04]])
        car.set_joint_velocities([[1.0, 2.0, 1.0, 2.0, 1.0, 1.0, 1.0]])
        
    elif i == 2:
        print("turning right")
        arm.set_joint_positions([[1.5, 0.5, 0.0, -0.5, 0.0, 0.5, 0.0, 0.0, 0.0]])
        car.set_joint_velocities([[4.0, 2.0, 4.0, 1.0, 1.0, 1.0, 1.0]])
        
    elif i == 3:
        print("stopping")
        arm.set_joint_positions([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        car.set_joint_velocities([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    # 2. 물리 엔진 100 프레임 실행 (명령을 실제로 수행하는 시간)
    for j in range(100):
        # 시뮬레이션 한 스텝 진행 및 화면 렌더링
        my_world.step(render=True)
        
        # 마지막 정지 사이클(i==3)일 때만 차량의 관절 상태를 출력
        if i == 3:
            car_joint_positions = car.get_joint_positions()
            print("car joint positions:", car_joint_positions)

# 3. 모든 사이클이 끝나면 시뮬레이션 앱 종료
simulation_app.close()