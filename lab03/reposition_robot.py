# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import robomaster
from robomaster import robot
import time
yaw = 0
turn_back = 0


def sub_attitude_info_handler(attitude_info):
    global yaw  
    yaw, pitch, roll = attitude_info
    print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(yaw, pitch, roll))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    # 订阅底盘姿态信息
    #yaw pitch roll
    # for i in range(0, 10):
    # time.sleep(1)
    # print(yaw)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    time.sleep(1)
    # for i in range(10):
    if 0 <= yaw > 10:
        turn_back = yaw
        print(f"turn left: {turn_back} from {yaw}")
    elif -10 <= yaw < 0:
        turn_back = yaw 
        print(f"turn right: {turn_back} from {yaw}")
        # print(i)
    ep_chassis.move(x=0, y=0, z=turn_back).wait_for_completed()
    ep_chassis.unsub_attitude()
    # ep_chassis.move(x=0, y=0, z=20).wait_for_completed()
    # ep_chassis.move(x=0, y=0, z=-90+90).wait_for_completed()
        # print(i)

    ep_robot.close()
