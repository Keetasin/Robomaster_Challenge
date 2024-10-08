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

    
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    time.sleep(1)
    """ Code01 """
    # if 0 < yaw > 90:
    #     turn_back = yaw
    #     print(f"turn left: {turn_back} from {yaw}")
    # elif -90 < yaw < 0:
    #     turn_back = yaw 
    #     print(f"turn right: {turn_back} from {yaw}")
    # elif -180 < yaw < -90 or -90 < yaw < 180:
    #     turn_back = yaw 
    #     print(f"turn right: {turn_back} from {yaw}")
    # elif -180 < yaw < 90 or 90 < yaw < 180:
    #     turn_back = yaw 
    #     print(f"turn right: {turn_back} from {yaw}")

    """ Code02 """
    
    # origin_yaw = 0

    # if yaw > 0:
    #     yaw_r1 = abs(yaw - 90)
    #     yaw_r2 = abs(yaw - 180)

    #     if yaw_r1 < yaw_r2:
    #         turn_back = -yaw
    #     elif yaw_r2 < yaw_r1:
    #         turn_back = -yaw

    # elif yaw < 0:
    #     yaw_l1 = abs(yaw + 90)
    #     yaw_l2 = abs(yaw + 180)

    #     if yaw_l1 < yaw_l2:
    #         turn_back = -90
    #     elif yaw_l2 < yaw_l1:
    #         turn_back = -180

    # if abs(yaw) > 5:  # หาก yaw เบี่ยงเบนเกิน 5 องศา
    


    target_yaw = 0
    correction = yaw  # ปรับทิศทางกลับไปยัง 0 องศา

    if -135 < yaw <= -45:
        target_yaw = -90
        ep_chassis.move(x=0, y=0, z=correction-target_yaw, z_speed=20).wait_for_completed()  
    elif 45 < yaw < 135:
        target_yaw = 90
        ep_chassis.move(x=0, y=0, z=correction-target_yaw, z_speed=20).wait_for_completed()
    elif -45 < yaw <= 45:
        target_yaw = 0
        ep_chassis.move(x=0, y=0, z=correction, z_speed=20).wait_for_completed()
    elif -180 <= yaw < -135 :
        target_yaw = -180
        ep_chassis.move(x=0, y=0, z=correction-target_yaw, z_speed=20).wait_for_completed() 
    elif 135 < yaw <= 180:
        target_yaw = 180
        ep_chassis.move(x=0, y=0, z=correction-target_yaw, z_speed=20).wait_for_completed() 
    #     target_yaw = -180

    # ep_chassis.move(x=0, y=0, z=correction, xy_speed=20).wait_for_completed()
    print(f"Correcting yaw by {correction} degrees")
    # ep_chassis.move(x=0, y=0, z=turn_back).wait_for_completed()
    # ep_chassis.move(x=0, y=0, z=0).wait_for_completed()
    # turn_back = 0
    ep_chassis.unsub_attitude()

    ep_robot.close()

