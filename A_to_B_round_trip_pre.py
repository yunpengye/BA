#Goal: Go from Position A to Position B. Land, disarm, arm, takeoff, back to Position A. 
#yunpye@ethz.ch 
import matplotlib.pyplot as plt
import math
from statistics import mean
import time
import vg
time_loop = 0.5 
slope_simulation = 15

from mavsdk import System
import asyncio
from pyquaternion import Quaternion
import numpy as np
##from subroutines.py import xxxx
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("altitude_cruise", help="type in the desired cruise altitude",
                    type=float)
parser.add_argument("latitude_target", help="type in the desired target latitude",
                    type=float)
parser.add_argument("longitude_target", help="type in the desired target longitude",
                    type=float)
parser.add_argument("tolerance_altitude", help="type in the desired tolerance of altitude",
                    type=float)
parser.add_argument("tolerance_position", help="type in the desired tolerance of position",
                    type=float)
parser.add_argument("landing_yaw_angle", help="type in the desired yaw angle",
                    type=float)
args = parser.parse_args()

async def get_rot_from_quat(drone):
    quat = await drone.telemetry.attitude_quaternion().__aiter__().__anext__()
    w = quat.w
    x = quat.x
    y = quat.y
    z = quat.z
    my_quaternion = Quaternion(w,x,y,z)
    rot = my_quaternion.rotation_matrix
    return rot

async def get_distance_sensor(drone):
    dist = await drone.telemetry.distance_sensor().__aiter__().__anext__()
    current_distance = dist.current_distance_m
    list_current_distance = [0,0,current_distance]
    vec_current_distance = np.array(list_current_distance).T
    return vec_current_distance

async def get_local_position(drone):
    pos_and_vel = await drone.telemetry.position_velocity_ned().__aiter__().__anext__()
    position_ned = pos_and_vel.position
    north_m = position_ned.north_m
    east_m = position_ned.east_m
    down_m = position_ned.down_m
    list_local_pos = [north_m,east_m,down_m]
    vec_local_pos = np.array(list_local_pos).T
    return vec_local_pos
async def res_vector(drone):
    n=0
    arm_status = await drone.telemetry.armed().__aiter__().__anext__()
    while arm_status == 1:
        rot = await get_rot_from_quat(drone)
        #print("Rot= ",rot)
        sens = await get_distance_sensor(drone)
        #print("distance= ",sens)
        pos_local = await get_local_position(drone)
        #print("local position= ",pos_local)
        res = rot.dot(sens) + pos_local
        #print ("result= ",res)
        if (n % 50==0):
            print("---- res_vector:", res)
        n=n+1    

async def takeoff_altitude(drone,altitude,tolerance): 
    print("--- Taking Off to Altitude of", altitude, "meter")
    await drone.action.takeoff()
    position = await drone.telemetry.position().__aiter__().__anext__()
    altitude_takeoff = position.absolute_altitude_m
    latitude_takeoff = position.latitude_deg
    longitude_takeoff = position.longitude_deg
    print("--- Takeoff Altitude:", altitude_takeoff, "meter")
    n_accepted = 0
    while n_accepted < 5:
        t_start = time.time() # look up the call
        #print("current time", t_start)
        position = await drone.telemetry.position().__aiter__().__anext__()
        altitude_takeoff = position.absolute_altitude_m
        await drone.action.goto_location(latitude_takeoff, longitude_takeoff, altitude, 0)        
        #print("---- Current Altitude:", altitude_takeoff, "meter")

        if abs(altitude - altitude_takeoff) > tolerance:
            n_accepted = 0
        else:
            n_accepted += 1
        #print ("counter",n_accepted)
        dt = time.time() - t_start # look up the call
        #print("dt",dt)
        if dt < time_loop:
            await asyncio.sleep(time_loop-dt)
        

async def goto_position(drone,latitude,longitude,altitude,tolerance_meter,yaw):
    print("--- Going to the Position ", latitude, " ,",longitude)
    position = await drone.telemetry.position().__aiter__().__anext__()
    latitude_current = position.latitude_deg
    longitude_current = position.longitude_deg
    #print("--- Current Location:", latitude_current, " ,",longitude_current)
    tolerance_latitude=conversion_delta_meter_to_delta_degrees(altitude,latitude,tolerance_meter)[0]
    tolerance_longitude=conversion_delta_meter_to_delta_degrees(altitude,latitude,tolerance_meter)[1]
    n_accepted = 0
    while n_accepted < 5:
        t_start = time.time() # look up the call
        #print("current time", t_start)
        position = await drone.telemetry.position().__aiter__().__anext__()
        latitude_current = position.latitude_deg
        longitude_current = position.longitude_deg
        #print("---- Current Location:", latitude_current, " ,",longitude_current)
        await drone.action.goto_location(latitude, longitude, altitude, yaw) ##change to the local coordinates at some point
        if abs(longitude-longitude_current)>tolerance_longitude and abs(latitude-latitude_current)>tolerance_latitude:
            n_accepted = 0
        else:
            n_accepted += 1
        #print("counter",n_accepted)
        dt = time.time() - t_start # look up the call
        # print("dt",dt)
        if dt < time_loop:
            await asyncio.sleep(time_loop-dt)

async def landing_position(drone,yaw_deg):
    #Yaw angle (in degrees, frame is NED, 0 is North, positive is clockwise)
    print("--- Landing to the Position with yaw angle of", yaw_deg, "degree")
    n=0
    position = await drone.telemetry.position().__aiter__().__anext__()
    altitude_current = position.absolute_altitude_m
    latitude_current = position.latitude_deg
    longitude_current = position.longitude_deg
    arm_status = await drone.telemetry.armed().__aiter__().__anext__()
    in_air_status = await drone.telemetry.in_air().__aiter__().__anext__()
    #print("--- Current Altitude:", altitude_current, "meter")
 
    await drone.action.goto_location(latitude_current, longitude_current, altitude_current, yaw_deg)
    while in_air_status == 1:##safe landing 
        position = await drone.telemetry.position().__aiter__().__anext__()
        altitude_current = position.absolute_altitude_m
        in_air_status = await drone.telemetry.in_air().__aiter__().__anext__()
        await drone.action.land()
        
        if (n % 1==0):
            print("---- Current Altitude:", altitude_current, "meter")

        n=n+1    
    #print("Arm Status:", arm_status)
    if arm_status == 1:
        await drone.action.disarm()
        print("successfully manually disarmed")

def conversion_delta_meter_to_delta_degrees(altitude, latitude, length):
    R_earth = 6366595
    R_total = R_earth + altitude
    conversion_factor = 2*math.pi*R_total*math.cos(latitude*math.pi/180)/360
    delta_degree_latitude = length * 360 /(2*(math.pi)*R_total)
    delta_degree_longitude = length/conversion_factor
    return delta_degree_latitude, delta_degree_longitude
    
async def get_rot_from_quat(drone):
    quat = await drone.telemetry.attitude_quaternion().__aiter__().__anext__()
    w = quat.w
    x = quat.x
    y = quat.y
    z = quat.z
    my_quaternion = Quaternion(w,x,y,z)
    rot = my_quaternion.rotation_matrix
    return rot

async def get_distance_sensor(drone):
    dist = await drone.telemetry.distance_sensor().__aiter__().__anext__()
    current_distance = dist.current_distance_m
    #print(dist)
    list_current_distance = [0,0,current_distance]
    vec_current_distance = np.array(list_current_distance).T
    return vec_current_distance


async def get_local_position(drone):
    pos_and_vel = await drone.telemetry.position_velocity_ned().__aiter__().__anext__()
    euler_angles = await drone.telemetry.attitude_euler().__aiter__().__anext__()
    position_ned = pos_and_vel.position
    north_m = position_ned.north_m
    east_m = position_ned.east_m
    down_m = position_ned.down_m
    roll_deg = euler_angles.roll_deg
    pitch_deg = euler_angles.pitch_deg
    yaw_deg = euler_angles.yaw_deg
    list_euler_angles = [roll_deg, pitch_deg, yaw_deg]
    list_local_pos = [north_m,east_m,down_m]
    vec_local_pos = np.array(list_local_pos).T
    return vec_local_pos

async def get_euler_angles(drone):
    euler_angles = await drone.telemetry.attitude_euler().__aiter__().__anext__()
    roll_deg = euler_angles.roll_deg
    pitch_deg = euler_angles.pitch_deg
    yaw_deg = euler_angles.yaw_deg
    list_euler_angles = [roll_deg, pitch_deg, yaw_deg]
    return list_euler_angles

async def res_vector(drone,array_x_value, array_y_value, array_z_value,array_altitude,array_roll,array_pitch,array_yaw):
    n=0
    
    # in_air_status = await drone.telemetry.in_air().__aiter__().__anext__()
    # print(in_air_status)
    # while in_air_status == 1:
    while n<50:
        rot = await get_rot_from_quat(drone)
        #print(rot)
        sens = await get_distance_sensor(drone)
        #print("distance= ",sens)
        pos_local = await get_local_position(drone)
        #print("local position= ",pos_local)
        euler_angles = await get_euler_angles(drone)
        res = rot.dot(sens) + pos_local
        #print(sens, rot.dot(sens))
        #print ("result= ",res)
        if (n % 1==0):
            #print("---- res_vector:", res[2])
            array_x_value.append(res[0])
            array_y_value.append(res[1])
            array_z_value.append(res[2])
            array_altitude.append(pos_local[2])
            array_roll.append(euler_angles[0])
            array_pitch.append(euler_angles[1])
            array_yaw.append(euler_angles[2])
            #print(array_res)
            # print(n)
            n=n+1    

async def get_measurement(drone,array_x_value,array_y_value,array_z_value):
    array_altitude=[]
    array_roll=[]
    array_pitch=[]
    array_yaw=[]
    
    await res_vector(drone,array_x_value,array_y_value,array_z_value,array_altitude,array_roll,array_pitch,array_yaw)
    ##to plot
    # fig, axs = plt.subplots(5)
    # axs[0].plot(array_z_value,color='blue',label='z_value')
    # axs[0].legend()
    # axs[1].plot(array_altitude,color='black',label='altitude')
    # axs[1].legend()
    # axs[2].plot(array_roll, color='red',label='roll')
    # axs[2].legend()
    # axs[3].plot(array_pitch, color='yellow',label='pitch')
    # axs[3].legend()
    # axs[4].plot(array_yaw, color='green',label='yaw')
    # axs[4].legend()
    # plt.show()


async def yaw_maneuver_square(drone,target_latitude,target_longitude,length,offset_to_north):
    print("square yaw maneuver")
    array_x_value=[]
    array_y_value=[]
    array_z_value=[]
    tolerance = 1 #meter
    position = await drone.telemetry.position().__aiter__().__anext__()
    altitude_current = position.absolute_altitude_m
    result_degrees = conversion_delta_meter_to_delta_degrees(altitude_current,target_latitude, length)
    delta_degree_latitude = result_degrees[0]
    delta_degree_longitude = result_degrees[1]
    print("delta degrees=", delta_degree_latitude, delta_degree_longitude)
    #await goto_position(drone,target_latitude,target_longitude,altitude_current,tolerance,offset_to_north)
    await goto_position(drone,target_latitude,target_longitude+delta_degree_longitude,altitude_current,tolerance,offset_to_north+90)
    #get several measurement 
    await get_measurement(drone,array_x_value,array_y_value,array_z_value)
    await goto_position(drone,target_latitude+delta_degree_latitude,target_longitude,altitude_current,tolerance,offset_to_north+360-45)
    await get_measurement(drone,array_x_value,array_y_value,array_z_value)
    await goto_position(drone,target_latitude,target_longitude-delta_degree_longitude,altitude_current,tolerance,offset_to_north+180+45)
    await get_measurement(drone,array_x_value,array_y_value,array_z_value)
    await goto_position(drone,target_latitude-delta_degree_latitude,target_longitude,altitude_current,tolerance,offset_to_north+180-45)
    await get_measurement(drone,array_x_value,array_y_value,array_z_value)
    await goto_position(drone,target_latitude,target_longitude+delta_degree_longitude,altitude_current,tolerance,offset_to_north+45)
    await get_measurement(drone,array_x_value,array_y_value,array_z_value)
    await goto_position(drone,target_latitude,target_longitude,altitude_current,tolerance,offset_to_north+270)
    await get_measurement(drone,array_x_value,array_y_value,array_z_value)
    # print ("x-value=",array_x_value)
    # print ("y-value=",array_y_value)
    # print ("z-value=",array_z_value)
    # do fit
    xs=array_x_value
    ys=array_y_value
    zs=array_z_value
    # plot raw data
    plt.figure()
    ax = plt.subplot(111, projection='3d')
    ax.scatter(xs, ys, zs, color='b')

    # do fit
    tmp_A = []
    tmp_b = []
    for i in range(len(xs)):
        tmp_A.append([xs[i], ys[i], 1])
        tmp_b.append(zs[i])
    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)

    # Manual solution
    fit = (A.T * A).I * A.T * b
    errors = b - A * fit
    residual = np.linalg.norm(errors)

    # Or use Scipy
    # from scipy.linalg import lstsq
    # fit, residual, rnk, s = lstsq(A, b)

    print("solution: %f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
    # print("errors: \n", errors)
    # print("residual:", residual)
    a_plane_equation = fit[0].item()
    b_plane_equation = fit[1].item()
    gradient = math.atan2((math.sqrt(a_plane_equation**2 + b_plane_equation**2)),1)*180/math.pi
    print("slope=",gradient)
    slope_error = gradient-slope_simulation
    orientation = math.atan2(b_plane_equation,a_plane_equation)*180/math.pi ##from -180 to 180 
    print("orientation=",orientation)
    orientation_error = orientation -180
    # # plot plane
    # xlim = ax.get_xlim()
    # ylim = ax.get_ylim()
    # # ax.set_aspect('equal', adjustable='box')
    # ax.set_box_aspect([2,2,2])
    # X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
    #                 np.arange(ylim[0], ylim[1]))
    # Z = np.zeros(X.shape)
    # for r in range(X.shape[0]):
    #     for c in range(X.shape[1]):
    #         Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
    # ax.plot_wireframe(X,Y,Z, color='k')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # plt.show()

    # # -tan(15 deg)
    # vec1 = np.array([-math.tan(15/180*math.pi),0, -1])#ground truth normal vector
    # # import pdb
    # # pdb.set_trace()
    # vec2 = np.array([fit[0].item(), fit[1].item(), -1]) ##it's not normalized yet 

    # relative_angle=vg.angle(vec1, vec2)
    # print("Relative Angle between Normal Vectors=",relative_angle,"Degree")

    return slope_error, orientation_error

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        #print(health)
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            async for position in drone.telemetry.position():
               altitude = round(position.relative_altitude_m)
               print(f"Altitude: {altitude}")
               break
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude_home = terrain_info.absolute_altitude_m
        latitude_home = terrain_info.latitude_deg
        longitude_home = terrain_info.longitude_deg
        print("-- Home Location Absolute Altitude =",absolute_altitude_home,"Meter -- Home Location Latitude =",latitude_home,"Degree -- Home Location Longitude =",longitude_home, "Degree")
        break
    
    #### parameters
    altitude_cruise = args.altitude_cruise ##substitute later by parsing argument
    print("--Cruise Altitude =",altitude_cruise,"Meter")

    latitude_target = args.latitude_target##substitute later by parsing argument
    longitude_target = args.longitude_target ##substitute later by parsing argument
    print("--Target Position: Latitude=",latitude_target, "Degree, Longitude=", longitude_target, "Degree")

    tolerance_position = args.tolerance_position ##substitute later by parsing argument
    print("Position Tolerance=",tolerance_position)

    tolerance_altitude = args.tolerance_altitude ##substitute later by parsing argument
    print("Altitude Tolerance=",tolerance_altitude)
 
    yaw_angle_landing_target = args.landing_yaw_angle ##substitute later by parsing argument
    print("Yaw Angle while Landing at Target = ", yaw_angle_landing_target, "Degree")
## safety mechanism for it not to fly away
    assert abs(latitude_home - latitude_target)<1 and abs(longitude_home - longitude_target)<1 and abs(altitude_cruise-absolute_altitude_home)<1000
               
    print("-- Arming")
    await drone.action.arm()

    # altitude_takeoff_A = await altitude_takeoff(drone)
    # print("Takeoff Altitude at Point A = ",altitude_takeoff_A," Meter" )

    

    await takeoff_altitude(drone,altitude_cruise,tolerance_altitude)
    
    await goto_position(drone, latitude_target, longitude_target, altitude_cruise, tolerance_position, 0)
    
    results_dict = {
        'slope error': [],
        'd': [],
        'orientation error': []
    }
    d=1
    while d<16 :
        j=0
        while j<5:
            slope_err, ori_error = await yaw_maneuver_square(drone, latitude_target, longitude_target, d, 0)
            results_dict['slope error'].append(slope_err)
            results_dict['orientation error'].append(ori_error)
            results_dict['d'].append(d)
            np.save('errors', results_dict)
            j+=1
            print ("---------d=",d)
        d+=1


    await landing_position(drone, yaw_angle_landing_target)
    
    print("-- Arming")
    await drone.action.arm()
    
    await takeoff_altitude(drone,altitude_cruise,tolerance_altitude)    

    await goto_position(drone, latitude_home, longitude_home, altitude_cruise,tolerance_position,0)
        
    #await yaw_maneuver_square(drone, latitude_home, longitude_home, 20, yaw_angle_landing_target)

    await landing_position(drone, yaw_angle_landing_target)
    
if __name__ == "__main__":
    asyncio.run(run())
    
