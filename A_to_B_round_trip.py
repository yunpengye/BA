#Goal: Go from Positon A to Position B. Land, disarm, arm, takeoff, back to Position A. 
#yunpye@ethz.ch 
import asyncio
import argparse
parser = argparse.ArgumentParser()
parser.parse_args()

from mavsdk import System
async def altitude_takeoff(drone):
    position = await drone.telemetry.position().__aiter__().__anext__()
    altitude_current = position.absolute_altitude_m
    altitude_takeoff = altitude_current
    return altitude_takeoff

async def takeoff_altitude(drone,altitude,tolerance):
    print("--- Taking Off to Altitude of", altitude, "meter")
    n=0

    position = await drone.telemetry.position().__aiter__().__anext__()
    altitude_current = position.absolute_altitude_m
    altitude_takeoff_A = altitude_current
    print("--- Current Altitude:", altitude_current, "meter")
    while altitude-altitude_current>tolerance:
        position = await drone.telemetry.position().__aiter__().__anext__()
        altitude_current = position.absolute_altitude_m
        await drone.action.takeoff()
        
        if (n % 50==0):
            print("---- Current Altitude:", altitude_current, "meter")

        n=n+1    

async def goto_position(drone,latitude,longitude,altitude,tolerance):
    print("--- Going to the Position ", latitude, " ,",longitude)
    
    n=0
    position = await drone.telemetry.position().__aiter__().__anext__()
    latitude_current = position.latitude_deg
    longitude_current = position.longitude_deg
    print("--- Current Location:", latitude_current, " ,",longitude_current)
    while abs(longitude-longitude_current)>tolerance and abs(latitude-latitude_current)>tolerance:
        position = await drone.telemetry.position().__aiter__().__anext__()
        latitude_current = position.latitude_deg
        longitude_current = position.longitude_deg
        await drone.action.goto_location(latitude, longitude, altitude, 0) ##change to the local coordinates at some point
        
        if (n % 100==0):
            print("---- Current Location:", latitude_current, " ,",longitude_current)

        n=n+1    

async def landing_position(drone,yaw_deg):
    #Yaw angle (in degrees, frame is NED, 0 is North, positive is clockwise)
    print("--- Landing to the Position with yaw angle of", yaw_deg, "degree")
    n=0
    position = await drone.telemetry.position().__aiter__().__anext__()
    altitude_current = position.absolute_altitude_m
    arm_status = await drone.telemetry.armed().__aiter__().__anext__()
    print("--- Current Altitude:", altitude_current, "meter")
    while arm_status == 1:##safe landing?
        position = await drone.telemetry.position().__aiter__().__anext__()
        altitude_current = position.absolute_altitude_m
        arm_status = await drone.telemetry.armed().__aiter__().__anext__()
        await drone.action.land()
        
        if (n % 1==0):
            print("---- Current Altitude:", altitude_current, "meter")

        n=n+1    
    

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
    
    print("-- Arming")
    await drone.action.arm()

    altitude_takeoff_A = await altitude_takeoff(drone)
    print("Takeoff Altitude at Point A = ",altitude_takeoff_A," Meter" )

    #### parameters
    altitude_cruise = 510 ##substitute later by parsing argument
    print("--Cruise Altitude =",altitude_cruise,"Meter")

    latitude_target = 47.397606 ##substitute later by parsing argument
    longitude_target = 8.543060 ##substitute later by parsing argument
    print("--Target Position: Latitude=",latitude_target, "Degree, Longitude=", longitude_target, "Degree")

    tolerance_position = 0.0000001 ##substitute later by parsing argument
    print("Position Tolerance=",tolerance_position)

    tolerance_altitude = 0 ##substitute later by parsing argument
    print("Altitude Tolerance=",tolerance_altitude)
 
    yaw_angle_landing_target = 123 ##substitute later by parsing argument
    print("Yaw Angle while Landing at Target = ", yaw_angle_landing_target, "Degree")

    await takeoff_altitude(drone,altitude_cruise,tolerance_altitude)

    await goto_position(drone, latitude_target, longitude_target, altitude_cruise, tolerance_position)
    
    await landing_position(drone, yaw_angle_landing_target)
    
    print("-- Arming")
    await drone.action.arm()
    
    await takeoff_altitude(drone,altitude_cruise,tolerance_altitude)    

    await goto_position(drone, latitude_home, longitude_home, altitude_cruise,tolerance_position)
    
    await landing_position(drone, yaw_angle_landing_target)
    
if __name__ == "__main__":
    asyncio.run(run())
    
