#Goal: Go from Positon A to Position B. Land, disarm, arm, takeoff, back to Position A. 
#yunpye@ethz.ch 
import asyncio
from mavsdk import System
async def altitude_takeoff(drone):
    position = await drone.telemetry.position().__aiter__().__anext__()
    altitude_current = position.absolute_altitude_m
    altitude_takeoff = altitude_current
    return altitude_takeoff

async def takeoff_altitude(drone,altitude):
    print("--- Taking Off to Altitude of", altitude, "meter")
    n=0

    position = await drone.telemetry.position().__aiter__().__anext__()
    altitude_current = position.absolute_altitude_m
    altitude_takeoff_A = altitude_current
    print("--- Current Altitude:", altitude_current, "meter")
    while altitude-altitude_current>0.01:
        position = await drone.telemetry.position().__aiter__().__anext__()
        altitude_current = position.absolute_altitude_m
        await drone.action.takeoff()
        
        if (n % 100==0):
            print("--- Current Altitude:", altitude_current, "meter")

        n=n+1    

async def goto_position(drone,latitude,longitude,altitude):
    print("--- Going to the Position ", latitude, " ,",longitude)
    
    n=0
    position = await drone.telemetry.position().__aiter__().__anext__()
    latitude_current = position.latitude_deg
    longitude_current = position.longitude_deg
    print("--- Current Location:", latitude_current, " ,",longitude_current)
    while abs(longitude-longitude_current)>0.0000001 and abs(latitude-latitude_current)>0.0000001:
        position = await drone.telemetry.position().__aiter__().__anext__()
        latitude_current = position.latitude_deg
        longitude_current = position.longitude_deg
        await drone.action.goto_location(latitude, longitude, altitude, 0) ##change to the local coordinates at some point
        
        if (n % 100==0):
            print("--- Current Location:", latitude_current, " ,",longitude_current)

        n=n+1     # needs to be substitute by do-while loop

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
            print("--- Current Altitude:", altitude_current, "meter")

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
               #position_takeoff = drone.telemetry.position()
               #print(position_takeoff)
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    print("-- Arming")
    await drone.action.arm()

    altitude_takeoff_A = await altitude_takeoff(drone)
    print("Takeoff Altitude at Point A = ",altitude_takeoff_A," Meter" )
    # print("--- Taking Off")
    # await drone.action.takeoff()
    await takeoff_altitude(drone,500)
    ##breaking things up with telemetry.position into robust routines takeoff, goto_positon_A, landing at position_A 
    ##set the yaw angle while landing. landing with a specific angle.
    # await asyncio.sleep(10)

    # To fly drone 20m above the ground plane
    #flying_alt = absolute_altitude + 20.0

    await goto_position(drone, 47.397606, 8.543060, 500)

    # goto_location() takes Absolute MSL altitude
    ###substitute with telemetry.position and a loop?
    
    
    ####substitute with for/break? 
    await landing_position(drone, 123)
    # print("-- Landing")
    # await drone.action.land()
    # await asyncio.sleep(30)
    
    #await drone.action.disarm()
    #print("Drone disarmed.")
    #await asyncio.sleep(10)
    
    print("-- Arming")
    await drone.action.arm()
    
    # print("--- Taking Off")
    # await drone.action.takeoff()
    await takeoff_altitude(drone,500)    

    #await asyncio.sleep(10)
    # To fly drone 20m above the ground plane
    #flying_alt = absolute_altitude + 20.0
    # goto_location() takes Absolute MSL altitude
    #await drone.action.goto_location(47.397744, 8.545625, flying_alt, 0)
    #await asyncio.sleep(50)
    await goto_position(drone, 47.397744, 8.545625, 500)
    
    # print("-- Landing")
    # await drone.action.land()
    # await asyncio.sleep(30)
    await landing_position(drone, 123)
    
    
    
    #await drone.action.return_to_launch()
    #print("--- Return to Launch")

if __name__ == "__main__":
    asyncio.run(run())
    
