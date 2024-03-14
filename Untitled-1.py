"""
"""

from __future__ import print_function
from src.env    import VrepEnvironment
from src.agents import Pioneer
from src.disp   import Display
import settings
import time, argparse
import matplotlib.pyplot as plt
import random

""" Motors:

          1. agent.change_velocity([ speed_left: float, speed_right: float ]) 
               Set the target angular velocities of left
               and right motors with a LIST of values:
               e.g. [1., 1.] in radians/s.
               
               Values in range [-5:5] (above these 
               values the control accuracy decreases)
                    
          2. agent.current_speed_API() 
          ----
               Returns a LIST of current angular velocities
               of the motors
               [speed_left: float, speed_right: float] in radians/s.

    Lidar:
          3. agent.read_lidars()   
          ----
               Returns a list of floating point numbers that 
               indicate the distance towards the closest object at a particular angle from the robot's lidar position.
               
               Basic configuration of the lidar:
               Lidar distance reading list: 270 lidar distance readings starting from 134 deg  to 1 deg 
               followed by a dummy-value and further readings from -1 deg to -135 deg. 
               That is: A total of 270 real values representing distance readings starting with the right-most lidar point/reading and going anti-clockwise
               Note: ignore center-element, number 135 (is index 134 as we start counting from 0), of the list, it has always value 1.0

    Agent:
          You can access these attributes to get information about the agent's positions

          4. agent.pos  

          ----
               Current x,y position of the agent (derived from 
               SLAM data) Note: unreliable as SLAM is not solved here.

          5. agent.position_history

               A deque containing N last positions of the agent 
               (200 by default, can be changed in settings.py) Note: unreliable as SLAM is not solved here.
"""

###########
###########

def loop(agent):

    """
    Robot control loop 
    Your code goes here
    """
    # Example:
    x = 0
    print("vel: {}".format(agent.current_speed_API()))
    current_velocity = agent.current_speed_API() 
    sum = 0
    scan_average = []
    array = agent.read_lidars()
    for i, data in enumerate(array):
        sum += data
        if i == 269:
            average = sum/6
            scan_average.append(average)
            break
        if i % 6 == 0:
            average = sum / 6
            scan_average.append(average)
            print("direction: {} degrees, average: {}".format(i, str(average)))
            sum = 0
    



    closest = agent.find_closest()
    print("ang lef: {} and dist {}".format(closest[0][0], closest[1][0]))
    print("ang des: {} and dist {}".format(closest[0][1], closest[1][1]))
    if closest[1][0] < 1 and closest[1][1] < 1:
        if 40 < abs(closest[0][0]) < 95 and 40 < abs(closest[0][1]) < 95 and scan_average[21] > 4 and scan_average[22] > 4:
            agent.change_velocity([-1,-1])
            time.sleep(2)
            agent.change_velocity([3.5,3])
            time.sleep(6)
            print("vado a bbombaaa")
            return

    # if abs(closest[0][0]) < 15 and closest[1][0] < 0.4 or abs(closest[0][1]) < 15 and closest[1][1] < 0.4 or abs(closest[0][0]) > 130 or abs(closest[0][1]) > 130 :
    #     agent.change_velocity([-1,-1])
    #     time.sleep(2)
    #     agent.change_velocity([2.5,1.5])
    #     time.sleep(3)
    #     # return
    
    i = 1 
    found = False
    for distance in scan_average[1:len(scan_average) - 1]:
        if abs(distance - scan_average[i - 1]) > 2 and abs(distance - scan_average[i + 1]) > 2:
            print("trovatooo")
            direction = i * 6
            print("direction is {}".format(direction ))
            found = True
            break
        i += 1
    if found == False:
        print("falseee")
    else:
        move(direction)
    current_velocity = agent.current_speed_API()
    if current_velocity[0] < 0.01 and current_velocity[1] < 0.01:
        rand = random.randint(1,2)
        if rand == 1:
            agent.change_velocity([-2,-2])
            time.sleep(4)
            return
        else:
            agent.change_velocity([0.5,0.5])
            time.sleep(3)
            return
    print("vel2: {}".format(agent.current_speed()))
    print("vel_API {}".format(agent.current_speed_API()))
    print("possss {}".format(agent.position_history))
##########
##########
def move(direction):
    # Define velocity mappings for each 20-degree range
    velocity_mapping = {
        (0, 20): [3, 1.5],    # Velocity for 0-20 degrees
        (20, 40): [2.9, 1.6],  # Velocity for 20-40 degrees
        (40, 60): [2.7, 1.8],  # Velocity for 40-60 degrees
        (60, 80): [2.5, 1.9],
        (80, 100): [2.35, 2],
        (100, 120): [2.2, 2],
        (120, 140): [2.3, 2],
        (140, 160): [2, 2.3],
        (160, 180): [2, 2.3],
        (180, 200): [1.9, 2,5],
        (200, 220): [1.8, 2.7],
        (220, 240): [1.6, 2.9],
        (240, 260): [1.5, 3],
    }

    # Find the appropriate velocity based on the direction
    for angle_range, velocity in velocity_mapping.items():
        if angle_range[0] <= direction <= angle_range[1]:
            agent.change_velocity(velocity)
            print("Moving forward with velocity: {}".format(velocity))
            time.sleep(4)  # Pause for 5 seconds (adjust as needed)
            return  # Exit the loop after setting velocity

    # If direction doesn't fall within any range, print an error message
    print("Direction {} is not within a valid range.".format(direction))


#######
#######
if __name__ == "__main__":
    plt.ion()
    # Initialize and start the environment
    environment = VrepEnvironment(settings.SCENES + '/room_static.ttt')  # Open the file containing our scene (robot and its environment)
    environment.connect()        # Connect python to the simulator's remote API
    agent   = Pioneer(environment)
    display = Display(agent, False)

    print('\nDemonstration of Simultaneous Localization and Mapping using CoppeliaSim robot simulation software. \nPress "CTRL+C" to exit.\n')
    start = time.time()
    step  = 0
    done  = False
    environment.start_simulation()
    time.sleep(1)

    try:
        while step < settings.simulation_steps and not done:
            display.update()                      # Update the SLAM display
            loop(agent)                           # Control loop
            step += 1
    except KeyboardInterrupt:
        print('\n\nInterrupted! Time: {}s'.format(time.time()-start))

    display.close()
    environment.stop_simulation()
    environment.disconnect()
