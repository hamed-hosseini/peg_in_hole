import pybullet as p
import time
from env import Env
# Initialize PyBullet in GUI mode
p.connect(p.GUI)

# Enable real-time simulation to continuously update the visualization
p.setRealTimeSimulation(True)

# Load your environment and other setup code
env = Env()

# Main loop
import pybullet as p

time_step = 1.0 / 240.0

# Main simulation loop
while True:
    # Step the simulation forward
    # p.stepSimulation()

    # Add a small delay to control the simulation speed (optional)
    # You can adjust the delay time based on your desired simulation speed
    # p.setTimeStep(time_step)
    # p.syncPhysics()
    # run the manual simulation loop
    env.run_simulation()

p.disconnect()
