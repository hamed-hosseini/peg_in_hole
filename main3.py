import pybullet as p
import time
from env import Env
# Initialize PyBullet in GUI mode
p.connect(p.GUI)

# Enable real-time simulation to continuously update the visualization
p.setRealTimeSimulation(True)

# Load your environment and other setup code
env = Env()

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



import numpy as np

def objective_function(x):
    # Define the objective function to be optimized
    return -(x ** 2)  # Negative sign because we want to maximize

def hill_climbing(max_iterations, step_size):
    # Initialize the current solution randomly
    current_solution = np.random.uniform(-10, 10)

    for i in range(max_iterations):
        # Evaluate the current solution
        current_value = objective_function(current_solution)

        # Generate a neighbor solution by perturbing the current solution
        neighbor_solution = current_solution + np.random.uniform(-step_size, step_size)

        # Evaluate the neighbor solution
        neighbor_value = objective_function(neighbor_solution)

        # If the neighbor solution is better, move to it
        if neighbor_value > current_value:
            current_solution = neighbor_solution

    return current_solution, objective_function(current_solution)

if __name__ == "__main__":
    max_iterations = 100
    step_size = 0.1

    best_solution, best_value = hill_climbing(max_iterations, step_size)

    print(f"Best Solution: {best_solution}")
    print(f"Best Value: {best_value}")

