# if True, connect to GUI, otherwise connect to graphic-less 'Direct' mode
is_gui_active = True

# gravity along axis x, y and z
gravity_values = (0, 0, -9.807)

# size of action vector
# action_vector_size = 6
action_vector_size = 3

# size of observation vector
# robot positions(3) + robot orientations(3) + gripper forces(6) + gripper positions(3)
# obs_vector_size = 15
obs_vector_size = 6 # x, y , z, fx, fy, fz

# initial position of the objects
plane_base_position = (0, 0, 0)
table_base_position = (0, 0, 0.18)
robot_base_position = (-0.3, -0.3, 0.815)
hole_base_position = (0.5, 0.0, 0.815)
hole_head_position = (0.5, 0.0, 0.970)

# joints of the robot
maxForce = 1000.0
maxVelocity = 0.6

totoll_simulation_steps = int(50000)
simulation_steps = int(10000)
time_resolution_per_second = 1. / 240

# RL specific configs ************************************************************
use_pretrained_model = False

do_training = True
do_test = True

num_episodes = 10

l1_params, l2_params, l3_params = 128, 128, 128
