from absl import flags


FLAGS = flags.FLAGS

flags.DEFINE_enum('planning_method', ['rrt', 'prm', 'gcs', 'gcs*'], 'Planning method to use') #TODO: update with all methods to test
flags.DEFINE_string('planning_env', '2d', 'Environment to use') # TODO: Update with list of envs
flags.DEFINE_string("footstep_planner", "tbd", "Footstep planner to use") #TODO: footstep planner to use
