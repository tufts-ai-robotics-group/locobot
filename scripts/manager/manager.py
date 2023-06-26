import rospy
import actionlib
import Action
from std_msgs.msgs import String
from learner.learner import Learner
from state.observer import Observer

class Manager(object):

    def __init__(self, learner: Learner, 
                       observer: Observer):

        # Populate action lists
        self.pddl_action_list = rospy.get_param("pddl_action_list")
        if not isinstance(self.pddl_action_list, list):
            raise TypeError("Parameter 'pddl_action_list' should be a list of strings.")

        self.rl_action_list = rospy.get_param("rl_action_list")
        if not (isinstance(self.rl_action_list, list)):
            raise TypeError("Parameter 'pddl_action_list' should be a list of strings.")
        
        pddl_action_clients = {}
        for action in self.pddl_action_list:
            pddl_action_clients[action] = actionlib.SimpleActionClient(action, Action)
        
        rl_action_list = {}
        for action in self.pddl_action_list:
            rl_action_list[action] = actionlib.SimpleActionClien(action, Action)

        # Wait for action servers
        

        # Have way to start RL without failure for testing

        # Verify that pddl actions have corresponding action servers
        # Verify that primitve actions have corresponding action servers
        # Build action clients for both primitive and pddl actions
        
        # Load pddl problem and domain file
        # Create a plan file

        # Loop through actions in plan file
        # 1. Grab an action from plan file
        # 2. Parse the action, 1st is the action name, next is each argument
        # 3. Pass arguments as strings in order to the action
        # 4. Catch error if there is one
        # 5. Have ability to query action state by user
        # 6. Continue untill failure

        # If there is failure, search for action executor in folder
        # Execute executor if it exists, otherwise instantiate learner

        # Instatiate Learner
        # 1. Verify that learning agent ouput is same size as RL action space
        # 2. Verify that learning agent input is same size as observation space
        # 3. Define RL based on some parameters

        # Loop until RL is complete
        # 1. Get observation through some API
        # 2. Send observation to RL
        # 3. Get action
        # 4. Execute action

        # Loop to begining
        pass

