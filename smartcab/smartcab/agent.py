import random
import itertools
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator

class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""
    
    sim = None
    q_hat = None
    policy = None
    
    useCustomDirection = False
        
    exploration_rate = 0.0              # epsilon
    max_exploration_rate = 0.8          # max epsilon
    min_exploration_rate = 0.01         # min epsilon
    decay_exploration_rate = 0.00085    # epsilon decay
    learning_rate = 0.9                 # alpha
    discount_rate = 0.9                 # gamma
    destination_reached_count = 0
    destination_reached_percentage = 0.0
    num_of_trials = 500

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        
        # TODO: Initialize any additional variables here
        
        direction_custom = ["Fo", "Fo/Ri", "Ri", "Ri/Ba", "Ba", "Ba/Le", "Le", "Fo/Le"]         # shortest direction to destination calculated using a custom function 
        direction_waypoint = ["forward", "left", "right"]                                       # shortest direction to destination gotten as waypoint from planner
        direction = direction_custom if self.useCustomDirection else direction_waypoint
        traffic_light = ["RED", "GRN"]                                                          # traffic light at intersection
        traffic_intersection = ["N", "O", "L", "R", "O/L", "O/R", "L/R", "O/L/R"]               # traffic situation with other cabs at intersection
        
        state_labels_combo = list(itertools.product(direction, traffic_light, traffic_intersection))
        full_state_labels_combo = ["{}-{}-{}".format(a,b,c) for a, b, c in state_labels_combo]
        full_state_labels_combo.append("GOAL")
        self.q_hat = {x:[0, 0, 0, 0] for x in full_state_labels_combo}
        self.policy = {x:-1 for x in full_state_labels_combo}
        
        self.exploration_rate = self.max_exploration_rate

    def reset(self, destination=None):
        self.planner.route_to(destination)
        # TODO: Prepare for a new trip; reset any variables here, if required

    def get_dir_to_destination(self, agent_state):
        location = agent_state['location']
        destination = agent_state['destination']
        heading = agent_state['heading']
        
        # get the direction of the destination from the cab's position in the global (up, down, right, left) co-ordinates
        col = self.env.grid_size[0]
        row = self.env.grid_size[1]
        loc_row = location[1]
        dest_row = destination[1]
        loc_col = location[0]
        dest_col = destination[0]
        
        up_dist = loc_row - dest_row
        if up_dist < 0:
            up_dist = row + up_dist
        down_dist = dest_row - loc_row
        if down_dist < 0:
            down_dist = row + down_dist
        
        right_dist = dest_col - loc_col
        if right_dist < 0:
            right_dist = col + right_dist
        left_dist = loc_col - dest_col
        if left_dist < 0:
            left_dist = col + left_dist
        
        vertical_dist = up_dist if up_dist <= down_dist else -down_dist
        horizontal_dist = right_dist if right_dist <= left_dist else -left_dist
        
        # convert cab's global (up, down, right, left) co-ordinates position to the global index co-ordinates
        index = -1
        if vertical_dist == 0 and horizontal_dist != 0:
            if horizontal_dist < 0:
                index = 6
            else:
                index = 2
        elif vertical_dist != 0 and horizontal_dist == 0:
            if vertical_dist < 0:
                index = 4
            else:
                index = 0
        elif horizontal_dist > 0:
            if vertical_dist < 0:
                index = 3
            else:
                index = 1
        elif horizontal_dist < 0:
            if vertical_dist < 0:
                index = 5
            else:
                index = 7
        
        # convert the direction of the destination from the cab's position from the global co-ordinates to the cab's local co-ordinates
        index_cab_view_local = -1
        if heading[0] == -1 and heading[1] == 0:
            index_cab_view_local = index - 6
            if index_cab_view_local < 0:
                index_cab_view_local = 8 + index_cab_view_local
        elif heading[0] == 1 and heading[1] == 0:
            index_cab_view_local = index - 2
            if index_cab_view_local < 0:
                index_cab_view_local = 8 + index_cab_view_local
        elif heading[0] == 0 and heading[1] == 1:
            index_cab_view_local = index - 4
            if index_cab_view_local < 0:
                index_cab_view_local = 8 + index_cab_view_local
        elif heading[0] == 0 and heading[1] == -1:
            index_cab_view_local = index - 0
            if index_cab_view_local < 0:
                index_cab_view_local = 8 + index_cab_view_local
        
        #print "LearningAgent.state_status(): up_dist = {}, down_dist = {}".format(up_dist, down_dist)  # [debug]
        #print "LearningAgent.state_status(): right_dist = {}, left_dist = {}".format(right_dist, left_dist)  # [debug]
        #print "LearningAgent.state_status(): vertical_dist = {}, horizontal_dist = {}".format(vertical_dist, horizontal_dist)  # [debug]
        #print "LearningAgent.state_status(): index = {}, index_cab_view_local = {}".format(index, index_cab_view_local)  # [debug]
        
        #print "LearningAgent.state_status(): state_labels_combo = {}".format(state_labels_combo)  # [debug]
        #print "LearningAgent.state_status(): full_state_labels_combo = {}".format(full_state_labels_combo)  # [debug]
        #print "LearningAgent.state_status(): full_q_hat = {}".format(full_q_hat['Ri/Ba-RED-O'][0])  # [debug]
        #print "LearningAgent.state_status(): traffic_intersection = {}".format(traffic_intersection)  # [debug]
        
        dir_to_destination = ""
        
        if index_cab_view_local == 0:
            dir_to_destination = "Fo"              # Forward Only
        elif index_cab_view_local == 1:
            dir_to_destination = "Fo/Ri"           # Forward and Right
        elif index_cab_view_local == 2:
            dir_to_destination = "Ri"              # Right Only
        elif index_cab_view_local == 3:
            dir_to_destination = "Ri/Ba"           # Right and Back
        elif index_cab_view_local == 4:
            dir_to_destination = "Ba"              # Back Only
        elif index_cab_view_local == 5:
            dir_to_destination = "Ba/Le"           # Back and Left
        elif index_cab_view_local == 6:
            dir_to_destination = "Le"              # Left Only
        elif index_cab_view_local == 7:
            dir_to_destination = "Fo/Le"           # Forward and Left
                
        return dir_to_destination
        
    def get_state_status(self, inputs, agent_state):
        state_label = self.get_dir_to_destination(agent_state) if self.useCustomDirection else self.planner.next_waypoint()
        state_label = None if (state_label == "" or state_label == None) else state_label
        
        if state_label == None:
            state_label = "GOAL"
        else:
            if inputs['light'] == 'red':
                state_label = state_label + "-RED"
            elif inputs['light'] == 'green':
                state_label = state_label + "-GRN"

            if inputs['oncoming'] == None and inputs['left'] == None and inputs['right'] == None: 
                state_label = state_label + "-N"
            elif inputs['oncoming'] != None and inputs['left'] == None and inputs['right'] == None: 
                state_label = state_label + "-O"
            elif inputs['oncoming'] == None and inputs['left'] != None and inputs['right'] == None: 
                state_label = state_label + "-L"
            elif inputs['oncoming'] == None and inputs['left'] == None and inputs['right'] != None: 
                state_label = state_label + "-R"
            elif inputs['oncoming'] != None and inputs['left'] != None and inputs['right'] == None: 
                state_label = state_label + "-O/L"
            elif inputs['oncoming'] != None and inputs['left'] == None and inputs['right'] != None: 
                state_label = state_label + "-O/R"
            elif inputs['oncoming'] == None and inputs['left'] != None and inputs['right'] != None: 
                state_label = state_label + "-L/R"
            elif inputs['oncoming'] != None and inputs['left'] != None and inputs['right'] != None: 
                state_label = state_label + "-O/L/R"
        
        return state_label

    def get_action_index(self, action):
        index = -1
    
        if action == None:
            index = 0
        elif action == 'left':
            index = 1
        elif action == 'forward':
            index = 2
        elif action == 'right':
            index = 3
            
        return index
        
    def get_action_label(self, index):
        label = ''
    
        if index == 0:
            label = None
        elif index == 1:
            label = 'left'
        elif index == 2:
            label = 'forward'
        elif index == 3:
            label = 'right'
            
        return label
    
    
    def update(self, t):
        # Gather inputs
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        inputs = self.env.sense(self)
        deadline = self.env.get_deadline(self)
                
        use_random_actions = False
        #self.get_state_status(inputs=inputs, agent_state=self.env.agent_states[self])
        
        if use_random_actions:
            # observe our current state
            state_label = self.get_state_status(inputs=inputs, agent_state=self.env.agent_states[self])
            self.state = state_label
            
            # select a random action
            action = random.choice([None, 'forward', 'left', 'right'])
            
            # determine reward for performing action in the current state
            reward = self.env.act(self, action)
            
            #print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, inputs, action, reward)  # [debug]
            
            
            #if inputs['oncoming'] != None or inputs['left'] != None or inputs['right'] != None:
                #self.sim.paused = True
                #self.sim.pause()   
        else:
            # observe our current state
            state_label = self.get_state_status(inputs=inputs, agent_state=self.env.agent_states[self])
            self.state = state_label
            
            # select action from policy
            action_index = self.policy[self.state]
            
            # add random restarts [with a probability of 1-epsilon] to solve local optima issues similar to algorithm in simulated annealing
            self.exploration_rate = self.exploration_rate - (self.exploration_rate * self.decay_exploration_rate)
            self.exploration_rate = self.exploration_rate if self.exploration_rate > self.min_exploration_rate else self.min_exploration_rate
            if (random.random() > (1-self.exploration_rate)) or (action_index < 0):  
                action_index_list = [0, 1, 2, 3]
                if action_index >= 0:
                    action_index_list.remove(action_index)
                    
                action_index = random.choice(action_index_list)
                #action_index = random.choice([1, 2, 3]) # favor action over in-action
                
                #if action_index < 0:
                #    print "LearningAgent.random(): using random policy for state action pair not yet set. actionList = {}".format(action_index_list)  # [debug]
                #else:
                #    print "LearningAgent.random(): using random, random number greater than (1-exploration_rate). actionList = {}".format(action_index_list)  # [debug]
                    
                #self.sim.paused = True
                #self.sim.pause()
                
            action = self.get_action_label(action_index)
        
            # determine reward for performing action in the current state
            reward = self.env.act(self, action)
            
            # observe the next state that we will transition into based on the current state and action
            next_state_label = self.get_state_status(inputs=inputs, agent_state=self.env.agent_states[self])
            
            # select the maximum estimated Q value from the new state and all the possible actions
            max_q_hat = -1000000.
            for i in range(len(self.q_hat[self.state])):
                if self.q_hat[next_state_label][i] > max_q_hat:
                    max_q_hat = self.q_hat[next_state_label][i]
                    
            # update the estimate Q value for the current state and action based on the values of the next state and maximum Q value of possible actions
            new_q_hat = reward + (self.discount_rate * max_q_hat)
            self.q_hat[self.state][action_index] = ((1-self.learning_rate) * self.q_hat[self.state][action_index]) + (self.learning_rate * new_q_hat)
            
            # TODO: Learn policy based on state, action, reward
            # update policy with action with max estimated Q value in this particular state
            max_q_hat = -1000000.
            for i in range(len(self.q_hat[self.state])):
                if self.q_hat[self.state][i] > max_q_hat:
                    max_q_hat = self.q_hat[self.state][i]
                    self.policy[self.state] = i
                elif self.q_hat[self.state][i] == max_q_hat:
                    self.policy[self.state] = random.choice([i, self.policy[self.state]])
                    
                    
            printDebug = {
                "update": False,
                "state": False,
                "policy": False,
                "exploration": False
            }
            
            printDebug["update"] = self.sim.debug_u if hasattr(self.sim, 'debug_u') else False
            printDebug["state"] = self.sim.debug_s if hasattr(self.sim, 'debug_s') else False
            printDebug["policy"] = self.sim.debug_p if hasattr(self.sim, 'debug_p') else False
            printDebug["exploration"] = self.sim.debug_e if hasattr(self.sim, 'debug_e') else False
            
            if printDebug["update"]:
                #print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, inputs, action, reward)  # [debug]
                print "LearningAgent.update(): deadline = {}, action = {}, reward = {}".format(deadline, action, reward)  # [debug]
                print "LearningAgent.update(): inputs = {}, next_waypoint = {}".format(inputs, self.next_waypoint)  # [debug]
            
            if printDebug["state"]:
                print "LearningAgent.current_state(): state_label = {}".format(state_label)  # [debug]
                for i in range(len(self.q_hat[self.state])):
                    print "LearningAgent.policy(): q_hat[self.state][{}] = {}".format(i, self.q_hat[self.state][i])  # [debug]
                    
                print "LearningAgent.next_state(): next_state_label = {}".format(next_state_label)  # [debug]
                for i in range(len(self.q_hat[next_state_label])):
                    print "LearningAgent.policy(): q_hat[next_state_label][{}] = {}".format(i, self.q_hat[next_state_label][i])  # [debug]
                
            if printDebug["policy"]:
                for key, val in self.policy.iteritems():
                    print "LearningAgent.policy(): policy[{}] = {}".format(key, val)  # [debug]
                    
            if printDebug["exploration"]:
                for i in range(len(self.policy)):
                    print "LearningAgent.exploration(): exploration_rate = {}".format(self.exploration_rate)  # [debug]
        
        # count number of times cab reached its destination
        # pause sim if cab reaches its destination
        location = self.env.agent_states[self]['location']
        destination = self.env.agent_states[self]['destination']
        if destination[0] == location[0] and destination[1] == location[1]:
            self.destination_reached_count = self.destination_reached_count + 1
            self.destination_reached_percentage = self.destination_reached_count * 1.0 / self.num_of_trials
            #self.sim.paused = True
            #self.sim.pause()
            
            print "LearningAgent.goal_count(): destination_reached_count = {}".format(self.destination_reached_count)  # [debug]
            print "LearningAgent.goal_percentage(): destination_reached_count = {}".format(self.destination_reached_percentage)  # [debug]
            
        
        #self.sim.paused = True
        #self.sim.pause()


def run():
    """Run the agent for a finite number of trials."""

    # Set up environment and agent
    e = Environment()  # create environment (also adds some dummy traffic)
    a = e.create_agent(LearningAgent)  # create agent
    e.set_primary_agent(a, enforce_deadline=True)  # specify agent to track
    # NOTE: You can set enforce_deadline=False while debugging to allow longer trials

    # Now simulate it
    sim = Simulator(e, update_delay=0.01, display=True)  # create simulator (uses pygame when display=True, if available)
    # NOTE: To speed up simulation, reduce update_delay and/or set display=False
    
    a.sim = sim

    sim.run(n_trials=a.num_of_trials)  # run for a specified number of trials
    # NOTE: To quit midway, press Esc or close pygame window, or hit Ctrl+C on the command-line
    


if __name__ == '__main__':
    run()
