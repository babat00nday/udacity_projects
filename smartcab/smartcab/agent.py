import random
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator

class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""
    
    sim = None
    q_hat = None
    policy = None
        
    exploration_rate = 0.0              # epsilon
    max_exploration_rate = 0.8          # max epsilon
    min_exploration_rate = 0.01         # min epsilon
    decay_exploration_rate = 0.00085    # epsilon decay
    learning_rate = 0.9                 # alpha
    discount_rate = 0.9                 # gamma

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        
        # TODO: Initialize any additional variables here
        num_of_states = 25  # 1 goal state plus (8 directional states * 3 traffic situation states) 24 informational states
        num_of_actions = 4 # 4 actions per state {"none": 0, "left": 0, "forward": 0, "right": 0}
        self.q_hat = [[0 for j in range(num_of_actions)] for i in range(num_of_states)]
        #self.policy = [random.randint(0,3) for i in range(num_of_states)]
        self.policy = [-1 for i in range(num_of_states)]
        
        self.exploration_rate = self.max_exploration_rate

    def reset(self, destination=None):
        self.planner.route_to(destination)
        # TODO: Prepare for a new trip; reset any variables here, if required

    def get_state_status(self, inputs, agent_state):
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
            #state_label = "-R-EW"
        elif heading[0] == 1 and heading[1] == 0:
            index_cab_view_local = index - 2
            if index_cab_view_local < 0:
                index_cab_view_local = 8 + index_cab_view_local
            #state_label = "-R-WE"
        elif heading[0] == 0 and heading[1] == 1:
            index_cab_view_local = index - 4
            if index_cab_view_local < 0:
                index_cab_view_local = 8 + index_cab_view_local
            #state_label = "-R-NS"
        elif heading[0] == 0 and heading[1] == -1:
            index_cab_view_local = index - 0
            if index_cab_view_local < 0:
                index_cab_view_local = 8 + index_cab_view_local
            #state_label = "-R-SN"
                    
        
        #print "LearningAgent.state_status(): up_dist = {}, down_dist = {}".format(up_dist, down_dist)  # [debug]
        #print "LearningAgent.state_status(): right_dist = {}, left_dist = {}".format(right_dist, left_dist)  # [debug]
        #print "LearningAgent.state_status(): vertical_dist = {}, horizontal_dist = {}".format(vertical_dist, horizontal_dist)  # [debug]
        #print "LearningAgent.state_status(): index = {}, index_cab_view_local = {}".format(index, index_cab_view_local)  # [debug]
        
        state_label = ""
        state_index = 0
        if index == -1:
            state_index = 0
            state_label = "-GOAL"
        else:
            if index_cab_view_local == 0:
                state_label = "Fo"              # Forward Only
            elif index_cab_view_local == 1:
                state_label = "Fo/Ri"           # Forward and Right
            elif index_cab_view_local == 2:
                state_label = "Ri"              # Right Only
            elif index_cab_view_local == 3:
                state_label = "Ri/Ba"           # Right and Back
            elif index_cab_view_local == 4:
                state_label = "Ba"              # Back Only
            elif index_cab_view_local == 5:
                state_label = "Ba/Le"           # Back and Left
            elif index_cab_view_local == 6:
                state_label = "Le"              # Left Only
            elif index_cab_view_local == 7:
                state_label = "Fo/Le"           # Forward and Left
                    
            temp_state_index = (index_cab_view_local*3)
            if inputs['light'] == 'red':                    # Cab is at a red traffic light
                state_index = temp_state_index + 1
                state_label = state_label + "-RED"
            else:
                if inputs['oncoming'] != None:              # Cab is at a green traffic light with oncoming vehicles in the opposite direction
                    state_index = temp_state_index + 2
                    state_label = state_label + "-GRE-O"
                else:                                       # Cab is at a green traffic light with no oncoming vehicles
                    state_index = temp_state_index + 3
                    state_label = state_label + "-GRE-N"
        
        return state_index, state_label

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
        
        if use_random_actions:
            # select a random action
            action = random.choice([None, 'forward', 'left', 'right'])
            
            # determine reward for performing action in the current state
            reward = self.env.act(self, action)
            
            print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, inputs, action, reward)  # [debug]
        else:
            #while(true) {
            # observe our current state
            state_index, state_label = self.get_state_status(inputs=inputs, agent_state=self.env.agent_states[self])
            self.state = state_label
            
            # select action from policy
            action_index = self.policy[state_index]
            
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
            
            # disable None action if light is green and there is no oncoming traffic
            #if ("-GRE-N" in state_label) and (action == None):
            #    action = random.choice(['left', 'forward', 'right'])
        
            # determine reward for performing action in the current state
            reward = self.env.act(self, action)
            
            # observe the next state that we will transition into based on the current state and action
            next_state_index, next_state_label = self.get_state_status(inputs=inputs, agent_state=self.env.agent_states[self])
            
            # select the maximum estimated Q value from the new state and all the possible actions
            max_q_hat = -1000000.
            for i in range(4):
                if self.q_hat[next_state_index][i] > max_q_hat:
                    max_q_hat = self.q_hat[next_state_index][i]
                    
            # update the estimate Q value for the current state and action based on the values of the next state and maximum Q value of possible actions
            new_q_hat = reward + (self.discount_rate * max_q_hat)
            self.q_hat[state_index][action_index] = ((1-self.learning_rate) * self.q_hat[state_index][action_index]) + (self.learning_rate * new_q_hat)
            #}
            
            # TODO: Learn policy based on state, action, reward
            # update policy with action with max estimated Q value in this particular state
            max_q_hat = -1000000.
            for i in range(len(self.q_hat[state_index])):
                if self.q_hat[state_index][i] > max_q_hat:
                    max_q_hat = self.q_hat[state_index][i]
                    self.policy[state_index] = i
                elif self.q_hat[state_index][i] == max_q_hat:
                    self.policy[state_index] = random.choice([i, self.policy[state_index]])
                    
                    
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
            
            if printDebug["state"]:
                print "LearningAgent.current_state(): state_index = {}, state_label = {}".format(state_index, state_label)  # [debug]
                for i in range(len(self.q_hat[state_index])):
                    print "LearningAgent.policy(): q_hat[curr_state_index][{}] = {}".format(i, self.q_hat[state_index][i])  # [debug]
                    
                print "LearningAgent.next_state(): next_state_index = {}, next_state_label = {}".format(next_state_index, next_state_label)  # [debug]
                for i in range(len(self.q_hat[next_state_index])):
                    print "LearningAgent.policy(): q_hat[next_state_index][{}] = {}".format(i, self.q_hat[next_state_index][i])  # [debug]
                
            if printDebug["policy"]:
                for i in range(len(self.policy)):
                    print "LearningAgent.policy(): policy[{}] = {}".format(i, self.policy[i])  # [debug]
                    
            if printDebug["exploration"]:
                for i in range(len(self.policy)):
                    print "LearningAgent.exploration(): exploration_rate = {}".format(self.exploration_rate)  # [debug]
        
        # pause sim if cab reaches its destination
        #location = self.env.agent_states[self]['location']
        #destination = self.env.agent_states[self]['destination']
        #if destination[0] == location[0] and destination[1] == location[1]:
            #self.sim.paused = True
            #self.sim.pause()
            
        
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
    sim = Simulator(e, update_delay=0.25, display=True)  # create simulator (uses pygame when display=True, if available)
    # NOTE: To speed up simulation, reduce update_delay and/or set display=False
    
    a.sim = sim

    sim.run(n_trials=200)  # run for a specified number of trials
    # NOTE: To quit midway, press Esc or close pygame window, or hit Ctrl+C on the command-line
    


if __name__ == '__main__':
    run()
