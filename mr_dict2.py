from simple_rl.mdp.StateClass import State
from simple_rl.mdp.MDPClass import MDP
from simple_rl.planning import ValueIteration
from simple_rl.amdp.AMDPTaskNodesClass import NonPrimitiveAbstractTask, RootTaskNode
import copy

from collections import defaultdict
import re

class FourRoomL1State(State):
    def __init__(self, room_number, is_terminal=False, items=[]):
        State.__init__(self, data=[room_number], is_terminal=is_terminal)
        self.agent_in_room_number = room_number
        self.items = items

    def __hash__(self):
        return hash(tuple(self.data))

    def __str__(self):
        return 'Agent in room {}'.format(self.agent_in_room_number)

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        return isinstance(other, FourRoomL1State) and self.agent_in_room_number == other.agent_in_room_number and self.items == other.items

class FourRoomL1GroundedAction(NonPrimitiveAbstractTask):
    def __init__(self, l1_action_string, subtasks, lowerDomain):
        self.action = l1_action_string
        goal_room = self.extract_goal_room(self.action)
        self.goal_state = FourRoomL1State(goal_room, is_terminal=True)
        tf, rf = self._terminal_function, self._reward_function
        self.l0_domain = lowerDomain
        NonPrimitiveAbstractTask.__init__(self, l1_action_string, subtasks, tf, rf)

    @classmethod
    def extract_goal_room(cls, action):
        room_numbers = re.findall(r'\d+', action)
        if len(room_numbers) == 0:
            raise ValueError('unable to extract room number from L1Action {}'.format(action))
        return int(room_numbers[0])

    def _terminal_function(self, state):
        if type(state) == FourRoomL1State:
            return state == self.goal_state
        room_number = self._room_number(state)
        assert type(room_number) == int, 'Room number {} not convertible to int'.format(room_number)
        return FourRoomL1State(room_number) == self.goal_state

    def _reward_function(self, state):
        if type(state) == FourRoomL1State:
            return 1. if state == self.goal_state else 0.
        room_number = self._room_number(state)
        assert type(room_number) == int, 'Room number {} not convertible to int'.format(room_number)
        return 1. if FourRoomL1State(room_number) == self.goal_state else 0.

    def _room_number(self, state):
        return self.l0_domain.get_room_numbers((int(state.x), int(state.y)))[0]

class FourRoomRootGroundedAction(RootTaskNode):
    def __init__(self, action_str, subtasks, l1_domain, terminal_func, reward_func):
        self.action = action_str
        self.goal_state = FourRoomL1State(FourRoomL1GroundedAction.extract_goal_room(action_str), is_terminal=True)

        RootTaskNode.__init__(self, action_str, subtasks, l1_domain, terminal_func, reward_func)

class FourRoomL1MDP(MDP):
    def __init__(self, starting_room=1, goal_room=4, starting_items=[], goal_items=[] ,gamma=0.99,actions=None,doors=[],rooms=[]):
        initial_state = FourRoomL1State(starting_room,items=starting_items)
        self.goal_state = FourRoomL1State(goal_room, is_terminal=True, items=goal_items)
        self.terminal_func = lambda state: state == self.goal_state
        self.doors=doors
        self.rooms=rooms
        

        MDP.__init__(self,actions, self._transition_func, self._reward_func, init_state=initial_state,
                     gamma=gamma)

    def _reward_func(self, state, action):
        if self._is_goal_state_action(state, action):
            return 1.0
        return 0.0

    def _is_goal_state_action(self, state, action):
        if state == self.goal_state:
            return False

        return self._transition_func(state, action) == self.goal_state

    def _transition_func(self, state, action):
        if state.is_terminal():
            return state

        current_room = state.agent_in_room_number
        next_state = None
        
        if current_room == 'Room1':
            if action == 'toRoom2':
                next_state = FourRoomL1State(action[2:],items=state.items)
            if action == 'toRoom3':
                next_state = FourRoomL1State(action[2:],items=state.items)
        if current_room == 'Room2':
            if action == 'toRoom1':
                next_state = FourRoomL1State(action[2:],items=state.items)
            #if action == 'toRoom4':
            #    next_state = FourRoomL1State(action[2:],items=state.items)
            if action == 'turnOnL1':
                next_items = copy.deepcopy(state.items)
                next_items['l1'][1] = 1
                next_state = FourRoomL1State(current_room,items=next_items)
        if current_room == 'Room3':
            if action == 'toRoom4':
                next_state = FourRoomL1State(action[2:],items=state.items)
            if action == 'toRoom1':
                next_state = FourRoomL1State(action[2:],items=state.items)
            if action == 'turnOnL2':
                next_items = copy.deepcopy(state.items)
                next_items['l2'][1] = 1
                next_state = FourRoomL1State(current_room,items=next_items)
        if current_room == 'Room4':
            if action == 'toRoom2':
                next_state = FourRoomL1State(action[2:],items=state.items)
            if action == 'toRoom3':
                next_state = FourRoomL1State(action[2:],items=state.items)
            if action == 'toRoom5':
                next_state = FourRoomL1State(action[2:],items=state.items)
        if current_room == 'Room5':
            if action == 'toRoom4':
                next_state = FourRoomL1State(action[2:],items=state.items)

        if next_state is None:
            next_state = state

        if next_state == self.goal_state:
            next_state.set_terminal(True)

        return next_state

    def __str__(self):
        return 'AbstractFourRoomMDP: InitState: {}, GoalState: {}'.format(self.init_state, self.goal_state)

    @classmethod
    def action_for_room_number(cls, room_number):
        for action in cls.ACTIONS:
            if str(room_number) in action:
                return action
        raise ValueError('unable to find action corresponding to room {}'.format(room_number))

def get_l1_policy(start_room=None, goal_room=None, mdp=None, starting_items=None, goal_items=None,actions=None,doors=None,rooms=None):
    if mdp is None:
        mdp = FourRoomL1MDP(start_room, goal_room,starting_items=starting_items,goal_items=goal_items,actions=actions,doors=doors,rooms=rooms)
    vi = ValueIteration(mdp)
    vi.run_vi()

    policy = defaultdict()
    action_seq, state_seq = vi.plan(mdp.init_state)

    print 'Plan for {}:'.format(mdp)
    for i in range(len(action_seq)):
        print "\tpi[{}] -> {}".format(state_seq[i], action_seq[i])
        policy[state_seq[i]] = action_seq[i]
    return policy

if __name__ == '__main__':
    rooms = ['Room1','Room2','Room3','Room4','Room5']
    lights = ['L1','L2']
    doors = [[rooms[0],rooms[1]],[rooms[1],rooms[2]],[rooms[2],rooms[3]],[rooms[2],rooms[4]]]
    ACTIONS = ['to'+room for room in rooms] + ['turnOn'+light for light in lights]
    #ACTIONS = ['toRoom1', 'toRoom2', 'toRoom3', 'toRoom4', 'toRoom5','turnOnL1','turnOnL2']
    print ACTIONS
    policy = get_l1_policy(rooms[0], rooms[1], starting_items={'l1':[rooms[0],0],'l2':[rooms[1],0]}, goal_items={'l1':[rooms[0],1], 'l2':[rooms[1],1]},actions=ACTIONS, doors=doors,rooms=rooms)
