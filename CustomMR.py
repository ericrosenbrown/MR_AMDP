from simple_rl.mdp.StateClass import State
from simple_rl.mdp.MDPClass import MDP
from simple_rl.planning import ValueIteration
from simple_rl.amdp.AMDPTaskNodesClass import NonPrimitiveAbstractTask, RootTaskNode
import copy

from collections import defaultdict
import re

class FourRoomL1State(State):
    def __init__(self, room_number, is_terminal=False, items=[],goal_type=None):
        State.__init__(self, data=[room_number], is_terminal=is_terminal)
        self.agent_in_room_number = room_number
        self.items = items
        self.goal_type = goal_type

    def __hash__(self):
        return hash(tuple(self.data))

    def __str__(self):
        return 'Agent in room {}'.format(self.agent_in_room_number)

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if self.goal_type == 'items':
            return self.items == other.items and isinstance(other, FourRoomL1State)
        elif self.goal_type == 'rooms':
            return self.agent_in_room_number == other.agent_in_room_number and isinstance(other, FourRoomL1State)
        elif self.goal_type == 'both':
            return isinstance(other, FourRoomL1State) and self.agent_in_room_number == other.agent_in_room_number and self.items == other.items
        else:
            return 'goal type in fourooml1state no set correctly'
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
        return FourRoomL1State(room_number,goal_type='blah') == self.goal_state

    def _reward_function(self, state):
        if type(state) == FourRoomL1State:
            return 1. if state == self.goal_state else 0.
        room_number = self._room_number(state)
        assert type(room_number) == int, 'Room number {} not convertible to int'.format(room_number)
        return 1. if FourRoomL1State(room_number,goal_type='blahw') == self.goal_state else 0.

    def _room_number(self, state):
        return self.l0_domain.get_room_numbers((int(state.x), int(state.y)))[0]

class FourRoomRootGroundedAction(RootTaskNode):
    def __init__(self, action_str, subtasks, l1_domain, terminal_func, reward_func):
        self.action = action_str
        self.goal_state = FourRoomL1State(FourRoomL1GroundedAction.extract_goal_room(action_str), is_terminal=True, goal_type='blah')

        RootTaskNode.__init__(self, action_str, subtasks, l1_domain, terminal_func, reward_func)

class FourRoomL1MDP(MDP):
    def __init__(self, starting_room=1, goal_room=4, starting_items=[], goal_items=[] ,gamma=0.99,actions=None,doors=[],rooms=[],goal_type=None):
        initial_state = FourRoomL1State(starting_room,items=starting_items,goal_type=goal_type)
        self.goal_state = FourRoomL1State(goal_room, is_terminal=True, items=goal_items, goal_type=goal_type)
        print goal_type
        if goal_type == 'items':
            print "setting terminal func to items"
            self.terminal_func = lambda state: state.items == self.goal_state.items
        elif goal_type == 'rooms':
            self.terminal_func = lambda state: state.agent_in_room_number == self.goal_state.agent_in_room_number
        elif goal_type == 'both':
            self.terminal_func = lambda state: state == self.goal_state
        else:
            self.terminal_func = 'messed up goal_type'
        self.doors=doors
        self.rooms=rooms
        self.goal_type = goal_type
        

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

        #if action is toRoomX, let it move if there is a door way
        if action[:2] == 'to':
            for path in self.doors:
                if set([action[2:],current_room]) == set(path): #there is a pathway between current room and desired room to go to, set next state to that room        
                    next_state = FourRoomL1State(action[2:],items=state.items,goal_type=self.goal_type)
        if action[:7] == 'turnOff': #if action is turn off, let it turn off the lights if there are in that room
            if state.items[action[7:]][0] == current_room: #The light you want to turn off is in your room
                next_items = copy.deepcopy(state.items)
                next_items[action[7:]][1] = 0
                next_state = FourRoomL1State(current_room,items=next_items,goal_type=self.goal_type)

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

def get_l1_policy(start_room=None, goal_room=None, mdp=None, starting_items=None, goal_items=None,actions=None,doors=None,rooms=None,goal_type=None):
    if mdp is None:
        mdp = FourRoomL1MDP(start_room, goal_room,starting_items=starting_items,goal_items=goal_items,actions=actions,doors=doors,rooms=rooms,goal_type=goal_type)
    vi = ValueIteration(mdp)
    vi.run_vi()

    policy = defaultdict()
    action_seq, state_seq = vi.plan(mdp.init_state)

    print 'Plan for {}:'.format(mdp)
    for i in range(len(action_seq)):
        print "\tpi[{}] -> {}".format(state_seq[i], action_seq[i])
        policy[state_seq[i]] = action_seq[i]
    return policy


def get_l1_action_state_policy_seq(start_room=None, goal_room=None, mdp=None, starting_items=None, goal_items=None,actions=None,doors=None,rooms=None,goal_type=None):
    if mdp is None:
        mdp = FourRoomL1MDP(start_room, goal_room,starting_items=starting_items,goal_items=goal_items,actions=actions,doors=doors,rooms=rooms,goal_type=goal_type)
    vi = ValueIteration(mdp)
    vi.run_vi()

    policy = defaultdict()
    action_seq, state_seq = vi.plan(mdp.init_state)

    print 'Plan for {}:'.format(mdp)
    for i in range(len(action_seq)):
        print "\tpi[{}] -> {}".format(state_seq[i], action_seq[i])
        policy[state_seq[i]] = action_seq[i]
    return (action_seq,state_seq,policy)

if __name__ == '__main__':
    rooms = ['Room0','Room1','Room2','Room3','Room4','Room5']
    lights = ['l1','l2']
    doors = [[rooms[0],rooms[1]],[rooms[1],rooms[2]],[rooms[2],rooms[3]],[rooms[1],rooms[4]],[rooms[1],rooms[5]]]
    ACTIONS = ['to'+room for room in rooms] + ['turnOff'+light for light in lights]
    starting_items={lights[0]:[rooms[4],0],lights[1]:[rooms[5],0]}
    goal_items = copy.deepcopy(starting_items)
    goal_items[lights[0]][1] = 1
    goal_items[lights[1]][1] = 1
    print ACTIONS
    start_room = rooms[0]
    goal_room = rooms[3]
    goal_type = 'both'
    policy = get_l1_policy(start_room=start_room, goal_room=goal_room, starting_items=starting_items, goal_items=goal_items,actions=ACTIONS, doors=doors,rooms=rooms,goal_type=goal_type)
