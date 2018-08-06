import simple_rl.amdp.AbstractMRWorld.CustomMR as MRWorld 
import copy

def get_room_of_location(agent=[1,2]):
    global kb
    #print "Getting agents!"
    for i in kb.keys():
        #print i
        if len(kb[i]) == 7: #this is a labeled place, check if agent is inside of it
            c1x = kb[i][0]
            c1y = kb[i][1]
            c2x = kb[i][2]
            c2y = kb[i][3]

            #print agent
            #print i
            #print kb[i]

            if min(c1x,c2x) < agent[0] and max(c1x,c2x) > agent[0] and min(c1y,c2y) < agent[1] and max(c1y,c2y) > agent[1]: #Agent is inside this place, return that room name
                return i
    return 'AGENT NOT IN A ROOM!!!!'

#ALL POSES ARE WITH RESPECT TO ROS
amdp_file = open('label_places.txt','r')
amdp_pieces = amdp_file.read().split("\n")
global kb
kb = {} #knowledbe base about the semantic map
rooms = []
items = []
doors = []
for piece in amdp_pieces:
    split_piece = piece.split(" ")
    if len(split_piece) == 8: #labeled place, add to set of rooms
        place_name = split_piece[0]
        c1x = float(split_piece[1]) #corner 1 x
        c1y = float(split_piece[2]) #corner 1 y
        c2x = float(split_piece[3]) #corner 2 x
        c2y = float(split_piece[4]) #corner 2 y
        red_val = float(split_piece[5])
        green_val = float(split_piece[6])
        blue_val = float(split_piece[7])

        rooms.append(place_name)
        kb[place_name] = [c1x,c1y,c2x,c2y,red_val,green_val,blue_val]

    
    elif len(split_piece) == 4: #static location/item, add to items
        #print "items!"
        item_name = split_piece[0]
        ix = float(split_piece[1]) #item x position
        iy = float(split_piece[2]) #item y position
        it = float(split_piece[3]) #item rotation amount (how the robot should face at that location)

        items.append(item_name)
        kb[item_name] = [ix,iy,it]

    elif len(split_piece) == 6: #door way, add to doors
        #print "doors!"
        place1 = split_piece[0]
        place2 = split_piece[1]
        p1x = float(split_piece[2]) #x position of place 1 door
        p1y = float(split_piece[3]) #y position of place 1 door
        p2x = float(split_piece[4]) #x position of place 2 door
        p2y = float(split_piece[5]) #y position of place 2 door

        doors.append([place1,place2])
        kb[place1+"^"+place2] = [p1x,p1y,p2x,p2y] #doorways are represented as set of places, key is place1^place2


ACTIONS = ['to'+room for room in rooms] + ['turnOff'+light for light in items]

#print rooms
#print items
#print doors
#print ACTIONS

#Needs to be specified
#starting_items={items[0]:[rooms[2],1],items[1]:[rooms[3],1]} #locations are specified by items locations, states are specified by MR
starting_items = {}
for item in items:
    starting_items[item] = [get_room_of_location(kb[item][:2]),1]
print starting_items
    

goal_items = copy.deepcopy(starting_items) #specified by MR. For now, we will assume that every item should be 'turned off'
#goal_items[items[0]][1] = 0 #Specified by MR
#goal_items[items[1]][1] = 0 #Specified by MR
for item in goal_items.keys():
    goal_items[item][1] = 0

#start_room = rooms[0] #extracted from robot location
start_room = get_room_of_location([3,-1.6]) #extracted from robot location

#goal_room = rooms[1] #Specified by MR. We will currently set it back to the initial room
goal_room = start_room #Specified by MR. We will currently set it back to the initial room
goal_type = 'both' #Specified by MR. For now we will assume goal_type stays both.

(action_seq,state_seq,policy) = MRWorld.get_l1_action_state_policy_seq(start_room=start_room, goal_room=goal_room, starting_items=starting_items, goal_items=goal_items,actions=ACTIONS, doors=doors,rooms=rooms,goal_type=goal_type)

print action_seq
print state_seq
print policy
