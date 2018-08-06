import simple_rl.amdp.AbstractMRWorld.CustomMR as MRWorld 
import copy


#ALL POSES ARE WITH RESPECT TO ROS
amdp_file = open('label_places.txt','r')
amdp_pieces = amdp_file.read().split("\n")
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

    
    elif len(split_piece) == 4: #static location/item, add to items
        print "items!"
        item_name = split_piece[0]
        ix = float(split_piece[1]) #item x position
        iy = float(split_piece[2]) #item y position
        it = float(split_piece[3]) #item rotation amount (how the robot should face at that location)

        items.append(item_name)

    elif len(split_piece) == 6: #door way, add to doors
        print "doors!"
        place1 = split_piece[0]
        place2 = split_piece[1]
        p1x = float(split_piece[2]) #x position of place 1 door
        p1y = float(split_piece[3]) #y position of place 1 door
        p2x = float(split_piece[4]) #x position of place 2 door
        p2y = float(split_piece[5]) #y position of place 2 door

        doors.append([place1,place2])


ACTIONS = ['to'+room for room in rooms] + ['turnOff'+light for light in items]

print rooms
print items
print doors
print ACTIONS


starting_items={items[0]:[rooms[2],1],items[1]:[rooms[3],1]}
goal_items = copy.deepcopy(starting_items)
goal_items[items[0]][1] = 0
goal_items[items[1]][1] = 0
start_room = rooms[0]
goal_room = rooms[1]
goal_type = 'both'

policy = MRWorld.get_l1_policy(start_room=start_room, goal_room=goal_room, starting_items=starting_items, goal_items=goal_items,actions=ACTIONS, doors=doors,rooms=rooms,goal_type=goal_type)
