import math

def bot_1_goals(x_list,y_list):
    i=0
    while i<=2*math.pi/3:
        x_goal=250+200*math.cos(i)
        y_goal=250+150*math.sin(4*i)
        x_list.append(x_goal)
        y_list.append(y_goal)
        # we have taken 20 ppoints here
        i+=math.pi/90
    return x_list,y_list

def bot_3_goals(x_list,y_list):
    i=2*math.pi/3
    while i<=4*math.pi/3:
        x_goal=250+200*math.cos(i)
        y_goal=250+150*math.sin(4*i)
        x_list.append(x_goal)
        y_list.append(y_goal)
        # we have taken 20 ppoints here
        i+=math.pi/60
    return x_list,y_list

def bot_2_goals(x_list,y_list):
    i=4*math.pi/3
    while i<=2*math.pi:
        x_goal=250+200*math.cos(i)
        y_goal=250+150*math.sin(4*i)
        x_list.append(x_goal)
        y_list.append(y_goal)
        # we have taken 20 ppoints here
        i+=math.pi/60
    return x_list,y_list

