import math

def bot_1_goals(x_list,y_list):
    i=0
    while i<=2.094:
        x_goal=200*math.cos(i)
        y_goal=159*math.sin(4*i)
        x_list.append(x_goal)
        y_list.append(y_goal)
        # we have taken 20 ppoints here
        i+=0.1045
    return x_list,y_list

def bot_2_goals(x_list,y_list):
    i=2.094
    while i<=4.188:
        x_goal=200*math.cos(i)
        y_goal=159*math.sin(4*i)
        x_list.append(x_goal)
        y_list.append(y_goal)
        # we have taken 20 ppoints here
        i+=0.1045
    return x_list,y_list

def bot_3_goals(x_list,y_list):
    i=4.188
    while i<=6.28:
        x_goal=200*math.cos(i)
        y_goal=159*math.sin(4*i)
        x_list.append(x_goal)
        y_list.append(y_goal)
        # we have taken 20 ppoints here
        i+=0.1045
    return x_list,y_list

