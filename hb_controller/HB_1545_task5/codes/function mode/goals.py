import math

x_list=[]
y_list=[]
def bot_1_goals(x_list,y_list):
    i=0
    n=90
    while i<=2*math.pi/3:
        
        # (220cos(4θ))cos(θ)
        
        x_goal=250+(220*math.cos(4*i))*math.cos(i)
        y_goal=250+(220*math.cos(4*i))*math.sin(i)
        x_list.append(x_goal)
        y_list.append(y_goal)
        # we have taken 20 ppoints here
        i+=2*math.pi/(3*n)
        
    return x_list,y_list

def bot_2_goals(x_list,y_list):
    n=90
    i=2*math.pi/3 
    
    while i<=4*math.pi/3 :
        x_goal=250+(220*math.cos(4*i))*math.cos(i)
        y_goal=250+(220*math.cos(4*i))*math.sin(i)
        x_list.append(x_goal)
        y_list.append(y_goal)
        # we have taken 20 ppoints here
        i+=2*math.pi/(3*n)
    
    return x_list,y_list

def bot_3_goals(x_list,y_list):
    i=4*math.pi/3
    n=90
    while i<=2*math.pi:
        x_goal=250+(220*math.cos(4*i))*math.cos(i)
        y_goal=250+(220*math.cos(4*i))*math.sin(i)
        x_list.append(x_goal)
        y_list.append(y_goal)
        # we have taken 20 ppoints here
        i+=2*math.pi/(3*n)
    return x_list,y_list


bot_3_goals(x_list,y_list)
print(len(y_list))