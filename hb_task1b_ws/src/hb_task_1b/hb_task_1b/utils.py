def getPose(response):
    x_goal = response.x_goal
    y_goal = response.y_goal
    theta_goal = response.theta_goal
    flag = response.end_of_list

    return x_goal, y_goal, theta_goal, flag

def stop():
    pass