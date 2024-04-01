"""
MFET 44200 - Lab 06
Zachary Wilson

PID Controller + utilities
"""

def PID_controller(ke, ki, kd, e, i, d):
    # Basic PID controller equation, takes in 3 data points and multiplies them by a constant
    # the error value is then returned for use in controlling the system

    # 'e' - original, first order variable
    # 'i' - the integral of the first order variable
    # 'd' - the derivative of the first order variable
    # 'ke' - constant of the variable
    # 'ki' - constant of the integral
    # 'kd' - constant of the derivative

    error = (ke * e) + (ki * i) + (kd * d)
    return error



def fake_derive(x, dx, dt):
    # takes in two data points, and the time between them, and returns the slope

    # 'x' - first data point
    # 'dx' - second data point
    # 'dt' - delta time
    
    return (dx-x) / dt



def fake_integrate(x, dx, dt):
    # takes in two data points, and the time between them, and returns the area underneath
    
    # 'x' - first data point
    # 'dx' - second data point
    # 'dt' - delta time
    
    return ((dx -x) * dt) / 2


"""
def calculate_steering_angle(theta, expected, vel):



    return angle
"""