"""
MFET 44200 - Lab 06

PID Controller 
"""

from simple_pid import PID

def PID_Controller(ke, ki, kd, InputReading, Output, MappedSetpoint):
    # Basic PID controller equation, takes in 3 data points and multiplies them by a constant
    # the error value is then returned for use in controlling the system

    # 'InputReading' - original, first order variable, defined as e in main function
    # 'Output' - the integral of the first order variable, defined as i in main function
    # 'MappedSetpoint' - the derivative of the first order variable, defined as d in main function
    # 'ke' - constant of the variable
    # 'ki' - constant of the integral
    # 'kd' - constant of the derivative

    ControllerPID = PID(InputReading, Output, MappedSetpoint)

    return ControllerPID


def main():
    i=1

    #ke ki and kd are constants, you do not want to change them after you find what values work with them
    ke = 2
    ki = 8
    kd = 0.125

    while i == 1: 
        
        #setpoint is going to be the starting position
        SetPoint = 0

        #Output is going to be the velocity value
        Output = 0

        #RawReading is going to be the voltage value
        RawReading = 0 

        e = map(RawReading, 0, 0, 0, 100)

        i = map(Output, 0, 0, 0, 100)

        d = map(SetPoint, 360, 0, 0, 100)

        PID_Controller(ke, ki, kd, e, i, d)

main()
