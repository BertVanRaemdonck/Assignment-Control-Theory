##from wachtrij_cudi import *
from math import cos, sin

def getU(time, speed):
    if time <= 17.83/speed:
        u = 0.07*speed
    else:
        u = 0
    return u


def getOmega(time, speed):
    if (2.0/speed <= time <= 4.25/speed) or \
       (5.11/speed <= time <= 7.35/speed) or \
       (9.35/speed <= time <= 11.59/speed) or \
       (12.74/speed <= time <= 14.98/speed) :

        omega = -0.7*speed
    else:
        omega = 0
    return omega


def getFlag(time, speed):
    if (time <= 1.99/speed) or (time >= 14.98/speed):
        flag = 1
    else:
        flag = 0
    return flag
    
def generateTrajectory(speed = 1):
    time = 0
    Ts = 0.01

    timeArr = [0]
    xArr = [0]
    yArr = [0]
    thetaArr = [0]
    uArr = [0]
    omegaArr = [0]
    flagArr = [0]

    while time <= 19.83/speed:
        
        u = getU(time, speed)
        omega = getOmega(time, speed)
        flag = getFlag(time, speed)
        x = xArr[-1]
        y = yArr[-1]
        theta = thetaArr[-1]
        time += Ts

        timeArr.append(time)
        xArr.append(x + Ts*u*cos(theta))
        yArr.append(y + Ts*u*sin(theta))
        thetaArr.append(theta + omega*Ts)
        uArr.append(u)
        omegaArr.append(omega)
        flagArr.append(flag)        

    return timeArr, xArr, yArr, thetaArr, uArr, omegaArr, flagArr

##def plotTrajectory(speed = 1):
##    x = generateTrajectory(2)[1]
##    y = generateTrajectory(2)[2]
##
##    root = Tk()
##    draw_canvas(root)
##    plt = Plot(x,y)
##    plt.draw()
##
##    root.mainloop()

def printTrajectory(speed = 1):
    tArr = generateTrajectory(speed)[0]
    xArr = generateTrajectory(speed)[1]
    yArr = generateTrajectory(speed)[2]
    aArr = generateTrajectory(speed)[3]
    vArr = generateTrajectory(speed)[4]
    wArr = generateTrajectory(speed)[5]
    fArr = generateTrajectory(speed)[6]
    
    print("time,X,Y,Theta,V,W,zero,zero,zero,DRflag,zero,zero,zero")
    
    for i in range(len(tArr)):
        t = tArr[i]
        x = xArr[i]
        y = yArr[i]
        a = aArr[i]
        v = vArr[i]
        w = wArr[i]
        f = fArr[i]
        
        print('{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t0\t0\t0\t{6}\t0\t0\t0'.format(t,x,y,a,v,w,f))
    


    
    
