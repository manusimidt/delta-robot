#https://hypertriangle.com/~alex/delta-robot-tutorial/
from math import sqrt, sin, tan, atan, cos

f: float = 241
e: float = 181
rf: float = 120
re: float = 205


sqrt3: float = sqrt(3.0)
pi: float = 3.141592653
sin120: float = sqrt3/2.0
cos120: float = -0.5
tan60: float = sqrt3
sin30: float = 0.5
tan30: float = 1/sqrt3

# forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
# returned status: 0=OK, -1=non-existing position


def delta_calcForward(theta1: float, theta2: float, theta3: float) -> tuple:
    t: float = (f-e)*tan30/2
    dtr: float = pi/180.0

    theta1 *= dtr
    theta2 *= dtr
    theta3 *= dtr

    y1: float = -(t + rf*cos(theta1))
    z1: float = -rf*sin(theta1)

    y2: float = (t + rf*cos(theta2))*sin30
    x2: float = y2*tan60
    z2: float = -rf*sin(theta2)

    y3: float = (t + rf*cos(theta3))*sin30
    x3: float = -y3*tan60
    z3: float = -rf*sin(theta3)

    dnm: float = (y2-y1)*x3-(y3-y1)*x2

    w1: float = y1*y1 + z1*z1
    w2: float = x2*x2 + y2*y2 + z2*z2
    w3: float = x3*x3 + y3*y3 + z3*z3

    #x = (a1*z + b1)/dnm
    a1: float = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1)
    b1: float = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0

    # y = (a2*z + b2)/dnm;
    a2: float = -(z2-z1)*x3+(z3-z1)*x2
    b2: float = ((w2-w1)*x3 - (w3-w1)*x2)/2.0

    # a*z^2 + b*z + c = 0
    a: float = a1*a1 + a2*a2 + dnm*dnm
    b: float = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm)
    c: float = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re)

    # discriminant
    d: float = b*b - 4.0*a*c
    if (d < 0):
        return -1, (0, 0, 0)  # non-existing point

    z0 = -0.5*(b+sqrt(d))/a
    x0 = (a1*z0 + b1)/dnm
    y0 = (a2*z0 + b2)/dnm
    return 0, (x0, y0, z0)


# inverse kinematics
# helper functions, calculates angle theta1 (for YZ-pane)
def delta_calcAngleYZ(x0: float, y0: float, z0: float):
    y1 = -0.5 * 0.57735 * f  # f/2 * tg 30
    y0 -= 0.5 * 0.57735 * e  # shift center to edge
    # z = a + b*y
    a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1)/(2*z0)
    b = (y1-y0)/z0
    # discriminant
    d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf)
    if (d < 0):
        return -1, 0  # non-existing point
    yj = (y1 - a*b - sqrt(d))/(b*b + 1)  # choosing outer point
    zj = a + b*yj
    theta = 180.0*atan(-zj/(y1 - yj))/pi + (180.0 if (yj > y1) else 0.0)
    return 0, theta


# inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
# returned status: 0=OK, -1=non-existing position
def delta_calcInverse(x0: float,  y0: float, z0: float):
    theta1 = theta2 = theta3 = 0
    status, theta1 = delta_calcAngleYZ(x0, y0, z0)
    if (status == 0):
        status, theta2 = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0)  # rotate coords to +120 deg
    if (status == 0):
        status, theta3 = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0)  # rotate coords to -120 deg
    return status, (theta1, theta2, theta3)


if __name__ == '__main__':
    print(delta_calcInverse(0,0,-250))
    print(delta_calcForward(39,39,39))