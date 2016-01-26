import numpy as np  # Needed for sqrt
import logging
import math
import time


class Scara:
    # TowerOffset in meter
    X_toweroffset = 100
    Y_toweroffset = -90

    # linkage lenght
    Ltow = 135.0         # the base tower height;
    L1 = 140.0           # the upper arm length;
    L2 = 160.0           # the lower arm arm length;
    Ltool = 45.0         # distance between lower arm and tool tip



    @staticmethod
    def inverse_kinematics(X, Y, Z):
        """
        Inverse kinematics for SCARA bot. Returns angle in degrees
        for A, B, and C
        """
        # add tower offset
        coordReal = np.array([X, Y, Z])
        coord = np.array([X*1000 - Scara.X_toweroffset, Y*1000 + -Scara.Y_toweroffset, Z*1000])
        # http://www.deltatau.com/Common/technotes/SCARA%20Robot%20Kinematics.pdf

        # calculate y distance from tower to the target x,y
        dX = math.sqrt(coord[0]**2 + coord[1]**2) - Scara.Ltool

        # remove height of the tower from z
        dZ = coord[2] - Scara.Ltow

        # angle between L1 and L2
        A = math.acos((dX**2 + dZ**2 - Scara.L1**2 - Scara.L2**2)/(2*Scara.L1*Scara.L2))


         # angle between tower and L1
        b_1 = math.atan2(dX, dZ)

        b_2 = math.acos((dX**2 + dZ**2 + Scara.L1**2 - Scara.L2**2)/(2*Scara.L1*math.sqrt(dX**2 + dZ**2)))

        B = b_1 - b_2

        # rotational angle of tower z achsis
        # the tower rotation is inverted
        C = -math.atan(coord[0] / coord[1])

        ABC = np.array([math.degrees(A), math.degrees(B), math.degrees(C)])
        logging.debug("Inverse XYZ: %s" % coordReal)
        logging.debug("Inverse ABC: %s" % ABC)
        return ABC

    @staticmethod
    def forward_kinematics(A, B, C):
        """
        Forward kinematics for SCARA Bot. Returns the X, Y, Z point given
        angle translations
        """
        # log before the angles are converted to radians
        logging.debug("Forward ABC: %s" % np.array([A, B, C]))

        # convert angles in radians
        A = math.radians(A)
        B = math.radians(B)
        C = math.radians(C)

        # calculate distabce betwwon tool tip and tower
        distXY = (Scara.L1*math.sin(B)) + (Scara.L2*math.sin(A+B)) + Scara.Ltool;

        # calc x, y, c coords
        # x is inverted caused becaus angle C is inverted
        # all elements are converted from mm to m / 1000
        x = (round(distXY*math.sin(C-math.pi),2) + Scara.X_toweroffset)/1000;
        y = (round(distXY*math.cos(C),2) - -Scara.Y_toweroffset)/1000;
        z = (round(Scara.L1*math.cos(B) + Scara.L2*math.cos(A+B),2) + Scara.Ltow)/1000;


        logging.debug("Forward XYZ: %s" % np.array([x, y, z]))



        return np.array([x, y, z])


t= time.clock()

for i in range(1,20000):
    ABC = Scara.inverse_kinematics(0.1,0.14,0.1)
# print(t)
# print(Scara.forward_kinematics(t[0],t[1],t[2]))
# t = Scara.inverse_kinematics(192.4,177.22,0)
# print(t)
    XYZ=Scara.forward_kinematics(ABC[0],ABC[1],ABC[2])

print time.clock() - t
# t = Scara.inverse_kinematics(0,0,0)
# print(t)
# print(Scara.forward_kinematics(t[0],t[1],t[2]))
# print("\n")
# t = Scara.inverse_kinematics(100,200,0)
# print(t)
# print(Scara.forward_kinematics(t[0],t[1],t[2]))
# t = Scara.inverse_kinematics(192.4,177.22,0)
# print(t)
# print(Scara.forward_kinematics(t[0],t[1],t[2]))

#
# java implementation, as reference
#
#
# double mySq(double d) {
#   return Math.pow(d, 2);
# }
#
# void phys2virt() {
#
#   // add Tower offset
#   D0.set(cartesian.x - TowerOffset.x, cartesian.y + -TowerOffset.y, cartesian.z);
#
#   // http://www.deltatau.com/Common/technotes/SCARA%20Robot%20Kinematics.pdf
#   // calculate y distance from tower to the target x,y
#   double x = Math.sqrt(mySq(D0.x) + mySq(D0.y)) - L1;   // y is straight line between tower base and X,Y
#   double z = cartesian.z - L4;
#   double l1 = L3;
#   double l2 = L2;
#
#   A1 = Math.acos((mySq(x)+mySq(z)-mySq(l1)-mySq(l2))/(2*l1*l2));
#   //println("Angle1: ", degrees(A1));
#
#   double a2_1 = Math.atan2(x, z);
#   //println("Angle3: ", degrees(a3));
#
#   double a2_2 = Math.acos((mySq(x)+mySq(z)+mySq(l1)-mySq(l2))/(2*l1*Math.sqrt(mySq(x)+mySq(z))));
#   //println("Angle4: ", degrees(a4));
#
#   A2 = a2_1 - a2_2;
#
#   // rotate the tower to the target
#   if (D0.x == 0 && D0.y == 0) {
#     A3 = 0 -PI;
#   } else {
#     A3 = -atan(D0.x/D0.y);
#   }
#   //println("Angle3: ", degrees(A3));
# }
#
#   void virt2phys() {
#
#     double l1 = L3;
#     double l2 = L2;
#
#     double distXY = (l1*Math.sin(A2))+(l2*Math.sin(A2+A1)) + L1;
#
#     double x = Math.round(distXY*Math.sin(A3-PI)) + TowerOffset.x;
#
#     println("X: ", x);
#
#     double y = Math.round(distXY*Math.cos(A3)) - -TowerOffset.y;
#
#     println("Y: ", y);
#
#     double z = Math.round(l1*Math.cos(A2)) + Math.round(l2*Math.cos(A2+A1)) + L4;
#     println("Z: ", z);
#
#   }
# }
