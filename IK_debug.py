from sympy import *
from time import time
from mpmath import radians
import tf
import numpy as np

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    # print(test_case[1][0])
    # Wc = Matrix(test_case[1])
    # Ee = Matrix(test_case[0][0])
    # print('wc-ee:' , sqrt((Wc-Ee).dot(Wc-Ee)))

    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ##
    # Create symbols
    a, alpha, q, d = symbols('a, alpha, q, d')
    q1, q2, q3, q4, q5, q6 = symbols('q1:7')

    # Create Modified DH parameters
    s = []
    s.append({alpha: 0,  	a: 0, 		d: 0.75,	q: q1 		}) #T1
    s.append({alpha: -pi/2, a: 0.35,	d: 0,		q: q2-pi/2 	}) #T2
    s.append({alpha: 0,  	a: 1.25, 	d: 0,		q: q3 		}) #T3
    s.append({alpha: -pi/2,	a: -0.054, 	d: 1.5,		q: q4 		}) #T4
    s.append({alpha: pi/2, 	a: 0, 		d: 0,		q: q5 		}) #T5
    s.append({alpha: -pi/2,	a: 0, 		d: 0,		q: q6 		}) #T6
    s.append({alpha: 0,	    a: 0,		d: 0.303,	q: 0		}) #Tg

    # Define Modified DH Transformation matrix
    T = Matrix([	[            cos(q),           -sin(q),           0,              a],
                    [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha),  -sin(alpha)*d],
                    [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),   cos(alpha)*d],
                    [                 0,                 0,           0,              1]])

    # Create individual transformation matrices, substitute constant parameters
    TList = 7*[T[:,:]]
    for i in range(0,len(TList)): TList[i] = TList[i].subs(s[i])

    # Extract rotation matrices from the transformation matrices
    # RotMat = []
    # for i in range(0,len(TList)): RotMat.append(TList[i][0:3,0:3])

    # completete modified D-H transformation
    T0_G = TList[0][:,:]
    for i in range(1,len(TList)): T0_G = T0_G*TList[i]
    T0_G = simplify(T0_G)


    # Define basic rotations
    rad = symbols('rad')
    RotX = Matrix([ [1,         0,          0,              0],
                    [0,         cos(rad),   -sin(rad),      0],
                    [0,         sin(rad),   cos(rad),       0],
                    [0,         0,          0,              1]])

    RotY = Matrix([	[cos(rad),	    0,		sin(rad),	    0],
                    [0,		        1,		0,		        0],
                    [-sin(rad),	    0,		cos(rad),	    0],
                    [0,		        0,		0,		        1]])

    RotZ = Matrix([	  [cos(rad),	   -sin(rad),	0, 	0],
                      [sin(rad),	   cos(rad),	0,	0],
                      [0,		       0,		    1,	0],
                      [0,		       0,		    0,	1]   ])

    ### Transform from gripper link in ROS to modified DH notation
    R_corr = RotZ.subs(rad, np.pi) * RotY.subs(rad, -np.pi/2)
    T_ROS = simplify(T0_G*R_corr)

    ### Transform from end effector pose from gazebo to base link orientation


    ## IK code here! ######################################################################################################################
    #######################################################################################################################################

    end_pose = Matrix([position.x, position.y, position.z])
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])


    ## Calculate transform for end effectr orientation relative to base link
    Rrpy_gaz = (RotZ.subs(rad, yaw) * RotY.subs(rad, pitch) * RotX.subs(rad, roll)).evalf() # gazebo
    Rrpy = ( Rrpy_gaz * R_corr).evalf() # modified DH noatation


    ## Calculate writst center position by moving 0.303 units along x axis of gazebo's gripper link
    L_xyz = Rrpy_gaz[0:3,0]
    R6_to_Rg = 0.303
    Wrc = simplify(end_pose - L_xyz*R6_to_Rg) #wrist center
    Wrc2 = Matrix([ sqrt(Wrc[0]**2+Wrc[1]**2)-0.35, 0,  Wrc[2]-0.75]) #wrist center with respect to joint 2


    # Calculate joints 1-3 values
    theta1 = atan2(Wrc[1], Wrc[0])

    lenA = sqrt(1.5**2 + 0.054**2)
    lenB = sqrt(Wrc2[0]**2 + Wrc2[2]**2)
    lenC = 1.25

    alph = acos( (lenB**2 + lenC**2 - lenA**2) / (2*lenB*lenC))
    beta = acos( (lenA**2 + lenC**2 - lenB**2) / (2*lenA*lenC))
    epsi = atan(Wrc2[2]/Wrc2[0])
    delt = atan(0.054/1.5)

    theta2 = pi/2-epsi-alph
    theta3 = pi/2 - beta - delt

    ########## Calculate joionts 4-6
    R0_3 = (TList[0]*TList[1]*TList[2]).subs({q1:theta1, q2:theta2, q3:theta3}).evalf()
    R3_6 = R0_3.inv('LU') * Rrpy
    R3_6DH = simplify(TList[3]*TList[4]*TList[5]*TList[6])
    #print(R3_6DH)

    # theta5 = acos(R3_6[1,2])
    # theta4 = asin(R3_6[2,2]/sin(theta5))
    # theta6 = acos(R3_6[1,0]/sin(theta5))
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])





    ##
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    vect =  Matrix([0,0,0,1])
    #vectBase = (T_ROS*vect).subs({q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6}).evalf()
    vectBase = (T_ROS*vect).subs({q1:-0, q2:0, q3:0, q4:0, q5:0, q6:0}).evalf()
    #vectBase = (T_ROS*vect).subs({q1:-0.65, q2:0.45, q3:-0.36, q4:0.95, q5:0.79, q6:0.49}).evalf()
    #print(vectBase)
    #print(((TList[0]*TList[1]*TList[2]*TList[3])*vect).subs({q1:-0.65, q2:0.45, q3:-0.36, q4:0.95, q5:0.79, q6:0.49}).evalf())
    print(((TList[0]*TList[1]*TList[2]*TList[3])*vect).subs({q1:theta1, q2:theta2, q3:theta3, q4:0, q5:0, q6:0}).evalf())
    #print(T_ROS.subs({q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}).evalf())
    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = Wrc # <--- Load your calculated WC values in this array
    your_ee = [vectBase[0],vectBase[1],vectBase[2]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==2):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
