from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix
import numpy as np

### Create symbols for joint variables
a, alpha, q, d = symbols('a, alpha, q, d')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')


# DH Parameters
s = []
s.append({alpha: 0,  	a: 0, 		d: 0.75,	q: q1 		}) #T1
s.append({alpha: -pi/2, a: 0.35,	d: 0,		q: q2-pi/2 	}) #T2
s.append({alpha: 0,  	a: 1.25, 	d: 0,		q: q3 		}) #T3
s.append({alpha: -pi/2,	a: -0.054, 	d: 1.5,		q: q4 		}) #T4
s.append({alpha: pi/2, 	a: 0, 		d: 0,		q: q5 		}) #T5
s.append({alpha: -pi/2,	a: 0, 		d: 0,		q: q6 		}) #T6
s.append({alpha: 0,	a: 0,		d: 0.303,	q: 0		}) #Tg

# DH Matrix for single transform
T = 	Matrix([[            cos(q),            -sin(q),           0,              a],
               [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
               [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
               [                  0,                  0,           0,              1]])

# Substitute constant dh parameters to transform matrices T0 - TG
TList = 7*[T[:,:]]
for i in range(0,len(TList)): TList[i] = TList[i].subs(s[i])

# Create transform matrix for transitin from base link to gripper T0_G
T0_G = Matrix( [[1,0,0,0],
		[0,1,0,0],
		[0,0,1,0],
		[0,0,0,1]])

for i in range(0,len(TList)): T0_G = T0_G*TList[i]

# Extract rotation matrces
RotMat = []
for i in range(0,len(TList)): RotMat.append(TList[i][0:3,0:3])

### Transform from gripper link in ROS to modified DH notation
R_z = Matrix([	[cos(np.pi),	-sin(np.pi),	0, 	0],
		[sin(np.pi),	cos(np.pi),	0,	0],
		[0,		0,		1,	0],
		[0,		0,		0,	1]])

R_y = Matrix([	[cos(np.pi),	0,		-sin(np.pi),	0],
		[0,		1,		0,		0],
		[sin(np.pi),	0,		cos(np.pi),	0],
		[0,		0,		0,		1]])
R_corr = simplify(R_z*R_y)
T_ROS = T0_G*R_corr


### Transform gripper position to base link coordinates
# Define angle of rotations for all joints
#s_q = {q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}
#s_q = {q1:-1.87,q2:0.86,q3:-1.77,q4:1.98,q5:0.22,q6:-4.84}
s_q = {q1:-2.98,q2:0.12,q3:-3.01,q4:-3.4,q5:-0.37,q6:-5.05}

# Crate vector representing gripper position in gripper link coordinates
vec = Matrix([0,0,0,1])

# Transform gripper link position to base link coordinates
vec_base = T_ROS*vec

# Substitute joint angles
vec_base = vec_base.subs(s_q)
print(vec_base.evalf())






