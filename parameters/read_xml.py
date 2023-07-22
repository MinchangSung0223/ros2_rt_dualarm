import xml.etree.ElementTree as ET
import numpy as np
np.set_printoptions(suppress=True)


class MDHParameters:
    def __init__(self, alpha, a, d, theta):
        self.alpha = alpha
        self.a = a
        self.d = d
        self.theta = theta

def TransInv(T):
    # Extract the rotation matrix R and the position vector p from the transformation matrix T
    R = T[0:3, 0:3]
    p = T[0:3, 3]

    # Compute the inverse transformation matrix
    invT = np.eye(4)
    invT[0:3,0:3] = R.T;
    val = -R.T@p;
    invT[0,3] = val[0];
    invT[1,3] = val[1];
    invT[2,3] = val[2];        
    

    return invT
def JacobianBodyMDH(MDH, thetalist):
    T = np.eye(4)
    N = len(thetalist)
    Jb = np.zeros((6, N))

    for i in range(0,N):
        Si = np.zeros((N, 1))
        Si[i] = 1
        Ei = np.array([[0, 0, 0, 0, 0, 1]]).T
        T_ = RotX(MDH.alpha[i]) @ TransX(MDH.a[i]) @ TransZ(MDH.d[i]) @ RotZ(thetalist[i] + MDH.theta[i])
        AdInv = Adjoint(TransInv(T_))
        Jb = AdInv @ Jb + Ei @ Si.T
        T = T @ T_
    return Jb,T
def VecToso3(omg):
    # Build the 3x3 skew-symmetric matrix in so(3)
    so3mat = np.array([[0, -omg[2], omg[1]],
                       [omg[2], 0, -omg[0]],
                       [-omg[1], omg[0], 0]])
    
    return so3mat
def Adjoint(T):
    # Extract the rotation matrix R and the position vector p from the transformation matrix T
    R = T[:3, :3]
    p = T[:3, 3]

    # Build the 6x6 adjoint representation [AdT]
    AdT = np.eye(6)
    AdT[0:3,0:3] = R;
    AdT[0:3,3:6] = VecToso3(p)@R;
    AdT[3:6,3:6] = R;    

    return AdT
def RotX(alpha):
    return np.array([[1, 0, 0, 0],
                     [0, np.cos(alpha), -np.sin(alpha), 0],
                     [0, np.sin(alpha), np.cos(alpha), 0],
                     [0, 0, 0, 1]])

def TransX(a):
    return np.array([[1, 0, 0, a],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def TransZ(d):
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, d],
                     [0, 0, 0, 1]])

def RotZ(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                     [np.sin(theta), np.cos(theta), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
        
# Replace 'file.xml' with the path to your XML file
tree = ET.parse('rightarm/KinematicsCalibrationIndy7.xml')
root = tree.getroot()

# Access and read the values
alphas, a_values, d_values, thetas = [], [], [], []
for offset_elem in root.findall('offset'):
    alpha = float(offset_elem.get('alpha'))*np.pi/180.0
    a = float(offset_elem.get('a'))/1000.0
    d = float(offset_elem.get('d'))/1000.0
    theta = float(offset_elem.get('theta'))*np.pi/180.0
    alphas.append(alpha)
    a_values.append(a)
    d_values.append(d)
    thetas.append(theta)
    print(f"Alpha: {alpha}, a: {a}, d: {d}, Theta: {theta}")

# Create an instance of MDHParameters
MDH = MDHParameters(alphas, a_values, d_values, thetas)

# Now you can use the MDH and JacobianBodyMDH as before
# For example:
thetalist = np.array([0 ,0 ,0 ,0 ,0, 0])  # Sample thetalist

Blist,M = JacobianBodyMDH(MDH, thetalist)
Slist = Adjoint(M)@ Blist
print("M:\n" ,M)
print("Blist : \n",Blist)
print("Slist : \n",Slist)
