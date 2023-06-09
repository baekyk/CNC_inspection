import numpy as np
from numpy import arctan2, cos, sin, tan, pi
import sys
from define import *
import math
import numpy.testing as npt

""" 
////////////////////// FUNCTIONS //////////////////////
"""
def unit2mm(unit, num):
    if unit == MILLEMETERS:
        return num
    elif unit == INCHES:
        return num*25.4
    elif unit == FEET:
        return num*304.8
    elif unit == MILES:
        return num*1609344
    elif unit == CENTIMETERS:
        return num*10
    elif unit == METERS:
        return num*1000
    elif unit == KILOMETERS:
        return num*1000000
        
def mm2unit(unit, num):
    if unit == MILLEMETERS:
        return num
    elif unit == INCHES:
        return num*0.03937
    elif unit == FEET:
        return num*0.003281
    elif unit == MILES:
        return num*6.2137e-7
    elif unit == CENTIMETERS:
        return num*0.1
    elif unit == METERS:
        return num*0.001
    elif unit == KILOMETERS:
        return num*1e-6
        
def funcname():
    return sys._getframe(1).f_code.co_name + "()"

def rotx(ang: float, unit='rad'):
    """
    get a matrix in SO(3) for rotation w.r.t x-axis
    
    :param ang: angle with respec to x-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[1,        0,         0],
              [0, cos(ang), -sin(ang)],
              [0, sin(ang),  cos(ang)]])  
              
   """
    if unit == 'deg':
        ang = np.deg2rad(ang)

    return np.array([[1,        0,         0],
                     [0, cos(ang), -sin(ang)],
                     [0, sin(ang),  cos(ang)]])

def roty(ang: float, unit='rad'):
    """
    get a matrix in SO(3) for rotation w.r.t y-axis
    
    :param ang: angle with respec to y-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[ cos(ang),  0,  sin(ang)],
              [        0,  1,         0],
              [-sin(ang),  0,  cos(ang)]])
              
   """
    if unit == 'deg':
        ang = np.deg2rad(ang)

    return np.array([[cos(ang),  0,  sin(ang)],
                     [0,  1,         0],
                     [-sin(ang),  0,  cos(ang)]])

def rotz(ang: float, unit='rad'):
    """
    get a matrix in SO(3) for rotation w.r.t z-axis
    
    :param ang: angle with respec to z-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[cos(ang), -sin(ang),  0],
              [sin(ang),  cos(ang),  0],
              [       0,         0,  1]])
              
   """

    if unit == 'deg':
        ang = np.deg2rad(ang)

    return np.array([[cos(ang), -sin(ang),  0],
                     [sin(ang),  cos(ang),  0],
                     [0,         0,  1]])

def trotx(ang: float, unit='rad'):
    """
    get a matrix in SE(3) for rotation w.r.t x-axis
    
    :param ang: angle with respec to x-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[1,        0,         0, 0],
              [0, cos(ang), -sin(ang), 0],
              [0, sin(ang),  cos(ang), 0],
              [0,        0,         0, 1]])
              
   """
    if unit == 'deg':
        ang = np.deg2rad(ang)

    mat = np.eye(4)
    mat[0:3, 0:3] = rotx(ang, unit)
    return mat

def troty(angle: float, unit='rad'):
    """
    get a matrix in SE(3) for rotation w.r.t y-axis
    
    :param ang: angle with respec to x-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[ cos(ang),  0,  sin(ang), 0],
              [        0,  1,         0, 0],
              [-sin(ang),  0,  cos(ang), 0],
              [        0,  0,         0, 1]])
              
   """

    if unit == 'deg':
        angle = np.deg2rad(angle)

    mat = np.eye(4)
    mat[0:3, 0:3] = roty(angle, unit)
    return mat

def trotz(angle: float, unit='rad'):
    """
    get a matrix in SE(3) for rotation w.r.t z-axis
    
    :param ang: angle with respec to x-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[cos(ang), -sin(ang),  0, 0],
              [sin(ang),  cos(ang),  0, 0],
              [       0,         0,  1, 0],
              [       0,         0,  0, 1]])
   """

    if unit == 'deg':
        angle = np.deg2rad(angle)

    mat = np.eye(4)
    mat[0:3, 0:3] = rotz(angle, unit)
    return mat

def r2t(rmat):
    """
    R2T Convert rotation matrix to a homogeneous transform

    :param rmat: rotation matrix
    :return: homogeneous transformation

    R2T(rmat) is an SE(2) or SE(3) homogeneous transform equivalent to an
    SO(2) or SO(3) orthonormal rotation matrix rmat with a zero translational
    component. Works for T in either SE(2) or SE(3):
    if rmat is 2x2 then return is 3x3, or
    if rmat is 3x3 then return is 4x4.

    Translational component is zero.
    """
    assert isinstance(rmat, np.matrix)
    dim = rmat.shape
    if dim[0] != dim[1]:
        raise ValueError(' Matrix Must be square ')
    elif dim[0] == 2:
        tmp = np.r_[rmat, np.zeros((1, 2))]
        mat = np.c_[tmp, np.array([[0], [0], [1]])]
        mat = np.asmatrix(mat.round(15))
        return mat
    elif dim[0] == 3:
        tmp = np.r_[rmat, np.zeros((1, 3))]
        mat = np.c_[tmp, np.array([[0], [0], [0], [1]])]
        mat = np.asmatrix(mat.round(15))
        return mat
    else:
        raise ValueError(' Value must be a rotation matrix ')

def t2r(tmat):
    """
    R2T Convert homogeneous transform to a rotation matrix

    :param tmat: homogeneous transform
    :return: rotation matrix

    T2R(tmat) is the orthonormal rotation matrix component of homogeneous
    transformation matrix tmat.  Works for T in SE(2) or SE(3)
    if tmat is 3x3 then return is 2x2, or
    if tmat is 4x4 then return is 3x3.

    Validity of rotational part is not checked
    """
    assert isinstance(tmat, np.matrix)
    dim = tmat.shape
    if dim[0] != dim[1]:
        raise ValueError(' Matrix Must be square ')
    elif dim[0] == 3:
        tmp = np.delete(tmat, [2], axis=0)
        mat = np.delete(tmp, [2], axis=1)
        mat = np.asmatrix(mat.round(15))
        return mat
    elif dim[0] == 4:
        tmp = np.delete(tmat, [3], axis=0)
        mat = np.delete(tmp, [3], axis=1)
        mat = np.asmatrix(mat.round(15))
        return mat
    else:
        raise ValueError('Value must be a rotation matrix ')
    
def translx(dx: float):
    """ get a matrix in SE(3) for displacement x direction
    """
    mat = np.eye(4)
    mat[0, 3] = dx
    return mat

def transly(dy: float):
    """ get a matrix in SE(3) for displacement y direction
    """
    mat = np.eye(4)
    mat[1, 3] = dy
    return mat

def translz(dz: float):
    """ get a matrix in SE(3) for displacement z direction
    """
    mat = np.eye(4)
    mat[2, 3] = dz
    return mat

def transl(dvec: np.ndarray or list):
    """ get a matrix in SE(3) for displacement x, y, z direction
    """
    mat = np.eye(4)
    mat[0:3, 3] = np.array(dvec)
    return mat

def inv_tform(T: np.ndarray):
    """ Compute invese matrix of a transformation matrix in SE(3)
    
    Inv T =  | R_T, -R_T *d |
             |   0,       1 |
    """
    RT = T[0:3, 0:3].T
    d = T[0:3, 3]

    res = np.eye(4)
    res[0:3, 0:3] = RT
    res[0:3, 3] = -RT @ d

    return res

def eulXYZ2r(eul):
    """ convert a 3x1 euler vector into 3x3 rotation matrix
    """
    rx = rotx(eul[0])
    ry = roty(eul[1])
    rz = rotz(eul[2])

    return rx @ ry @ rz

def eulXYZ2tr(eul):
    """ convert a 3x1 euler vector into 4x4 transfomr matrix
    """
    rx = trotx(eul[0])
    ry = troty(eul[1])
    rz = trotz(eul[2])

    return rx @ ry @ rz

def eulRPY2r(eul):
    """ convert a 3x1 euler vector into 3x3 rotation matrix
    Used function from robopy
    """
    eul = list(eul)
    eul[0] = float(eul[0])
    eul[1] = float(eul[1])
    eul[2] = float(eul[2])

    return np.array(rpy2r(eul, order='zyx', unit='rad'))

def eulRPY2tr(eul):
    """ convert a 3x1 euler vector into 4x4 SE3 matrix
    Used function from robopy
    """
    eul = list(eul)
    eul[0] = float(eul[0])
    eul[1] = float(eul[1])
    eul[2] = float(eul[2])
    
    return np.array(rpy2tr(eul, order='zyx', unit='rad'))


def pose2tr(vec, eulType = 'XYZ'):
    """ convert 6x1 pose vector into a homogeneous transform matrix
    The euler form XYZ and RPY are accepted
    """
    eul = vec[3:]
    pos = vec[0:3]

    if eulType == 'XYZ':
        T = eulXYZ2tr(eul)
    elif eulType == 'RPY_old':
        T = eulRPY2tr(eul)
    elif eulType == 'RPY':
            T = eulINDY2tr(eul)

    T[0:3, 3] = pos

    return T

def tr2pose(T, in180=False, eulType='XYZ'):
    """ convert a homogenous transform matrix into 6x1 pose vector
    The euler form XYZ and RPY are accepted
    """
    try:
        p = np.zeros(6)
        p[0:3] = T[0:3, 3]
        if eulType == 'XYZ':
            p[3:] = tr2eulXYZ(T[0:3, 0:3], in180=in180)
        elif eulType == 'RPY_old':
            p[3:] = tr2eulRPY(T[0:3, 0:3], in180=in180)
        elif eulType == 'RPY':
                p[3:] = tr2eulINDY(T[0:3, 0:3], in180=in180)
        else:
            raise Exception("[]".format(funcname()) + \
                            MSG_ERR_ARG_MUSTBE_EUL)
    except:
        print(MSG_ERR_EXCEPT)
        print("T : ", T)
        print("in180 : ", in180)
        print("eulType : ", eulType)
    return p

def tr2eulRPY(R, in180=True):
    """ convert a 3x3 rotation matrix into 3x1 RPY euler vector
    """
    r, p, y = tr2rpy(R).reshape(-1)

    if in180 == True:
        r = set_in_180(r)
        p = set_in_180(p)
        y = set_in_180(y)

    return np.array([r, p, y])

def tr2eulXYZ(R, in180=True, TINY=1e-4):
    """ convert a 3x3 rotation matrix into 3x1 XYZ euler vector
    TODO:
    - Check SO(3) Functions
    """
    try:
        if R[0, 2] < 1 and R[0, 2] > -1:
            phi = np.arctan2(-R[1, 2], R[2, 2])
            theta = np.arctan2(R[0, 2], np.sqrt(R[0, 0]**2 + R[0, 1]**2))
            psi = np.arctan2(-R[0, 1], R[0, 0])

        elif np.abs(R[0, 2] + 1) < TINY:
            theta = -np.pi/2
            psi = 0.0
            phi = np.arctan2(-R[1, 0], R[1, 1])

        elif np.abs(R[0, 2]-1) < TINY:
            theta = np.pi/2
            phi = np.arctan2(R[1, 0], R[1, 1])
            psi = 0.0
        else:
            raise Exception("[{}]".format(funcname()) + \
                            "R[0,2] is out of range (1, -1), R[0,2] :", R[0, 2])

        if in180 == True:
            phi = set_in_180(phi)
            theta = set_in_180(theta)
            psi = set_in_180(psi)

    except:
        print(MSG_ERR_EXCEPT)
        print("R : ", R)
        print("in180 : ", in180)
        print("TINY : ", TINY)

    return np.array([phi, theta, psi])


'''
/////////////////////////////////////////////////////////////////////////////
#                            EULER EUL_RPY (INDY7)
/////////////////////////////////////////////////////////////////////////////
'''
def eulINDY2r(eul):
    eul = list(eul)
    eul[0] = float(eul[0])
    eul[1] = float(eul[1])
    eul[2] = float(eul[2])
    r, p, y = eul
    
    return rotz(eul[2]) @ roty(eul[1]) @ rotx(eul[0])

def eulINDY2tr(eul):
    eul = list(eul)
    eul[0] = float(eul[0])
    eul[1] = float(eul[1])
    eul[2] = float(eul[2])
    r, p, y = eul
    
    return trotz(eul[2]) @ troty(eul[1]) @ trotx(eul[0])

def tr2eulINDY(R, in180=True, TINY=1e-4):
    """ convert a 3x3 rotation matrix into 3x1 INDY7 euler vector
    TODO:
    - Check SO(3) Functions
    """
    
    try:
        if -R[2, 0] < 1 and -R[0, 2] > -1:
            r = np.arctan2(R[2, 1], R[2, 2])
            p = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
            y = np.arctan2(R[1, 0], R[0, 0])
        elif np.abs(-R[2, 0] + 1) < TINY:
            p = -np.pi/2
            r = 0.0
            y = np.arctan2(-R[1, 2], R[1, 1]) # 
        elif np.abs(-R[2, 0] - 1) < TINY:
            p = np.pi/2
            r = 0.0
            y = np.arctan2(R[1, 2], R[1, 1])
        else:
            raise Exception("[{}]".format(funcname()) + \
                            "R[0,2] is out of range (1, -1), R[0,2] :", R[0, 2])
        if in180 == True:
            r = set_in_180(r)
            p = set_in_180(p)
            y = set_in_180(y)

    except:
        print(MSG_ERR_EXCEPT)
        print("R : ", R)
        print("in180 : ", in180)
        print("TINY : ", TINY)

    return np.array([r, p, y])

    
def rpy2r(thetas, order='zyx', unit='rad'):
    """
    RPY2R Roll-pitch-yaw angles to rotation matrix

    :param thetas: list of angles
    :param order: 'xyz', 'zyx' or 'yxz'
    :param unit: 'rad' or 'deg'
    :return: rotation matrix

    RPY2R(ROLL, PITCH, YAW, OPTIONS) is an SO(3) orthonormal rotation matrix
    (3x3) equivalent to the specified roll, pitch, yaw angles angles.
    These correspond to rotations about the Z, Y, X axes respectively. If ROLL,
    PITCH, YAW are column vectors (Nx1) then they are assumed to represent a
    trajectory and R is a three-dimensional matrix (3x3xN), where the last index
    corresponds to rows of ROLL, PITCH, YAW.

    R = RPY2R(RPY, OPTIONS) as above but the roll, pitch, yaw angles are taken
    from the vector (1x3) RPY=[ROLL,PITCH,YAW]. If RPY is a matrix(Nx3) then R
    is a three-dimensional matrix (3x3xN), where the last index corresponds to
    rows of RPY which are assumed to be [ROLL,PITCH,YAW].

    Options::
        'deg'   Compute angles in degrees (radians default)
        'xyz'   Rotations about X, Y, Z axes (for a robot gripper)
        'yxz'   Rotations about Y, X, Z axes (for a camera)

    Note::
    - Toolbox rel 8-9 has the reverse angle sequence as default.
    - ZYX order is appropriate for vehicles with direction of travel in the X
    direction.  XYZ order is appropriate if direction of travel is in the Z direction.
    """
    if type(thetas[0]) is float or type(thetas[0]) is int:
        # TODO
        # enforce if one element is list.
        # All are list. OR one element is int or float then all are either int or float
        thetas = [thetas]  # Put list in a list

    if unit == 'deg':
        thetas = [[(angles * math.pi / 180) for angles in each_rpy] for each_rpy in thetas]
    if type(thetas[0]) is list:
        roll = [theta[0] for theta in thetas]
        pitch = [theta[1] for theta in thetas]
        yaw = [theta[2] for theta in thetas]

        if order == 'xyz' or order == 'arm':
            x = [rotx(theta) for theta in yaw]
            y = [roty(theta) for theta in pitch]
            z = [rotz(theta) for theta in roll]
            xyz = [(x[i] * y[i] * z[i]) for i in range(len(thetas))]
            xyz = [np.asmatrix(each.round(15)) for each in xyz]
            if len(xyz) == 1:
                return xyz[0]
            else:
                return xyz
        if order == 'zyx' or order == 'vehicle':
            z = [rotz(theta) for theta in yaw]
            y = [roty(theta) for theta in pitch]
            x = [rotx(theta) for theta in roll]
            zyx = [(z[i] * y[i] * x[i]) for i in range(len(thetas))]
            zyx = [np.asmatrix(each.round(15)) for each in zyx]
            if len(zyx) == 1:
                return zyx[0]
            else:
                return zyx
        if order == 'yxz' or order == 'camera':
            y = [roty(theta) for theta in yaw]
            x = [rotx(theta) for theta in pitch]
            z = [rotz(theta) for theta in roll]
            yxz = [(y[i] * x[i] * z[i]) for i in range(len(thetas))]
            yxz = [np.asmatrix(each.round(15)) for each in yxz]
            if len(yxz) == 1:
                return yxz[0]
            else:
                return yxz
    else:
        raise TypeError('thetas must be a list of roll pitch yaw angles\n'
                        'OR a list of list of roll pitch yaw angles.')


# ---------------------------------------------------------------------------------------#
def rpy2tr(thetas, order='zyx', unit='rad'):
    """
    RPY2TR Roll-pitch-yaw angles to homogeneous transform

    :param thetas: list of angles
    :param order: order can be 'xyz'/'arm', 'zyx'/'vehicle', 'yxz'/'camera'
    :param unit: unit of input angles
    :return: homogeneous transformation matrix

    T = RPY2TR(ROLL, PITCH, YAW, OPTIONS) is an SE(3) homogeneous
    transformation matrix (4x4) with zero translation and rotation equivalent
    to the specified roll, pitch, yaw angles angles. These correspond to
    rotations about the Z, Y, X axes respectively. If ROLL, PITCH, YAW are
    column vectors (Nx1) then they are assumed to represent a trajectory and
    R is a three-dimensional matrix (4x4xN), where the last index corresponds
    to rows of ROLL, PITCH, YAW.

    T = RPY2TR(RPY, OPTIONS) as above but the roll, pitch, yaw angles are
    taken from the vector (1x3) RPY=[ROLL,PITCH,YAW]. If RPY is a matrix
    (Nx3) then R is a three-dimensional matrix (4x4xN), where the last index
    corresponds to rows of RPY which are assumed to be
    ROLL,PITCH,YAW].

    Options::
     'deg'   Compute angles in degrees (radians default)
     'xyz'   Rotations about X, Y, Z axes (for a robot gripper)
     'yxz'   Rotations about Y, X, Z axes (for a camera)

    Note::
    - Toolbox rel 8-9 has the reverse angle sequence as default.
    - ZYX order is appropriate for vehicles with direction of travel in the X
    direction.  XYZ order is appropriate if direction of travel is in the Z
    direction.
    """
    rot = rpy2r(thetas, order, unit)
    if type(rot) is list:
        rot = [r2t(each) for each in rot]
    else:
        rot = r2t(rot)
    return rot

def set_in_180(x):
    while(1):
        if x > PI or x < -PI:
            if x <= -PI:
                x += PI_TWO
            elif x > PI:
                x -= PI_TWO
        else:
            break
    return x

def tr2eul(tr, unit='rad', flip=False):
    """
    TR2EUL Convert homogeneous transform to Euler angles
    :param tr: Homogeneous transformation
    :param unit: 'rad' or 'deg'
    :param flip: True or False
    :return: Euler angles
    TR2EUL(T, OPTIONS) are the ZYZ Euler angles (1x3) corresponding to the rotational part of a homogeneous transform T (4x4). The 3 angles EUL=[PHI,THETA,PSI] correspond to sequential rotations about the Z, Y and Z axes respectively.
    TR2EUL(R, OPTIONS) as above but the input is an orthonormal rotation matrix R (3x3).
    If R (3x3xK) or T (4x4xK) represent a sequence then each row of EUL corresponds to a step of the sequence.
    Options::
    'deg'   Compute angles in degrees (radians default)
    'flip'  Choose first Euler angle to be in quadrant 2 or 3.
    Notes::
    - There is a singularity for the case where THETA=0 in which case PHI is arbitrarily set to zero and PSI is the sum (PHI+PSI).
    - Translation component is ignored.
    """

    if tr.ndim > 2:
        eul = np.zeros([tr.shape[2], 3])
        for i in range(0, tr.shape[2]):
            eul[i, :] = tr2eul(tr[i, :, :])
        return eul
    else:
        eul = np.zeros([1, 3])

    if abs(tr[0, 2]) < np.spacing([1])[0] and abs(tr[1, 2]) < np.spacing([1])[0]:
        eul[0, 0] = 0
        sp = 0
        cp = 0
        eul[0, 1] = math.atan2(cp * tr[0, 2] + sp * tr[1, 2], tr[2, 2])
        eul[0, 2] = math.atan2(-sp * tr[0, 0] + cp * tr[1, 0], -sp * tr[0, 1] + cp * tr[1, 1])
    else:
        if flip:
            eul[0, 0] = math.atan2(-tr[1, 2], -tr[0, 2])
        else:
            eul[0, 0] = math.atan2(tr[1, 2], tr[0, 2])
        sp = math.sin(eul[0, 0])
        cp = math.cos(eul[0, 0])
        eul[0, 1] = math.atan2(cp * tr[0, 2] + sp * tr[1, 2], tr[2, 2])
        eul[0, 2] = math.atan2(-sp * tr[0, 0] + cp * tr[1, 0], -sp * tr[0, 1] + cp * tr[1, 1])

    if unit == 'deg':
        eul = eul * 180 / math.pi

    return eul


# ------------------------------------------------------------------------------------------------------------------- #
def tr2rpy(tr, unit='rad', order='zyx'):
    """
    TR2RPY Convert a homogeneous transform to roll-pitch-yaw angles
    :param tr: Homogeneous transformation
    :param unit: 'rad' or 'deg'
    :param order: 'xyz', 'zyx' or 'yxz'
    :return: Roll-pitch-yaw angle
    TR2RPY(T, options) are the roll-pitch-yaw angles (1x3) corresponding to the rotation part of a homogeneous transform T. The 3 angles RPY=[R,P,Y] correspond to sequential rotations about the Z, Y and X axes respectively.
    TR2RPY(R, options) as above but the input is an orthonormal rotation matrix R (3x3).
    If R (3x3xK) or T (4x4xK) represent a sequence then each row of RPY corresponds to a step of the sequence.
    Options::
    'deg'   Compute angles in degrees (radians default)
    'xyz'   Return solution for sequential rotations about X, Y, Z axes
    'yxz'   Return solution for sequential rotations about Y, X, Z axes
    Notes::
    - There is a singularity for the case where P=pi/2 in which case R is arbitrarily set to zero and Y is the sum (R+Y).
    - Translation component is ignored.
    - Toolbox rel 8-9 has the reverse default angle sequence as default
    """

    if tr.ndim > 2:
        rpy = np.zeros([tr.shape[2], 3])
        for i in range(0, tr.shape[2]):
            rpy[i, :] = tr2rpy(tr[i, :, :])
        return rpy
    else:
        rpy = np.zeros([1, 3])

    if isrot(tr) or ishomog(tr, dim=[4, 4]):
        if order == 'xyz' or order == 'arm':
            if abs(abs(tr[0, 2]) - 1) < np.spacing([1])[0]:
                rpy[0, 0] = 0
                rpy[0, 1] = math.asin(tr[0, 2])
                if tr[0, 2] > 0:
                    rpy[0, 2] = math.atan2(tr[2, 1], tr[1, 1])
                else:
                    rpy[0, 2] = -math.atan2(tr[1, 0], tr[2, 0])
            else:
                rpy[0, 0] = -math.atan2(tr[0, 1], tr[0, 0])
                rpy[0, 1] = math.atan2(tr[0, 2] * math.cos(rpy[0, 0]), tr[0, 0])
                rpy[0, 2] = -math.atan2(tr[1, 2], tr[2, 2])
        if order == 'zyx' or order == 'vehicle':
            if abs(abs(tr[2, 0]) - 1) < np.spacing([1])[0]:
                rpy[0, 0] = 0
                rpy[0, 1] = -math.asin(tr[2, 0])
                if tr[2, 0] < 0:
                    rpy[0, 2] = -math.atan2(tr[0, 1], tr[0, 2])
                else:
                    rpy[0, 2] = math.atan2(-tr[0, 1], -tr[0, 2])
            else:
                rpy[0, 0] = math.atan2(tr[2, 1], tr[2, 2])
                rpy[0, 1] = math.atan2(-tr[2, 0] * math.cos(rpy[0, 0]), tr[2, 2])
                rpy[0, 2] = math.atan2(tr[1, 0], tr[0, 0])
        if order == 'yxz' or order == 'camera':
            if abs(abs(tr[1, 2]) - 1) < np.spacing([1])[0]:
                rpy[0, 0] = 0
                rpy[0, 1] = -math.asin(tr[1, 2])
                if tr[1, 2] < 0:
                    rpy[0, 2] = -math.atan2(tr[2, 0], tr[0, 0])
                else:
                    rpy[0, 2] = math.atan2(-tr[2, 0], -tr[2, 1])
            else:
                rpy[0, 0] = math.atan2(tr[1, 0], tr[1, 1])
                rpy[0, 1] = math.atan2(-math.cos(rpy[0, 0]) * tr[1, 2], tr[1, 1])
                rpy[0, 2] = math.atan2(tr[0, 2], tr[2, 2])
    else:
        raise TypeError('Argument must be a 3x3 or 4x4 matrix.')

    if unit == 'deg':
        rpy = rpy * 180 / math.pi

    return rpy

def ishomog(tr, dim, rtest=''):
    """ISHOMOG Test if SE(3) homogeneous transformation matrix.
    ISHOMOG(T) is true if the argument T is of dimension 4x4 or 4x4xN, else false.
    ISHOMOG(T, 'valid') as above, but also checks the validity of the rotation sub-matrix.
    See Also: isrot, ishomog2, isvec"""
    try:
        assert type(tr) is np.matrix, "Argument should be a numpy matrix"
        assert dim == (3, 3) or dim == (4, 4)
    except AssertionError:
        return False
    is_valid = None
    if rtest == 'valid':
        is_valid = lambda matrix: abs(np.linalg.det(matrix) - 1) < np.spacing([1])[0]
    flag = True
    if is_mat_list(tr):
        for matrix in tr:
            if not (matrix.shape[0] == dim[0] and matrix.shape[1] == dim[0]):
                flag = False
        # if rtest = 'valid'
        if flag and rtest == 'valid':
            flag = is_valid(tr[0])  # As in matlab code only first matrix is passed for validity test
            # TODO-Do we need to test all matrices in list for validity of rotation submatrix -- Yes
    elif isinstance(tr, np.matrix):
        if tr.shape[0] == dim[0] and tr.shape[1] == dim[0]:
            if flag and rtest == 'valid':
                flag = is_valid(tr)
        else:
            flag = False
    else:
        raise ValueError('Invalid data type passed to common.ishomog()')
    return flag

def isrot(rot, dtest=False):
    """
    ISROT Test if SO(2) or SO(3) rotation matrix
    ISROT(rot) is true if the argument if of dimension 2x2, 2x2xN, 3x3, or 3x3xN, else false (0).
    ISROT(rot, 'valid') as above, but also checks the validity of the rotation.
    See also  ISHOMOG, ISROT2, ISVEC.
    """
    if type(rot) is np.matrix:
        rot = [rot]
    if type(rot) is list:
        for each in rot:
            try:
                assert type(each) is np.matrix
                assert each.shape == (3, 3)
                npt.assert_almost_equal(np.linalg.det(each), 1)
            except AssertionError:
                return False
    return True

def is_mat_list(list_matrices):
    """is_mat_list checks(arg1) checks if arg1
    is a list containing numpy matrix data type elements or not.
    If not, False is returned."""
    flag = True
    if isinstance(list_matrices, list):
        for matrix in list_matrices:
            if not isinstance(matrix, np.matrix):
                flag = False
                # TODO Check for matrix dimensions?
    else:
        flag = False
    return flag