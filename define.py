import numpy as np

# fillet or screw
FL = 0
SCRW = 1

# edge 형상
FILLET = 'Fillet'
CHAMFER = 'Chamfer'
SURFACE = 'Surface'


# constant
PHI = 45
INF = np.inf

# Math
PI = np.pi; 
PI_TWO = 2* np.pi
PI_DIV_2 = np.pi/2
PI_DIV_4 = np.pi/4

# UNIT CONVERSION 
RAD2DEG = 180.0 / np.pi
DEG2RAD = np.pi / 180.0


# EROOR MESSAGE
MSG_ERR_EXCEPT = "an exception has occured"
MSG_ERR_ARG_MUSTBE_EUL = "Euler Type must be only XYZ or RPY"

# CAD length unit
UNITLESS = 0
INCHES = 1
FEET = 2
MILES = 3
MILLEMETERS = 4
CENTIMETERS = 5
METERS = 6
KILOMETERS = 7