import numpy as np

# fillet or screw
FL = 0
SCRW = 1

# edge 형상
FILLET = 'Fillet'
CHAMFER = 'Chamfer'
SURFACE = 'Surface'

# constant
INF = np.inf
PHI = 45

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
MSG_ERR_LAYER = "check name of layers for centerline, entity, dimension"
MSG_ERR_HEIGHT = "inputed height exceeds the maximum height of object"
MSG_INSERT_HEIGHT = "Please input height what you want to inspect"

# CAD length unit
UNITLESS = 0
INCHES = 1
FEET = 2
MILES = 3
MILLEMETERS = 4
CENTIMETERS = 5
METERS = 6
KILOMETERS = 7

# layer name
CENTER_LAYER = 'CENTER'
ENTITY_LAYER = 'SHAPE'
DIM_LAYER = 'DIM'