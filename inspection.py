from .core import *


class RobotInspect():
    def __init__(self, T_BO, T_EC, points:list, robot_unit=MILLEMETERS):
        '''
        T_BO : 로봇으로부터 가공품까지의 위치 \n
        T_EC : 로봇의 End-effector부터 카메라 중심점까지의 위치 \n
        points : 가공품 좌표계 기준 카메라 위치 리스트 \n
        robot_unit : 로봇 작동을 위한 mm에서 변환시킬 단위 \n
        '''
        self.T_BO = T_BO
        self.T_EC = T_EC
        self.points = points
        self.robot_unit = robot_unit
        
    def robot_points(self):
        """
        end-effector 카메라의 t-form 리스트
        """
        T_BF_list = list()
        for T_OF in self.points:
            T_BF = self.T_BO @ T_OF @ inv_tform(self.T_EC)
            T_BF[:3,3] = mm2unit(self.robot_unit, T_BF[:3,3])
            T_BF_list.append(T_BF)
        return T_BF_list