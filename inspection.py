from .core import *


class RobotInspectPoint():
    def __init__(self, InfoCAD, T_BO, T_EC, offset, theta, height=None, robot_unit=MILLEMETERS, phi=None):
        '''
        cad : InfoCAD 클래스 객체
        T_BO : 로봇으로부터 가공품까지의 위치 \n
        T_EC : 로봇의 End-effector부터 카메라 중심점까지의 위치 \n
        offset : 검사 위치로부터 카메라 중심점까지의 거리 \n
        theta : 가공품으로부터 카메라가 위치한 방향(각도)[degree] \n
        robot_unit : 로봇 작동을 위한 mm에서 변환시킬 단위 \n
        phi = 검사 부위에 대한 카메라 각도[degree]
        '''
        self.cad = InfoCAD
        self.T_BO = T_BO
        self.T_EC = T_EC
        self.offset = offset
        self.theta = theta*DEG2RAD
        self.height = height
        self.robot_unit = robot_unit
        self.phi = phi
        
    def destination(self, T_OF):
        """
        end-effector 카메라의 t-form
        - 로봇 좌표계 기준
        """
        T_BF = self.T_BO @ T_OF @ inv_tform(self.T_EC)
        return T_BF
    
    def det_inter_tilt(self, height):
        '''
        가공품의 특정 높이에서 단순 상하 형상 비교하여 카메라 tilt 값 반환
        '''
        diff = 0.5
        l_diff = height-diff
        r_diff = height+diff
        if r_diff > self.cad.edges[-1]:
            r_diff = self.cad.edges[-1]
        if l_diff < self.cad.edges[0]:
            l_diff = self.cad.edges[0]
        left = self.cad.get_r(self.cad.surface, l_diff)[0]
        right = self.cad.get_r(self.cad.surface, r_diff)[0]
        r = self.cad.get_r(self.cad.surface, height)[0]

        if left < r and r <= right:
            return -PHI*DEG2RAD
        elif left <= r and r < right:
            return -PHI*DEG2RAD
        elif left == r and r == right:
            return 0
        elif left > r and r < right:
            return 0
        elif left >= r and r > right:
            return PHI*DEG2RAD
        elif left > r and r >= right:
            return PHI*DEG2RAD
        
    def det_tilt(self, height, r):
        '''
        det_inter_tilt에서 반환된 tilt 값에 대한 카메라 시야를 고려하여
        다른 형상이 시야를 방해하는지 고려하여 tilt 값 반환
        '''
        if self.phi != None:
            return self.phi*DEG2RAD
        else:
            inter_phi = self.det_inter_tilt(height)
            if inter_phi == 0 :
                return 0
            elif inter_phi > 0:
                height_end = height + self.offset*np.sin(PHI*DEG2RAD)
            elif inter_phi < 0 :
                height_end = height - self.offset*np.sin(PHI*DEG2RAD)
            margine = self.offset*0.05

            if height_end <= 0:
                return 0
            
            for edge, r_list in self.cad.table_edges.items():
                if inter_phi == PHI*DEG2RAD:
                    sight_line = edge + r - height
                    if height <= edge <= height_end:
                        for sub_r in r_list:
                            if sub_r >= sight_line + margine :
                                return 0
                elif inter_phi == -PHI*DEG2RAD:
                    sight_line = -edge + r + height
                    if height >= edge >= height_end:
                        for sub_r in r_list:
                            if sub_r >= sight_line + margine :
                                return 0
            
            return inter_phi
    
    def get_offset_fixed(self, x, z, offset):
        '''
        가공품 검사위치 포인트로부터 offset과 카메라 tilt를 고려한 카메라의 t-form
        - 가공품 좌표계 기준
        '''
        phi = self.det_tilt(z, x)
        off_x = (x+offset*np.cos(phi))*np.cos(self.theta)
        off_y = (x+offset*np.cos(phi))*np.sin(self.theta)
        off_z = z + offset*np.sin(phi)
        tform = transl([off_x, off_y, off_z])
        return tform @ trotx(np.pi) @ troty(np.pi/2 - phi)
    
    def list_spec_inspect(self):
        '''
        가공품의 특정높이에 대한 end-effector 카메라가 위치할 t-form 리스트
        '''
        T_BF_list = list()
        points = self.cad.spec_point(self.height)
        for point in points:
            T_OF = self.get_offset_fixed(point[0], point[2], self.offset)
            T_BF = self.destination(T_OF)
            T_BF[:3,3] = mm2unit(self.robot_unit, T_BF[:3,3])
            T_BF_list.append(T_BF)
        return T_BF_list
    
    def list_all_inspect(self):
        '''
        가공품의 모든 모서리에 대한 end-effector 카메라가 위치할 t-form 리스트
        '''
        T_BF_list = list()
        dict_points = self.cad.all_points()
        for points in dict_points.values():
            for point in points:
                T_OF = self.get_offset_fixed(point[0], point[2], self.offset)
                T_BF = self.destination(T_OF)
                T_BF[:3,3] = mm2unit(self.robot_unit, T_BF[:3,3])
                T_BF_list.append(T_BF)
        return T_BF_list