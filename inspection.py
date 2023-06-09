from .CAD_extract import *


class PointClass():
    def __init__(self, z, d):
        self.z = z
        self.d = d

class InspectionClass():
    def __init__(self, dxf, T_BO, T_EC, offset, theta, height=None, to_unit=MILLEMETERS, phi = None):
        '''
        T_BO : 로봇으로부터 가공품까지의 위치 \n
        T_EC : 로봇의 End-effector부터 카메라 중심점까지의 위치 \n
        offset : 검사 위치로부터 카메라 중심점까지의 거리 \n
        theta : 가공품으로부터 카메라가 위치한 방향(각도)[degree] \n
        height : 검사하려는 가공품의 높이[mm] \n
        to_unit : 로봇 작동을 위한 mm에서 변환시킬 단위 \n
        phi = 검사 부위에 대한 카메라 각도[degree]
        '''
        self.cad = InfoCAD(dxf)
        self.T_BO = T_BO
        self.T_EC = T_EC
        self.offset = offset
        self.theta = theta*DEG2RAD
        self.height = height
        self.to_unit = to_unit
        self.phi = phi
        
    def destination(self, T_OF):
        """
        get robot target pose
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
    
    def get_offset_fixed(self, p:PointClass, r):
        z = p.z
        d = p.d
        phi = self.det_tilt(z, r)
        off_x = (r+d*np.cos(phi))*np.cos(self.theta)
        off_y = (r+d*np.cos(phi))*np.sin(self.theta)
        off_z = z + d*np.sin(phi)
        tform = transl([off_x, off_y, off_z])
        return tform @ trotx(np.pi) @ troty(np.pi/2 - phi)
    
    def make_all_list(self):
        '''
        모든 edge들과 fillet 중간 부분의 높이 리스트
        '''
        all_list = list()
        for i in range(len(self.cad.edges)):
            all_list.append(self.cad.edges[i])
            if self.cad.edges[i] == self.cad.edges[-1]:
                break
            mid = (self.cad.edges[i+1]+self.cad.edges[i])/2
            if self.cad.find_whichtype(self.cad.surface, mid) == FILLET: 
                all_list.append(mid)
        return all_list
    
    def make_all_table(self):
        '''
        cad.edges_table에 fillet 중간 부분 추가 하는 함수
        '''
        edges = list(self.cad.table_edges.keys())
        for i in range(len(edges)):
            if edges[i] == edges[-1]:
                break
            mid = (edges[i+1] + edges[i])/2
            if self.cad.find_whichtype(self.cad.surface, mid) == FILLET:
                r_list = self.cad.get_r(self.cad.surface, mid)
                self.cad.edges_table[mid] = r_list

    def inspt_all_edges(self):
        '''
        검사목록 별 검사 위치로 변환

        key : 가공품의 높이
        value : 가공품의 높이에 대한 검사 위치 리스트
        '''
        dict_T_BF = dict()
        edge_list = self.make_all_list()
        for h in edge_list:
            p = PointClass(z= h, d= self.offset)
            r_list = self.cad.get_r(self.cad.surface, h)
            T_BF_list = list()
            for r in r_list:
                T_OF = self.get_offset_fixed(p, r)
                T_BF = self.destination(T_OF)
                T_BF[:3,3] = mm2unit(self.to_unit, T_BF[:3,3])
                T_BF_list.append([T_BF, self.det_tilt(h, r)])
            dict_T_BF[h] = T_BF_list
        return dict_T_BF
    
    def list_all_edges(self):
        '''
        가공품의 밑 부분부터 위쪽으로 edge를 순차적으로 검사
        '''
        dict_T_BF = self.inspt_all_edges()
        list_all_edges = list()
        for edge, T_BF_list in dict_T_BF.items():
            for T_BF in T_BF_list:
                list_all_edges.append(T_BF[0])
        return list_all_edges
    
    def list_all_edges_tilt_ord(self):
        '''
        검사 각도 tilt 값이 PHI, 0, -PHI 값 순서로 검사
        (PHI 각도의 edge 모두 검사 -> 0 각도 -> -PHI 각도)
        '''
        if self.phi != None:
            return self.list_all_edges()
        else:
            dict_T_BF = self.inspt_all_edges()
            list_all_edges = list()
            for edge, T_BF_list in dict_T_BF.items():
                for T_BF in T_BF_list:
                    if T_BF[1] == PHI*DEG2RAD:
                        list_all_edges.append(T_BF[0])
            for edge, T_BF_list in dict_T_BF.items():
                for T_BF in T_BF_list:
                    if T_BF[1] == 0:
                        list_all_edges.append(T_BF[0])
            for edge, T_BF_list in dict_T_BF.items():
                for T_BF in T_BF_list:
                    if T_BF[1] == -PHI*DEG2RAD:
                        list_all_edges.append(T_BF[0])
            return list_all_edges
    
    def list_spec_edge(self):
        '''
        특정 높이 부분 검사 위치로 변환
        '''
        if self.height == None:
            raise Exception(MSG_INSERT_HEIGHT)
        p = PointClass(z= self.height, d= self.offset)
        r_list = self.cad.get_r(self.cad.surface, self.height)
        T_BF_list = list()
        for r in r_list:
            T_OF = self.get_offset_fixed(p, r)
            T_BF = self.destination(T_OF)
            T_BF[:3,3] = mm2unit(self.to_unit, T_BF[:3,3])
            T_BF_list.append(T_BF)
        return T_BF_list