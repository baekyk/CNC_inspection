from CAD_extract import *


class PointClass():
    def __init__(self, z, d):
        self.z = z
        self.d = d

class InspectionClass():
    def __init__(self, dxf, T_BO, T_CE, offset, theta, center_layer, height=None, unit=MILLEMETERS):
        '''
        T_BO : 로봇으로 부터 물체의 위치 \n
        T_CE : 카메라로 부터 End-effector의 위치 \n
        offset : 검사 위치로부터 카메라를 offset 시킬 거리 \n
        theta : 물체의 x 좌표로 부터 로봇의 방향, 각도(radian) \n
        center_laber : CAD 도면 상 중심선 layer의 이름 \n
        height : 검사하려는 물체의 높이 \n
        '''
        self.cad = InfoCAD(dxf, center_layer)
        self.T_BO = T_BO
        self.T_CE = T_CE
        self.offset = offset
        self.theta = theta
        self.height = height
        self.to_unit = unit
        
    def destination(self, T_OF):
        """
        get robot target pose
        """
        T_BF = self.T_BO @ T_OF @ inv_tform(self.T_CE)
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
        left = self.cad.get_r(self.cad.bottom, l_diff)[0]
        right = self.cad.get_r(self.cad.bottom, r_diff)[0]
        r = self.cad.get_r(self.cad.bottom, height)[0]

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
        inter_phi = self.det_inter_tilt(height)
        if inter_phi == 0 :
            return 0
        elif inter_phi > 0:
            height_end = height + self.offset*np.sin(PHI)
            step = 1
            margine = self.offset*0.05
        elif inter_phi < 0 :
            height_end = height - self.offset*np.sin(PHI)
            step = -1
            margine = -self.offset*0.05

        if height_end <= 0:
            return 0
        
        space = np.arange(height+margine, height_end, step)
        for h in space:
            if h > self.cad.edges[-1]:
                return inter_phi
            if h < self.cad.edges[0]:
                return 0
            
            if inter_phi == PHI*DEG2RAD:
                sight_line = h + r - height
            elif inter_phi == -PHI*DEG2RAD:
                sight_line = -h - r + height
            entity_r = self.cad.get_r(self.cad.bottom,h)
            for enti_r in entity_r:
                if enti_r >= sight_line - margine:
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
        return tform  @ troty(phi + np.pi/2)
    
    def make_all_list(self):
        '''
        모든 edge들과 fillet 부분의 중간 부분의 높이 리스트
        '''
        all_list = list()
        for i in range(len(self.cad.edges)):
            all_list.append(self.cad.edges[i])
            if self.cad.edges[i] == self.cad.edges[-1]:
                break
            mid = (self.cad.edges[i+1]+self.cad.edges[i])/2
            if self.cad.find_whichtype(self.cad.bottom, mid) == FILLET: 
                all_list.append(mid)
        return all_list
    
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
            r_list = self.cad.get_r(self.cad.bottom, h)
            T_BF_list = list()
            for r in r_list:
                T_OF = self.get_offset_fixed(p, r)
                T_BF = self.destination(T_OF)
                T_BF[:2,3] = mm2unit(self.to_unit, T_BF[:2,3])
                T_BF_list.append([T_BF, self.det_tilt(h, r)])
            dict_T_BF[h] = T_BF_list
        return dict_T_BF
    
    def list_inspt_all_edges(self):
        '''
        가공품의 밑 부분부터 위쪽으로 edge를 순차적으로 검사
        '''
        dict_T_BF = self.inspt_all_edges()
        list_all_edges = list()
        for edge, T_BF_list in dict_T_BF.items():
            for T_BF in T_BF_list:
                list_all_edges.append(T_BF[0])
        return list_all_edges
    
    def list_inspt_all_edges_tilt_ord(self):
        '''
        검사 각도 tilt 값이 PHI, 0, -PHI 값 순서로 검사
        (PHI 각도의 edge 모두 검사 -> 0 각도 -> -PHI 각도)
        '''
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
    
    def inspt_spec_edge(self):
        '''
        특정 높이 부분 검사 위치로 변환
        '''
        if self.height == None:
            raise Exception('검사하고 싶은 높이를 입력하세요.')
        p = PointClass(z= self.height, d= self.offset)
        r_list = self.cad.get_r(self.cad.bottom, self.height)
        T_BF_list = list()
        for r in r_list:
            T_OF = self.get_offset_fixed(p, r)
            T_BF = self.destination(T_OF)
            T_BF[:2,3] = mm2unit(self.to_unit, T_BF[:2,3])
            T_BF_list.append(T_BF)
        return T_BF_list
    
