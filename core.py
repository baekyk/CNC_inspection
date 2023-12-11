import ezdxf
import numpy as np
from .define import *
from .utils import *
            

class Dimension:
    def __init__(self):
        self.defpt = 0  # location or angle 치수선 화살표 tail
        self.defpt2 = 0 # 치수 치수선 첫번째
        self.defpt3 = 0 # 치수 치수선 두번쨰
        self.defpt4 = 0 # angle 치수선 화살표 head
        self.defpt5 = 0
        self.angle = 0
        self.text = ''
        self.loc_text = 0

class Line:
    def __init__(self):
        self.start = 0
        self.end = 0 
        # self.thickness = 0    # 사용안함
        # self.linetype = ''    # 사용안함

class Arc:
    def __init__(self):
        self.center = 0
        self.radius = 0
        self.start_angle = 0
        self.end_angle = 0
        self.start_point = 0
        self.end_point = 0

class Insert:
    def __init__(self):
        """
        형상의 특정 부분을 지시하는 지시선
        ex) Chamfer(모따기) 형상 수치 입력
        """
        self.text = ''
        self.point = 0
        self.flag = None

class InfoCAD():
    def __init__(self, target_dxf):
        self.dxf_file = target_dxf

        # -- dxf 파일 읽기 -- #
        self.doc = self.data_loader(self.dxf_file)
        self.verify_dxf(self.doc)
        self.msp = self.doc.modelspace()
        self.dxf_unit = self.get_dxf_unit()

        # -- entities 추출 -- #
        self.v_dim, self.h_dim, self.angle_dim = self.get_dimensions(self.msp)
        self.v_line, self.h_line, self.d_line= self.get_lines(self.msp)
        self.center_line = self.get_centerline(self.msp)
        self.inserts = self.get_inserts(self.doc, self.msp)
        self.arcs = self.get_arcs(self.msp)

        # -- 밑면의 entity -- #
        self.surface = self.get_surface(self.center_line, self.arcs, self.d_line, self.h_line)
        self.edges = self.get_edges(self.surface)
        self.table_edges = self.edge_table(self.edges)

    def verify_dxf(self, doc):
        layer_names = list()
        for layer in doc.layers:
            layer_names.append(layer.dxf.name)
        for name in  [CENTER_LAYER, ENTITY_LAYER, DIM_LAYER]:
            if name not in layer_names:
                raise Exception(MSG_ERR_LAYER)

    def data_loader(self, dxf_file):
        doc = ezdxf.readfile(dxf_file)
        return doc

    def get_dxf_unit(self):
        return self.doc.header['$INSUNITS']
    
    def get_dimensions(self, msp):
        '''
        수평 치수선 angle : 0에 수렴하는 작은 수
        수직 치수선 angle : 90
        arc,radial 치수선은 defpoint 1,4 만 가지고 있음
        '''
        v_dim = []
        h_dim = []
        angle_dim = []

        for d in msp.query('dimension'):
            if d.dxf.layer == DIM_LAYER:
                dim = Dimension()
                pt = self.vec2coordnt(d.dxf.defpoint)
                pt2 = self.vec2coordnt(d.dxf.defpoint2)
                pt3 = self.vec2coordnt(d.dxf.defpoint3)
                pt4 = self.vec2coordnt(d.dxf.defpoint4)
                angle = d.dxf.angle

                if pt2[0] != 0 and round(angle,3) != (0 or 90.0):
                    dim.defpt = pt
                    dim.defpt2, dim.defpt3 = arange_point(pt2, pt3)
                    dim.angle = angle
                    dim.text = (round(d.dxf.actual_measurement))
                    dim.loc_text = self.vec2coordnt(d.dxf.text_midpoint)
                    h_dim.append(dim)

                elif pt2[0] != 0 and round(angle,3) == 90.0:
                    dim.defpt = pt
                    dim.defpt2, dim.defpt3 = arange_point(pt2, pt3)
                    dim.angle = angle
                    dim.text = (round(d.dxf.actual_measurement))
                    dim.loc_text = self.vec2coordnt(d.dxf.text_midpoint)
                    v_dim.append(dim)

                elif pt2[0] == 0 and pt4[0] != 0:
                    dim.defpt2 = pt
                    dim.defpt3 = pt4
                    dim.text = (round(d.dxf.actual_measurement))
                    dim.loc_text = self.vec2coordnt(d.dxf.text_midpoint)
                    angle_dim.append(dim)
        return v_dim, h_dim, angle_dim

    def get_lines(self, msp):
        v_line = []
        h_line = []
        d_line = []
        if len(msp.query('lwpolyline')) != 0:
            for poly in msp.query('lwpolyline'):
                poly.explode()
        for l in msp.query('line'):
            if l.dxf.layer == ENTITY_LAYER:
                ln = Line()
                start = self.vec2coordnt(l.dxf.start)        
                end = self.vec2coordnt(l.dxf.end)

                if start[0] != end[0] and start[1] == end[1]:
                    ln.start, ln.end = arange_point(start, end)
                    h_line.append(ln)
                
                elif start[0] == end[0] and start[1] != end[1]:
                    ln.start, ln.end = arange_point(start, end)
                    v_line.append(ln)
                
                elif start[0] != end[0] and start[1] != end[1]:
                    ln.start, ln.end = arange_point(start, end)
                    d_line.append(ln)
        return v_line, h_line, d_line

    def get_arcs(self, msp):
        arcs =[]
        for a in msp.query('arc'):
            if a.dxf.layer == ENTITY_LAYER:
                ar = Arc()
                ar.center = self.vec2coordnt(a.dxf.center)
                ar.radius = unit2mm(self.dxf_unit, a.dxf.radius)
                ar.start_angle = a.dxf.start_angle
                ar.end_angle = a.dxf.end_angle
                ar.start_point = self.vec2coordnt(a.start_point)
                ar.end_point = self.vec2coordnt(a.end_point)
                arcs.append(ar)
        return arcs

    def get_centerline(self, msp):
        ln = Line()
        for l in msp.query('line'):
            if l.dxf.layer == CENTER_LAYER:
                ln.start = self.vec2coordnt(l.dxf.start)
                ln.end = self.vec2coordnt(l.dxf.end)
                return ln

    def get_origin(self, hline, centerline):
        hline_xs=[ln.start[0] for ln in hline]
        origin_x = min(hline_xs)
        origin_y = centerline.start[1]
        return (origin_x, origin_y)

    def get_inserts(self, doc, msp):
        b_name = []
        inserts = []
        for i in msp.query('insert'):
            b_name.append(i.dxf.name)
        
        for b in b_name:
            ins = Insert()
            ins.text = doc.blocks.get(b)[3].dxf.text
            if 'M' in ins.text:
                ins.flag = SCRW
            elif 'C' in ins.text:
                ins.flag = FL
            ins.point = self.vec2coordnt(doc.blocks.get(b)[2].dxf.vtx3)
            inserts.append(ins)
        return inserts

    def get_edges(self, surface) -> list:
        '''
        가공품 각 모서리들의 x 좌표 리스트(centerline 방향)
        '''
        edges = list()
        init = x_start_whichtype(surface[0])
        for b in surface:
            edges.append(round(x_start_whichtype(b)-init,3))
            if type(b) == Arc:
                mid = round((x_end_whichtype(b) + x_start_whichtype(b)-2*init)/2,3)
                edges.append(mid)
        edges.append(round(x_end_whichtype(surface[-1])-init,3))
        return edges

    def edge_table(self, edges) -> dict:
        '''
        key : 가공품의 모서리
        value : 해당 모서리에 대응되는 반지름 값 리스트
        '''
        edges_table = dict()
        for edge in edges:
            r_list = self.get_r(self.surface, edge)
            edges_table[edge]=r_list
        return edges_table

    def get_surface(self, center_line, arcs, d_line, h_line):
        '''
        가공품의 밑면을 이루고 있는 객체들의 리스트
        '''
        surface = list()
        cl = center_line.start[1]
        for arc in arcs:
            if arc.start_point[1] < cl or \
                arc.end_point[1] < cl:
                surface.append(arc)

        for d in d_line:
            if d.start[1] < cl:
                surface.append(d)

        for h in h_line:
            if h.start[1] < cl:
                surface.append(h)

        def sort_key(x):
            if type(x) == Line:
                return x.start[0]
            elif type(x) == Arc:
                if 360 > round(x.start_angle,4) >= 180:
                    return x.start_point[0]
                else:
                    return x.end_point[0]
        surface = sorted(surface, key=lambda x: sort_key(x) )
        return surface
    
    def get_y_line(self, x, line):
        if line.start[1] != line.end[1]:
            a = (line.end[1] - line.start[1]) / (line.end[0]- line.start[0])
            b = line.start[1]- (a*line.start[0])
            return a*x + b
        else:
            return line.end[1]

    def get_y_arc_surface(self, x, arc):
        t_y = np.sqrt(abs((arc.radius)**2-(x-arc.center[0])**2))
        y = arc.center[1] - t_y
        return y
    
    def get_y_arc_top(self, x, arc):
        t_y = np.sqrt(abs((arc.radius)**2-(x-arc.center[0])**2))
        y = arc.center[1] + t_y
        return y

    def find_whichtype(self, surface, h):
        init = x_start_whichtype(surface[0])
        last = x_end_whichtype(surface[-1])
        if h > last-init:
            raise Exception(MSG_ERR_HEIGHT)
        for b in surface:
            start = x_start_whichtype(b)-init
            end = x_end_whichtype(b)-init
            if start <= h < end:
                if type(b) == Arc:
                    return FILLET
                else:   # Line class
                    if b.start[1] != b.end[1]:
                        return CHAMFER
                    else:
                        return SURFACE

    def get_r(self, surface, h):
        init = x_start_whichtype(surface[0])
        last = x_end_whichtype(surface[-1])
        if h > round(last-init,4):
            raise Exception(MSG_ERR_HEIGHT)
        r = list()
        for i, b in enumerate(surface):
            if type(b) == Line:
                if b.start[0] <= round(h+init,4) <= b.end[0]:
                    y = self.get_y_line(h+init, b)
                    temp_r = self.center_line.start[1] - y
                    r.append(round(temp_r, 2))
                   
            if type(b) == Arc:
                if 360 > round(b.start_angle,4) >= 180:
                    if b.start_point[0] != b.end_point[0]:
                        if b.start_point[0] <= round(h+init,4) <= b.end_point[0]:
                            y = self.get_y_arc_surface(h+init, b)
                            temp_r = self.center_line.start[1] - y
                            r.append(round(temp_r, 2))
                    else:
                        if b.start_point[0] <= round(h+init,4) <= b.start_point[0]+b.radius:
                            y = self.get_y_arc_surface(h+init, b)
                            temp_r = self.center_line.start[1] - y
                            r.append(round(temp_r, 2))
                else:
                    if b.start_point[0] != b.end_point[0]:
                        if b.end_point[0] <= round(h+init,4) <= b.start_point[0]:
                            y = self.get_y_arc_top(h+init, b)
                            temp_r = self.center_line.start[1] - y
                            r.append(round(temp_r, 2))
                    else:
                        if b.end_point[0] - b.radius <= round(h+init,4) <= b.start_point[0]:
                            y = self.get_y_arc_top(h+init, b)
                            temp_r = self.center_line.start[1] - y
                            r.append(round(temp_r, 2))
        return sorted(list(set(r)))

    def vec2coordnt(self, vec):
        '''
        ezdxf point 포멧(vec3) -> tuple
        '''
        coordnt = (round(unit2mm(self.dxf_unit,vec[0]),3),
                   round(unit2mm(self.dxf_unit,vec[1]),3),
                   round(unit2mm(self.dxf_unit,vec[2]),3))
        return coordnt

    def spec_point(self, height) -> list:
        '''
        가공품의 특정 높이에 대한 검사위치의 3D좌표 리스트
        '''
        x_list = self.get_r(self.surface, height)
        y = 0
        z = height
        list_3D_points = list()
        for x in x_list:
            list_3D_points.append((x, y, z))
        return list_3D_points
    
    def all_points(self) -> dict:
        '''
        가공품의 모든 모서리에 대한 검사위치의 3D좌표 dict
        - key : 가공품의 높이
        - value : 해당 모서리에 대응되는 검사위치의 3D좌표 리스트
        '''
        dict_3D_points = dict()
        for h in self.edges:
            x_list = self.get_r(self.surface, h)
            y = 0
            z = h
            list_3D_points = list()
            for x in x_list:
                list_3D_points.append((x, y, z))
            dict_3D_points[h] = list_3D_points
        return dict_3D_points

def arange_point(pt1, pt2):
    '''
    세로선 : 아래가 시작점, 위가 끝점
    가로선 : 왼쪽이 시작점, 오른쪽이 끝점
    대각선 : 왼쪽이 시작점, 오른쪽이 끝점
    '''
    if pt1[0] == pt2[0] and pt1[1] != pt2[1]:
        if pt1[1] < pt2[1]:
            return pt1, pt2
        elif pt1[1] > pt2[1]:
            return pt2, pt1

    elif pt1[1] == pt2[1] and pt1[0] != pt2[0]:
        if pt1[0] < pt2[0]:
            return pt1, pt2
        elif pt1[0] > pt2[0]:
            return pt2, pt1
    
    elif pt1[1] != pt2[1] and pt1[0] != pt2[0]:
        if pt1[0] < pt2[0]:
            return pt1, pt2
        elif pt1[0] > pt2[0]:
            return pt2, pt1 

def x_start_whichtype(entity):
    if type(entity) == Line:
        x_point = entity.start[0]
    elif type(entity) == Arc:
        if 360 > round(entity.start_angle,4) >= 180:
            x_point = entity.start_point[0]
        else:
            if entity.start_point[0] == entity.end_point[0]:
                x_point = entity.start_point[0] - entity.radius
            else:
                x_point = entity.end_point[0]
    return x_point

def y_start_whichtype(entity):
    if type(entity) == Line:
        y_point = entity.start[1]
    elif type(entity) == Arc:
        if 360 > round(entity.start_angle,4) >= 180:
            y_point = entity.start_point[1]
        else:
            y_point = entity.end_point[1]
    return y_point

def x_end_whichtype(entity):
    if type(entity) == Line:
        x_point = entity.end[0]
    elif type(entity) == Arc:
        if 360 > round(entity.start_angle,4) >= 180:
            if entity.start_point[0] == entity.end_point[0]:
                x_point = entity.start_point[0] + entity.radius
            else:
                x_point = entity.end_point[0]
        else:
            x_point = entity.start_point[0]
    return x_point

def y_end_whichtype(entity):
    if type(entity) == Line:
        y_point = entity.end[1]
    elif type(entity) == Arc:
        if 360 > round(entity.start_angle,4) >= 180:
            y_point = entity.end_point[1]
        else:
            y_point = entity.start_point[1]
    return y_point