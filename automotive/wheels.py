#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

"""

from dessia_common.core import DessiaObject

import math
import volmdlr as vm
import volmdlr.primitives2d
import volmdlr.primitives3d

import automotive.tyres

class Rim(DessiaObject):
    _standalone_in_db = True
    _generic_eq = True
    
    def __init__(self,
                 rim_diameter:float,
                 overall_rim_diameter:float,
                 hub_diameter:float,
                 wheel_offset:float,
                 rim_width:float,
                 overall_rim_width:float,
                 web_thickness:float,
                 flange_thickness:float,
                 bolt_circle_diameter:float,
                 bolt_diameter:float,
                 number_bolts:int=5,
                 name:str=''):

        DessiaObject.__init__(self,
                              rim_diameter=rim_diameter,
                              overall_rim_diameter=overall_rim_diameter,
                              hub_diameter=hub_diameter,
                              wheel_offset=wheel_offset,
                              rim_width=rim_width,
                              overall_rim_width=overall_rim_width,
                              web_thickness=web_thickness,
                              flange_thickness=flange_thickness,
                              bolt_circle_diameter=bolt_circle_diameter,
                              bolt_diameter=bolt_diameter,
                              number_bolts=number_bolts,
                              name=name)

    def axial_upper_contour(self, frame=vm.OXY):
        split_radius = 0.6*self.bolt_diameter+0.5*self.bolt_circle_diameter
        p1 = vm.Point2D(-0.5*self.overall_rim_width, 0.5*self.overall_rim_diameter)
        p2 = vm.Point2D(-0.5*self.rim_width, 0.5*self.overall_rim_diameter)
        p3 = vm.Point2D(-0.5*self.rim_width, 0.5*self.rim_diameter)
        p4 = vm.Point2D(0.5*self.rim_width, 0.5*self.rim_diameter)
        p5 = vm.Point2D(0.5*self.rim_width, 0.5*self.overall_rim_diameter)
        p6 = vm.Point2D(0.5*self.overall_rim_width,
                        0.5*self.overall_rim_diameter)
        p7 = vm.Point2D(0.5*self.overall_rim_width,
                        0.5*self.overall_rim_diameter-self.flange_thickness)
        p8 = vm.Point2D(0.5*self.rim_width+self.flange_thickness,
                        0.5*self.overall_rim_diameter-self.flange_thickness)
        p9 = vm.Point2D(0.5*self.rim_width+self.flange_thickness,
                        0.5*self.rim_diameter-self.flange_thickness)
        p10 = vm.Point2D(self.web_thickness+self.wheel_offset,
                         0.5*self.rim_diameter-self.flange_thickness)
        p11 = vm.Point2D(self.web_thickness+self.wheel_offset, split_radius)
        p12 = vm.Point2D(self.wheel_offset, split_radius)
        p13 = vm.Point2D(self.wheel_offset,
                         0.5*self.rim_diameter-self.flange_thickness)
        p14 = vm.Point2D(-0.5*self.rim_width-self.flange_thickness,
                         0.5*self.rim_diameter-self.flange_thickness)
        p15 = vm.Point2D(-0.5*self.rim_width-self.flange_thickness,
                         0.5*self.overall_rim_diameter-self.flange_thickness)
        p16 = vm.Point2D(-0.5*self.overall_rim_width,
                         0.5*self.overall_rim_diameter-self.flange_thickness)

        points = [frame.old_coordinates(p) for p in [p1, p2, p3, p4, p5, p6, p7,
                                                    p8, p9, p10, p11, p12, p13,
                                                    p14, p15, p16]]

        contour = volmdlr.primitives2d.ClosedRoundedLineSegments2D(points,
                                                             {0: 0.25*self.flange_thickness,
                                                              1: 0.5*self.flange_thickness,
                                                              2: 0.5*self.flange_thickness,
                                                              3: 0.5*self.flange_thickness,
                                                              4: 0.5*self.flange_thickness,
                                                              5: 0.25*self.flange_thickness,
                                                              6: 0.25*self.flange_thickness,
                                                              7: 0.5*self.flange_thickness,
                                                              8: 1.5*self.flange_thickness,
                                                              9: self.web_thickness,
#                                                              10: 0.2*self.web_thickness,
#                                                              11: 0.2*self.web_thickness,
                                                              12: self.web_thickness,
                                                              13: 1.5*self.flange_thickness,
                                                              14: 0.5*self.flange_thickness,
                                                              15: 0.25*self.flange_thickness},
                                                              adapt_radius=True)

        return contour
    
    def axial_contour(self, frame=vm.OXY):
        contour = self.axial_upper_contour(frame=frame)
        contour.points[10].vector[1] = 0.5*self.hub_diameter
        contour.points[11].vector[1] = 0.5*self.hub_diameter
        return contour
    
    def transversal_outer_contour(self):
        return vm.wires.Circle2D(vm.O2D,
                                 0.6*self.bolt_diameter+0.5*self.bolt_circle_diameter)

    def bolt_circles(self):
        circles = []
        delta_angle = 2*math.pi/self.number_bolts
        inner_circle = vm.wires.Circle2D(vm.O2D, 0.5*self.hub_diameter)
        first_circle = vm.wires.Circle2D(vm.Point2D(0, 0.5*self.bolt_circle_diameter),
                                         0.5*self.bolt_diameter)
        circles = [inner_circle, first_circle]
        for i in range(1, self.number_bolts):
            circles.append(first_circle.rotation(vm.O2D, i*delta_angle))
        return circles

    def axial_plot(self, frame=vm.OXY, ax=None, measures=True):
        upper_contour = self.axial_contour(frame=frame)
        lower_contour = upper_contour.frame_mapping(vm.Frame2D(vm.O2D, -frame.u, -frame.v), side='old')
        
        fig, ax = upper_contour.plot(ax=ax)
        lower_contour.plot(ax=ax)
        
        if measures:
            m1 = vm.Measure2D(lower_contour.points[10], upper_contour.points[10], label='Hub Diameter')
            m1.plot(ax=ax)
    
            m2 = vm.Measure2D(lower_contour.points[3], upper_contour.points[3], label='Rim Diameter')
            m2.plot(ax=ax)
    
            m3 = vm.Measure2D(self.hub_diameter*vm.Y2D,
                              self.hub_diameter*vm.Y2D+vm.X2D*self.wheel_offset,
                              label='Wheel offset')
            m3.plot(ax=ax)
    
            m4 = vm.Measure2D(upper_contour.points[1], upper_contour.points[4],
                              label='Rim width')
            m4.plot(ax=ax)
    
            m5 = vm.Measure2D(lower_contour.points[0], lower_contour.points[5],
                              label='Overall rim width')
            m5.plot(ax=ax)
    
            m6 = vm.Measure2D(lower_contour.points[0], upper_contour.points[0],
                              label='Overall rim diameter')
            m6.plot(ax=ax)

        return fig, ax

        
    def transversal_plot(self, offset=0, ax=None):
        oc = vm.Circle2D(vm.O2D, self.overall_rim_diameter*0.5)
        fig, ax = oc.plot(ax=ax)
        for circle in self.bolt_circles():
            circle.plot(ax=ax)      
        return fig, ax

    def volmdlr_primitives(self, frame=vm.OXYZ):
        axial_upper_contour = self.axial_upper_contour()
        return [volmdlr.primitives3d.RevolvedProfile(frame.origin, frame.u, frame.v,
                                                    axial_upper_contour, frame.origin, frame.u,
                                                    color=(0.78, 0.78, 0.78),
                                                    name='rim radial part'),
                volmdlr.primitives3d.ExtrudedProfile(frame.origin+self.wheel_offset*frame.u, frame.v, frame.w,
                                                     self.transversal_outer_contour(),
                                                     self.bolt_circles(),
                                                     self.web_thickness*frame.u,
                                                     color=(0.78, 0.78, 0.78),
                                                     name='Rim bolt plane')]



class Wheel(DessiaObject):
    """
    An assembly of a tyre and a rim
    """
    _standalone_in_db = True
    _generic_eq = True

    def __init__(self, rim: Rim, tyre:automotive.tyres.Tyre):
        DessiaObject.__init__(self,
                              rim=rim,
                              tyre=tyre)


    def axial_plot(self, frame=vm.OXY, ax=None, measures=False):
        fig, ax = self.rim.axial_plot(frame=frame, ax=ax, measures=measures)
        self.tyre.axial_plot(ax=ax, measures=measures)

    def transversal_plot(self, frame=vm.OXY, ax=None):
        # fig, ax = self.wheel_hub.transversal_plot()    
        fig, ax = self.tyre.transversal_plot(ax=ax)
        self.rim.transversal_plot(ax=ax)
        return fig, ax

    def volmdlr_primitives(self, frame=vm.OXYZ):
        primitives = []
        primitives.extend(self.rim.volmdlr_primitives(frame))
        primitives.extend(self.tyre.volmdlr_primitives(frame))
        return primitives
