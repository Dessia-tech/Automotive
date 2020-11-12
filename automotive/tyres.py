#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""


"""

from dessia_common.core import DessiaObject
import math
import volmdlr as vm
import volmdlr.primitives2d
import volmdlr.primitives3d

class Tyre(DessiaObject):
    _standalone_in_db = True
    _generic_eq = True
    
    def __init__(self, rim_diameter:float, diameter:float, rim_width:float,
                 width:float, thickness:float,
                 top_radius:float=None, loaded_diameter:float=None, name:str=''):
        """

        """
        if top_radius is None:
            top_radius = 0.3*0.5*(diameter-rim_diameter)
            
            
        
        DessiaObject.__init__(self,
                              rim_diameter=rim_diameter,
                              diameter=diameter,
                              rim_width=rim_width,
                              width=width,
                              thickness=thickness,
                              top_radius=top_radius,
                              loaded_diameter=loaded_diameter,
                              name=name)
        
        
    def axial_contour(self):
        delta_radius = 0.5*(self.diameter-self.rim_diameter)
        # delta_width = self.width-self.rim_width
        p1 = vm.Point2D(-0.5*self.rim_width, 0.5*self.rim_diameter)
        p2 = vm.Point2D(-0.5*self.rim_width, 0.5*self.rim_diameter+0.2*delta_radius)
        p3 = vm.Point2D(-0.5*self.width, 0.5*self.rim_diameter+0.6*delta_radius)
        p4 = vm.Point2D(-0.5*self.width, 0.5*self.diameter)
        p5 = vm.Point2D(0.5*self.width, 0.5*self.diameter)
        p6 = vm.Point2D(0.5*self.width, 0.5*self.rim_diameter+0.6*delta_radius)
        p7 = vm.Point2D(0.5*self.rim_width, 0.5*self.rim_diameter+0.2*delta_radius)
        p8 = vm.Point2D(0.5*self.rim_width, 0.5*self.rim_diameter)
        p9 = vm.Point2D(0.5*self.rim_width-self.thickness, 0.5*self.rim_diameter)
        p10 = vm.Point2D(0.5*self.rim_width-self.thickness, 0.5*self.rim_diameter+0.2*delta_radius)
        p11 = vm.Point2D(0.5*self.width-self.thickness, 0.5*self.rim_diameter+0.6*delta_radius)
        p12 = vm.Point2D(0.5*self.width-self.thickness, 0.5*self.diameter-self.thickness)
        p13 = vm.Point2D(-0.5*self.width+self.thickness, 0.5*self.diameter-self.thickness)
        p14 = vm.Point2D(-0.5*self.width+self.thickness, 0.5*self.rim_diameter+0.6*delta_radius)
        p15 = vm.Point2D(-0.5*self.rim_width+self.thickness, 0.5*self.rim_diameter+0.2*delta_radius)
        p16 = vm.Point2D(-0.5*self.rim_width+self.thickness, 0.5*self.rim_diameter)
        contour = volmdlr.primitives2d.ClosedRoundedLineSegments2D([p1, p2, p3,
                                                                    p4, p5, p6,
                                                                    p7, p8, p9,
                                                                    p10, p11, p12, p13, p14, p15, p16],
                                                             {2: 0.2*delta_radius,
                                                              3: self.top_radius,
                                                              4: self.top_radius,
                                                              5: 0.2*delta_radius,
                                                              11: self.top_radius-self.thickness,
                                                              12: self.top_radius-self.thickness
                                                              })

        return contour
    
    def above_contour(self, frame=vm.OXY):
        p1 = frame.origin - 0.5*self.diameter*frame.v - 0.5*self.width*frame.u
        p2 = frame.origin + 0.5*self.diameter*frame.v - 0.5*self.width*frame.u
        p3 = frame.origin + 0.5*self.diameter*frame.v + 0.5*self.width*frame.u
        p4 = frame.origin - 0.5*self.diameter*frame.v + 0.5*self.width*frame.u
        contour = volmdlr.primitives2d.ClosedRoundedLineSegments2D([p1, p2,
                                                                    p3, p4],
                                                                     {0: self.top_radius,
                                                                      1: self.top_radius,
                                                                      2: self.top_radius,
                                                                      3: self.top_radius,
                                                                      })

        return contour
        
    def axial_plot(self, ax=None, measures=False):
        contour_top = self.axial_contour()
        contour_bottom = contour_top.Rotation(vm.O2D, math.pi)

        fig, ax = contour_top.MPLPlot(ax=ax)
        contour_bottom.MPLPlot(ax=ax)
        
        if measures:
            m1 = vm.Measure2D(contour_top.points[1], contour_top.points[6], label='Rim width')
            m1.MPLPlot(ax=ax)
            
            m2 = vm.Measure2D(0.5*(contour_bottom.points[3]+contour_bottom.points[4]),
                              0.5*(contour_top.points[3]+contour_top.points[4]), label='Diameter')
            m2.MPLPlot(ax=ax)
            
            m3 = vm.Measure2D(contour_top.points[2], contour_top.points[5], label='Width')
            m3.MPLPlot(ax=ax)
            
            m4 = vm.Measure2D(contour_top.primitives[6].center,
                              contour_top.primitives[6].interior,
                              label='Top radius', type_='radius')
            m4.MPLPlot(ax=ax)
            
            m5 = vm.Measure2D(contour_bottom.points[0], contour_top.points[7], label='Rim diameter')
            m5.MPLPlot(ax=ax)
    
            m6 = vm.Measure2D(contour_bottom.points[7], contour_bottom.points[8], label='Thickness')
            m6.MPLPlot(ax=ax)
            
    def transversal_plot(self, ax=None):
        oc = vm.Circle2D(vm.O2D, self.diameter*0.5)
        ic = vm.Circle2D(vm.O2D, self.rim_diameter*0.5)
        fig, ax = oc.MPLPlot(ax=ax)
        ic.MPLPlot(ax=ax)
        return fig, ax

    def volmdlr_primitives(self, frame=volmdlr.OXYZ):
        contour = self.axial_contour()
        return [volmdlr.primitives3d.RevolvedProfile(frame.origin, frame.u, frame.v,
                                                     contour, frame.origin, frame.u,
                                                     color=(0.3, 0.3, 0.3),
                                                     name='Tyre')]
        

    # def loaded_diameter(self):
    #     """
    #     :returns: the diameter of the tyre under load
    #     """
    #     return 0.8*self.diameter