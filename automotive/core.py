#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""


"""
import dessia_common as dc
import volmdlr as vm
import volmdlr.primitives2d as p2d
import volmdlr.primitives3d as p3d
import automotive.interior as interior
import automotive.wheels as wheels

class Car(dc.DessiaObject):
    def __init__(self,
                 front_wheel:wheels.Wheel,
                 rear_wheel:wheels.Wheel, wheelbase:float, track:float,
                 interior:interior.CarInterior,
                 outside_xz_contour:vm.wires.Contour2D=None,
                 name:str=''):
        self.front_wheel = front_wheel
        self.rear_wheel = rear_wheel
        self.wheelbase = wheelbase
        self.track = track
        self.interior = interior
        self.outside_xz_contour = outside_xz_contour
        dc.DessiaObject.__init__(self, name=name)

    def volmdlr_primitives(self):
        primitives = self.interior.volmdlr_primitives()

        front_left_wheel_frame = vm.Frame3D(
            vm.Point3D(0, 0.5*self.track, 0.5*self.front_wheel.tyre.diameter),
            vm.Y3D, -vm.X3D, vm.Z3D)
        primitives.extend(self.front_wheel.volmdlr_primitives(frame=front_left_wheel_frame))

        front_right_wheel_frame = vm.Frame3D(
            vm.Point3D(0, -0.5*self.track, 0.5*self.front_wheel.tyre.diameter),
            -vm.Y3D, vm.X3D, vm.Z3D)
        primitives.extend(self.front_wheel.volmdlr_primitives(frame=front_right_wheel_frame))


        rear_left_wheel_frame = vm.Frame3D(
            vm.Point3D(-self.wheelbase, 0.5*self.track, 0.5*self.rear_wheel.tyre.diameter),
            vm.Y3D, -vm.X3D, vm.Z3D)
        primitives.extend(self.rear_wheel.volmdlr_primitives(frame=rear_left_wheel_frame))

        rear_right_wheel_frame = vm.Frame3D(
            vm.Point3D(-self.wheelbase, -0.5*self.track, 0.5*self.rear_wheel.tyre.diameter),
            -vm.Y3D, vm.X3D, vm.Z3D)
        primitives.extend(self.rear_wheel.volmdlr_primitives(frame=rear_right_wheel_frame))

        if self.outside_xz_contour:
            outside_xz_contour2 = self.outside_xz_contour.offset(0.01)
            if outside_xz_contour2.area() < self.outside_xz_contour.area():
                primitives.append(p3d.ExtrudedProfile(-0.4*self.track*vm.Y3D, vm.X3D, vm.Z3D,
                                                      self.outside_xz_contour,
                                                      [outside_xz_contour2],
                                                      vm.Y3D*0.8*self.track,
                                                      alpha=0.5))
            else:
                primitives.append(p3d.ExtrudedProfile(-0.4*self.track*vm.Y3D,
                                                      vm.X3D, vm.Z3D,
                                                      outside_xz_contour2,
                                                      [self.outside_xz_contour],
                                                      vm.Y3D*0.8*self.track,
                                                      alpha=0.5))

        return primitives


