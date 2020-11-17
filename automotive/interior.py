#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

"""
import math
import dessia_common as dc
import pandas as pd
import numpy as npy
import volmdlr as vm
import volmdlr.faces as faces
import volmdlr.primitives2d as p2d
import volmdlr.primitives3d as p3d
from typing import List, Union

import matplotlib.pyplot as plt



class Seat(dc.DessiaObject):
    _standalone_in_db = True

    def __init__(self, cushion_length:float, cushion_angle:float, 
                 backrest_height:float, backrest_angle:float,
                 headrest_height:float, width:float,
                 floor_height:float, roof_height:float,
                 name:str=''):
        """
        floor height: the height from the origin of seat
        """
        self.cushion_length = cushion_length
        self.cushion_angle = cushion_angle
        self.backrest_height = backrest_height
        self.backrest_angle = backrest_angle
        self.headrest_height = headrest_height
        self.width = width
        self.floor_height = floor_height
        self.roof_height = roof_height

        self.backrest_base_width = 0.12
        self.backrest_top_width = 0.08
        self.name = name

    def plot(self, ax=None, x_setting=0., z_setting=0.):
        ax = self.cushion_contour().translation(vm.Point2D(x_setting,
                                                           z_setting)).plot(ax=ax)
        self.backrest_contour().translation(vm.Point2D(x_setting,
                                                           z_setting)).plot(ax=ax)
        self.headrest_contour().translation(vm.Point2D(x_setting,
                                                           z_setting)).plot(ax=ax)
        ax.axvline(0.)
        ax.axhline(0.)
        return ax

    def cushion_contour(self):
        cushion_p1 = vm.O2D
        cushion_p2 = self.cushion_length * vm.X2D.rotation(vm.O2D,
                                                           self.cushion_angle)
        cushion_p3 = cushion_p2 - vm.Y2D * 0.2
        cushion_p5 = cushion_p1 - vm.X2D * self.backrest_base_width
        cushion_p4 = vm.Point2D(cushion_p5[0], cushion_p3[1])

        cushion_points = [cushion_p1, cushion_p2, cushion_p3, cushion_p4,
                          cushion_p5]
        cushion_contour = p2d.ClosedRoundedLineSegments2D(cushion_points,
                                                          {1: 0.05})
        return cushion_contour

    def backrest_contour(self):
        backrest_p1 = vm.O2D
        backrest_p4 = - self.backrest_base_width * vm.X2D
        backrest_p2 = self.backrest_height * vm.Y2D.copy().rotation(vm.O2D,
                                                                    self.backrest_angle)
        backrest_p3 = backrest_p2 - self.backrest_top_width * vm.X2D

        backrest_points = [backrest_p1, backrest_p2, backrest_p3, backrest_p4]
        backrest_contour = p2d.ClosedRoundedLineSegments2D(backrest_points,
                                                           {1: 0.04, 2: 0.01})
        return backrest_contour

    def headrest_contour(self):
        backrest_p2 = self.backrest_height * vm.Y2D.copy().rotation(vm.O2D,
                                                                    self.backrest_angle)
        headrest_p1 = backrest_p2 + 0.15 * self.headrest_height * vm.Y2D
        headrest_p2 = backrest_p2 + self.headrest_height * vm.Y2D
        headrest_p3 = headrest_p2 - self.backrest_top_width*vm.X2D
        headrest_p4 = headrest_p1 - self.backrest_top_width*vm.X2D
        headrest_points = [headrest_p1, headrest_p2, headrest_p3, headrest_p4]
        headrest_contour = p2d.ClosedRoundedLineSegments2D(headrest_points,
                                                           {1: 0.04, 2:0.02})
        return headrest_contour

    def volmdlr_primitives(self, frame=vm.OXYZ,
                           x_setting=0.,
                           z_setting=0):
        cushion_contour = self.cushion_contour().translation(vm.Point2D(x_setting,
                                                           z_setting))
        cushion = p3d.ExtrudedProfile(frame.origin-0.5*self.width*frame.v,
                                      frame.u, frame.w, cushion_contour, [],
                                      self.width*frame.v, name='cushion',
                                      color=(0.2, 0.2, 0.2))

        backrest_contour = self.backrest_contour().translation(vm.Point2D(x_setting,
                                                           z_setting))
        backrest = p3d.ExtrudedProfile(
            frame.origin - 0.5 * self.width * frame.v,
            frame.u, frame.w, backrest_contour, [],
            self.width * frame.v,
            color=(0.2, 0.2, 0.2), name='backrest')

        headrest_contour = self.headrest_contour().translation(vm.Point2D(x_setting,
                                                           z_setting))
        headrest = p3d.ExtrudedProfile(frame.origin-0.25*self.width*frame.v,
                              frame.u, frame.w, headrest_contour, [],
                              0.5*self.width*frame.v, color=(0.2, 0.2, 0.2),
                                       name='headrest')

        floor = faces.Plane3D(vm.Frame3D(frame.origin-self.floor_height*vm.Z3D,
                                      vm.X3D, vm.Y3D, vm.Z3D))\
                        .rectangular_cut(0.25, 1, -0.5*self.width, 0.5*self.width)

        roof = faces.Plane3D(vm.Frame3D(frame.origin+self.roof_height*vm.Z3D,
                                      vm.X3D, vm.Y3D, vm.Z3D))\
                        .rectangular_cut(-0.15, 0.15, -0.5*self.width, 0.5*self.width)

        primitives = [cushion, backrest, headrest, floor, roof]
        return primitives
        

class Cockpit(dc.DessiaObject):
    _standalone_in_db = True

    def __init__(self, seat:Seat, steering_wheel_position:vm.Point2D,
                 pedals_position:vm.Point2D, name:str=''):
        self.seat = seat
        self.steering_wheel_position = steering_wheel_position
        self.pedals_position = pedals_position

        self.steering_wheel_tilt = math.radians(25)
        self.steering_wheel_diameter = 0.34
        self.lower_steering_wheel_point = \
            (self.steering_wheel_position
             -0.5*self.steering_wheel_diameter*vm.Y2D.rotation(vm.O2D,
                                                               -self.steering_wheel_tilt))
        self.name = name

    def plot(self, ax=None, seat_x_setting=0., seat_z_setting=0.):
        ax = self.seat.plot(ax=ax, x_setting=seat_x_setting,
                            z_setting=seat_z_setting)
        self.steering_wheel_position.plot(ax=ax, color='r')
        self.lower_steering_wheel_point.plot(ax=ax, color='r')
        self.pedals_position.plot(ax=ax, color='r')


        return ax

    def volmdlr_primitives(self, frame=vm.OXYZ, seat_x_setting=0., seat_z_setting=0):
        primitives = self.seat.volmdlr_primitives(frame,seat_x_setting,
                                                  seat_z_setting)

        xsw, zsw = self.steering_wheel_position
        # ysw = self.left_front_seat_position.y
        wsw = frame.u.rotation(vm.O3D, frame.v,
                               (self.steering_wheel_tilt))
        usw = frame.v.cross(wsw)
        steering_wheel_frame = vm.Frame3D(frame.origin+vm.Point3D(xsw, 0., zsw),
                                          usw, frame.v, wsw)
        steering_wheel = faces.ToroidalSurface3D(steering_wheel_frame,
                                                 0.5*self.steering_wheel_diameter,
                                                 0.017) \
            .rectangular_cut(0, vm.TWO_PI, 0, vm.TWO_PI)
        steering_wheel.name = 'Steering wheel'
        primitives.append(steering_wheel)

        xp, zp = self.pedals_position
        # yp = self.left_front_seat_position.y
        wp = vm.Z3D.rotation(vm.O3D, vm.Y3D, math.radians(-60))
        up = vm.Y3D.cross(wp)
        pedals_frame = vm.Frame3D(frame.origin + vm.Point3D(xp, 0., zp),
                                  up, frame.v, wp)

        pedals = faces.Plane3D(pedals_frame).rectangular_cut(-0.04, 0.04, -0.1,
                                                             0.1)
        pedals.name = 'Pedals'
        primitives.append(pedals)

        return primitives

class Passenger(dc.DessiaObject):
    _standalone_in_db = True

    def __init__(self, sitting_height:float, sitting_shoulder_height:float,
                 shoulder_elbow_length:float, elbow_grip_center_length:float,
                 buttock_knee_length:float, buttock_popliteal_length:float,
                 lower_leg_length:float,
                 gender:str, name:str=''):
        self.sitting_height = sitting_height
        self.sitting_shoulder_height = sitting_shoulder_height
        self.shoulder_elbow_length = shoulder_elbow_length
        self.elbow_grip_center_length = elbow_grip_center_length
        self.buttock_knee_length = buttock_knee_length
        self.buttock_popliteal_length = buttock_popliteal_length
        self.lower_leg_length = lower_leg_length
        self.gender = gender
        self.name = name

        # Computed values
        self.head_height = self.sitting_height - self.sitting_shoulder_height
        self.upper_leg_length = 0.5*(self.buttock_knee_length + self.buttock_popliteal_length)
        # self.height = self.sitting_height + self.upper_leg_length + self.lower_leg_length

        # widths
        self.chest_width = 0.25*self.sitting_shoulder_height
        self.head_width = 0.4*self.head_height
        self.upper_arm_width = 0.2*self.shoulder_elbow_length
        self.lower_arm_width = 0.2*self.elbow_grip_center_length
        self.upper_leg_width = 0.2*self.upper_leg_length
        self.lower_leg_width = 0.16 * self.lower_leg_length

        # depth
        self.chest_depth = 2.2* self.chest_width
        self.head_depth = 0.45 * self.head_height
        self.upper_arm_depth = self.upper_arm_width
        self.lower_arm_depth = self.lower_arm_width
        self.upper_leg_depth = self.upper_leg_width
        self.lower_leg_depth = self.lower_leg_width



    def head_contour(self, position):
        p1 = position + vm.Point2D(-0.5*self.head_width, -0.3* self.head_height)
        p2 = position + vm.Point2D(-0.5*self.head_width, 0.5* self.head_height)
        p3 = position + vm.Point2D(0.3*self.head_width, 0.5* self.head_height)
        p4 = position + vm.Point2D(0.7*self.head_width, 0.15* self.head_height)
        p5 = position + vm.Point2D(0.2*self.head_width, -0.3* self.head_height)
        
        contour = p2d.ClosedRoundedLineSegments2D([p1, p2, p3, p4, p5],
                                               {1: 0.3*self.head_width,
                                                2: 0.6*self.head_width,
                                                3: 0.1*self.head_width})
        return contour

    def chest_contour(self, origin=vm.O2D, chest_angle=0.):

        p1 = origin + vm.Point2D(0.8*self.chest_width, 0.)
        p2 = origin + vm.Point2D(self.chest_width, self.sitting_shoulder_height)
        p3 = origin + vm.Point2D(0, self.sitting_shoulder_height)
        p4 = origin + vm.Point2D(0, 0.)

        contour = p2d.ClosedRoundedLineSegments2D([p1, p2, p3, p4],
                                               {1: 0.3*self.chest_width})
        return contour.rotation(origin, chest_angle)


    def upper_arm_contour(self, shoulder_position:vm.Point2D,
                          upper_arm_angle:float):

        p1 = shoulder_position + vm.Point2D(-0.5*self.upper_arm_width, 0.)
        p2 = shoulder_position + vm.Point2D(-0.5*self.upper_arm_width, -self.shoulder_elbow_length)
        p3 = shoulder_position + vm.Point2D(0.5*self.upper_arm_width, -self.shoulder_elbow_length,
                                            )
        p4 = shoulder_position + vm.Point2D(0.5*self.upper_arm_width, 0.)

        contour = p2d.ClosedRoundedLineSegments2D([p1, p2, p3, p4],
                                               {1: 0.3*self.upper_arm_width,
                                                2: 0.3*self.upper_arm_width})
        return contour.rotation(shoulder_position, upper_arm_angle)

    def lower_arm_contour(self, elbow_postion:vm.Point2D,
                          lower_arm_angle:float):

        p1 = elbow_postion + vm.Point2D(0., -0.5*self.lower_arm_width)
        p2 = elbow_postion + vm.Point2D(self.elbow_grip_center_length, -0.5*self.lower_arm_width)
        p3 = elbow_postion + vm.Point2D(self.elbow_grip_center_length, 0.5*self.lower_arm_width)
        p4 = elbow_postion + vm.Point2D(0., 0.5*self.lower_arm_width)

        contour = p2d.ClosedRoundedLineSegments2D([p1, p2, p3, p4],
                                               {1: 0.3*self.lower_arm_width,
                                                2: 0.3*self.lower_arm_width})
        return contour.rotation(elbow_postion, lower_arm_angle)

    def upper_leg_contour(self,upper_leg_angle:float, origin=vm.O2D):

        p1 = origin
        p2 = origin + vm.Point2D(self.upper_leg_length, 0.)
        p3 = origin + vm.Point2D(self.upper_leg_length, self.upper_leg_width)
        p4 = origin + vm.Point2D(0., self.upper_leg_width)

        contour = p2d.ClosedRoundedLineSegments2D([p1, p2, p3, p4],
                                               {1: 0.3*self.upper_leg_width,
                                                2: 0.3*self.upper_leg_width})
        return contour.rotation(origin, upper_leg_angle)

    def lower_leg_contour(self, knee_postion:vm.Point2D,
                          lower_leg_angle:float):
        # width = 0.25*self.lower_leg_length
        p1 = knee_postion + vm.Point2D(-0.5*self.lower_leg_width, 0.)
        p2 = knee_postion + vm.Point2D(-0.5*self.lower_leg_width, -self.lower_leg_length)
        p3 = knee_postion + vm.Point2D(0.5*self.lower_leg_width, -self.lower_leg_length)
        p4 = knee_postion + vm.Point2D(0.5*self.lower_leg_width, 0.)

        contour = p2d.ClosedRoundedLineSegments2D([p1, p2, p3, p4],
                                               {1: 0.3*self.lower_leg_width,
                                                2: 0.3*self.lower_leg_width})

        return contour.rotation(knee_postion, lower_leg_angle)

    def shoulder_position(self, chest_angle):
        return vm.Point2D(0.5*self.chest_width,
                          self.sitting_shoulder_height).rotation(
                                        vm.O2D, chest_angle)

    def elbow_position(self, chest_angle, upper_arm_angle):
        shoulder_position = self.shoulder_position(chest_angle=chest_angle)
        return shoulder_position+vm.Point2D(0, -self.shoulder_elbow_length).rotation(
            vm.O2D, upper_arm_angle)

    def sit(self, seat:Seat):
        """
        Make a passenger fit in a seat returns a PassengerPosture
        """
        upper_leg_angle = seat.cushion_angle
        chest_angle = seat.backrest_angle

        roof_clerance = seat.roof_height - self.sitting_height
        if roof_clerance < 0.06:
            seat_z_setting = -(0.06 - roof_clerance)
        else:
            seat_z_setting = 0.

        try:
            lower_leg_floor_angle = math.asin((seat.floor_height + seat_z_setting + 0.5*self.upper_leg_width
                                               +self.upper_leg_length*math.sin(upper_leg_angle))/self.lower_leg_length)
        except ValueError:
            lower_leg_floor_angle = 0.5*math.pi
        knee_angle = (math.pi - upper_leg_angle - lower_leg_floor_angle)


        upper_arm_angle = chest_angle + math.radians(5.)
        lower_arm_angle = math.radians(-5.)



        return SeatingPassenger(self, seat=seat,
                              chest_angle=chest_angle,
                              upper_leg_angle=upper_leg_angle,
                              knee_angle=knee_angle,
                              upper_arm_angle=upper_arm_angle,
                              lower_arm_angle=lower_arm_angle,
                              seat_z_setting=seat_z_setting)

    def cockpit_sit(self, cockpit:Cockpit):

        seating_passenger = self.sit(cockpit.seat)

        seat_x_setting = (cockpit.pedals_position.x - 0.08
                           - seating_passenger.ankle_position.x)

        seating_passenger.seat_x_setting = seat_x_setting

        shoulder_center = seating_passenger.shoulder_position
        delta_z = shoulder_center.y - cockpit.steering_wheel_position.y
        delta_x = cockpit.steering_wheel_position.x - shoulder_center.x
        d = shoulder_center.point_distance(cockpit.steering_wheel_position)
        r, R = sorted((self.elbow_grip_center_length, self.shoulder_elbow_length))
        if d>(r+R):
            return None
        h = 0.5 / d * math.sqrt(
            4 * d ** 2 * R ** 2 - (d ** 2 - r ** 2 + R ** 2) ** 2)
        d1 = (d ** 2 - r ** 2 + R ** 2) / (2 * d)
        d2 = d - d1
        if self.elbow_grip_center_length < self.shoulder_elbow_length:
            d1, d2 = d2, d1

        upper_arm_angle = 0.5 * math.pi - math.atan(
            delta_z / delta_x) - math.atan(h / d2)
        lower_arm_angle = -math.atan(delta_z / delta_x) + math.atan(h / d1)


        seating_passenger.lower_arm_angle = lower_arm_angle
        seating_passenger.upper_arm_angle = upper_arm_angle

        return seating_passenger


class SeatingPassenger(dc.DessiaObject):
    _standalone_in_db = True

    acceptable_knee_angles = (math.radians(80), math.radians(140))
    acceptable_elbow_angles = (math.radians(70), math.radians(140))


    def __init__(self, passenger:Passenger,
                 seat:Seat,
                 chest_angle:float,
                 upper_leg_angle:float,
                 knee_angle:float, upper_arm_angle:float,
                 lower_arm_angle:float,
                 seat_x_setting:float=0.,
                 seat_z_setting:float=0.,
                 name:str=''):
        self.passenger = passenger
        self.seat = seat
        self.chest_angle = chest_angle
        self.upper_leg_angle = upper_leg_angle
        self.knee_angle = knee_angle
        self.upper_arm_angle = upper_arm_angle
        self.lower_arm_angle = lower_arm_angle
        self.seat_x_setting = seat_x_setting
        self.seat_z_setting = seat_z_setting
        self.name = name


    @property
    def lower_leg_angle(self):
        return self.upper_leg_angle + self.knee_angle - 0.5*math.pi

    @property
    def elbow_angle(self):
        return 0.5*math.pi - self.lower_arm_angle + self.upper_arm_angle

    @property
    def shoulder_position(self):
        return (self.passenger.shoulder_position(chest_angle=self.chest_angle)
                + vm.Point2D(self.seat_x_setting, self.seat_z_setting))

    @property
    def head_position(self):
        return self.shoulder_position + 0.5*self.passenger.head_height*vm.Y2D

    @property
    def elbow_position(self):
        return (self.passenger.elbow_position(
                    chest_angle=self.chest_angle,
                    upper_arm_angle=self.upper_arm_angle)
                + vm.Point2D(self.seat_x_setting, self.seat_z_setting))

    @property
    def knee_position(self):
        return (vm.Point2D(self.passenger.upper_leg_length,
                                   0.5*self.passenger.upper_leg_width).rotation(
                                vm.O2D, self.upper_leg_angle)
                +  vm.Point2D(self.seat_x_setting, self.seat_z_setting))

    @property
    def ankle_position(self):
        return self.knee_position\
               -self.passenger.lower_leg_length*vm.Y2D.rotation(vm.O2D,
                                                                self.lower_leg_angle)


    def target_criteria_mark(self, min_, max_, value):
        if value < min_:
            return 0.
        elif value > max_:
            return 0.
        else:
            targeted_value = 0.5*(min_+max_)
            return 1-abs(value-targeted_value)/(max_-min_)*0.5

    def min_criteria_mark(self, min_, acceptable_value, value):
        if value < min_:
            return 0.
        elif value > acceptable_value:
            return 1
        else:
            return 1-abs(value-acceptable_value)/(acceptable_value-min_)


    def roof_clearance_mark(self):
        mark = self.min_criteria_mark(0.02, 0.07,
                                      self.seat.roof_height - self.passenger.sitting_height)
        return mark

    def knee_angle_mark(self):
        return self.target_criteria_mark(*self.acceptable_knee_angles,
                                         self.knee_angle)

    def elbow_angle_mark(self):
        return self.target_criteria_mark(*self.acceptable_elbow_angles,
                                         self.elbow_angle)

    def mark(self):

        return SeatMark(self,
                        elbow_mark=self.elbow_angle_mark(),
                        knee_mark=self.knee_angle_mark(),
                        roof_clearance_mark=self.roof_clearance_mark())

    def chest_contour(self):
        return self.passenger.chest_contour(chest_angle=self.chest_angle,
                                            origin=vm.Point2D(
                                                self.seat_x_setting,
                                                self.seat_z_setting),
                                            )

    def head_contour(self):
        return self.passenger.head_contour(self.head_position)

    def upper_arm_contour(self):
        return self.passenger.upper_arm_contour(self.shoulder_position,
            self.upper_arm_angle)

    def lower_arm_contour(self):
        return self.passenger.lower_arm_contour(self.elbow_position,
                                                self.lower_arm_angle)

    def upper_leg_contour(self):
        return self.passenger.upper_leg_contour(self.upper_leg_angle,
                                                origin=vm.Point2D(
                                                    self.seat_x_setting,
                                                    self.seat_z_setting))

    def lower_leg_contour(self):
        return self.passenger.lower_leg_contour(self.knee_position,
                                                self.lower_leg_angle)

    def plot(self, ax=None):
        ax = self.chest_contour().plot(ax=ax)
        self.head_contour().plot(ax=ax)
        self.knee_position.plot(ax=ax, color='b')
        self.elbow_position.plot(ax=ax, color='b')
        self.shoulder_position.plot(ax=ax, color='b')
        self.ankle_position.plot(ax=ax, color='b')
        self.upper_arm_contour().plot(ax=ax)
        self.lower_arm_contour().plot(ax=ax)
        self.upper_leg_contour().plot(ax=ax)
        self.lower_leg_contour().plot(ax=ax)
        return ax

    def volmdlr_primitives(self, frame=vm.OXYZ):
        chest = p3d.ExtrudedProfile(frame.origin-0.5*self.passenger.chest_depth*frame.v,
                                    frame.u, frame.w, self.chest_contour(),
                                    [], self.passenger.chest_depth*frame.v,
                                    name='chest')


        head = p3d.ExtrudedProfile(
            frame.origin - 0.5*self.passenger.head_depth * frame.v,
            frame.u, frame.w, self.head_contour(),
            [], self.passenger.head_depth * frame.v, name='head')

        # Arms
        upper_arm_left = p3d.ExtrudedProfile(
            frame.origin
            - (0.5*self.passenger.chest_depth+self.passenger.upper_arm_depth) * frame.v,
            frame.u, frame.w, self.upper_arm_contour(),
            [], self.passenger.upper_arm_depth * frame.v, name='upper_arm_left')

        upper_arm_right = upper_arm_left.translation(
            (self.passenger.upper_arm_depth+self.passenger.chest_depth) * frame.v)
        upper_arm_right.name = 'Upper arm right'


        # lower_arm_contour.plot(ax = ax)
        lower_arm_left = p3d.ExtrudedProfile(
            frame.origin
            - (self.passenger.lower_arm_depth + 0.5*self.passenger.chest_depth) * frame.v,
            frame.u, frame.w, self.lower_arm_contour(),
            [], self.passenger.lower_arm_depth * frame.v, name='lower_arm_left')

        lower_arm_right = lower_arm_left.translation(
            (self.passenger.upper_arm_depth+self.passenger.chest_depth) * frame.v)
        lower_arm_right.name = 'Lower arm right'

        # Legs
        upper_leg_left = p3d.ExtrudedProfile(
            frame.origin + 0.3*self.passenger.upper_leg_depth * frame.v,
            frame.u, frame.w, self.upper_leg_contour(),
            [], self.passenger.upper_leg_depth * frame.v)

        upper_leg_right = upper_leg_left.translation(-1.6*self.passenger.upper_leg_depth * frame.v)
        upper_leg_right.name = 'Upper leg right'


        lower_leg_left = p3d.ExtrudedProfile(
            frame.origin + 0.3*self.passenger.upper_leg_depth * frame.v,
            frame.u, frame.w, self.lower_leg_contour(),
            [], self.passenger.lower_leg_depth * frame.v,
            name='Lower leg left')

        lower_leg_right = lower_leg_left.translation(
            -(self.passenger.lower_leg_depth
              + 0.6 * self.passenger.upper_leg_depth) * frame.v)
        lower_leg_right.name = 'Lower leg left'


        return [head, chest, upper_arm_right,
                upper_arm_left,
                lower_arm_right,
                lower_arm_left,
                upper_leg_left, upper_leg_right,
                lower_leg_left, lower_leg_right
        ]


class Population(dc.DessiaObject):
    _standalone_in_db = True

    def __init__(self, passengers:List[Passenger], name:str=''):
        self.men = []
        self.women = []
        self.name = name

        self.passengers = passengers
        for p in passengers:
            if p.gender == 'm':
                self.men.append(p)
            elif p.gender == 'f':
                self.women.append(p)
            else:
                raise NotImplementedError('gender must be m or f')

    def mark_cockpit(self, cockpit:Cockpit):
        marks = []
        for p in self.passengers:
            posture = p.cockpit_sit(cockpit)
            if posture:
                marks.append(posture.mark())
        return SeatErgonomyAnalysis(cockpit, self, marks)

    def mark_seat(self, seat:Seat):
        marks = []
        for p in self.passengers:
            posture = p.sit(seat)
            if posture:
                marks.append(posture.mark())
            # else:
            #     marks.append()
        return SeatErgonomyAnalysis(seat, self, marks)

    @classmethod
    def random_population(cls, number:int, correlation:float=0.95) -> 'Population':
        men_mean = [0.9139, 0.5978, 0.3690, 0.36, 0.6164, 0.5004, 0.5048]
        men_std_dev = [0.0296, 0.0356, 0.0179, 0.0179, 0.0299, 0.0266, 0.0276]
        number = int(0.5*number)
        women_mean = [0.8520, 0.5555, 0.3358, 0.3288, 0.5889, 0.4817, 0.4587]
        women_std_dev = [0.0349, 0.0286, 0.0174, 0.0177, 0.0296, 0.0266, 0.0261]
        n = len(men_mean)
        men_cov = npy.zeros((n, n))
        women_cov = npy.zeros((n, n))
        for i in range(n):
           for j in range(n):
               if i == j:
                   men_cov[i, j] = men_std_dev[i]*men_std_dev[j]
                   women_cov[i, j] = women_std_dev[i] * women_std_dev[j]
               else:
                   men_cov[i, j] = correlation*men_std_dev[i]*men_std_dev[j]
                   women_cov[i, j] = correlation*women_std_dev[i]*women_std_dev[j]

        men_data = npy.random.multivariate_normal(men_mean, men_cov, number)
        women_data = npy.random.multivariate_normal(women_mean, women_cov, number)

        mens = []
        womens = []

        for man in men_data:
            mens.append(Passenger(*man, gender='m'))
        for woman in women_data:
            womens.append(Passenger(*woman, gender='f'))
        return Population(mens+womens)

    def plot(self):
        data = npy.zeros((len(self.passengers), 7))
        for ip, p in enumerate(self.passengers):
            data[ip, :] = [p.sitting_height,
                           p.sitting_shoulder_height,
                           p.shoulder_elbow_length,
                           p.elbow_grip_center_length,
                           p.buttock_knee_length,
                           p.buttock_popliteal_length,
                           p.lower_leg_length]

        df = pd.DataFrame(data,
                          columns=['sitting_height',
                                   'sitting_shoulder_height',
                                   'shoulder_elbow_length',
                                   'elbow_grip_center_length',
                                   'buttock_knee_length',
                                   'buttock_popliteal_length',
                                   'lower_leg_length'
                                   ])

        pd.plotting.scatter_matrix(df, alpha=0.5)


class CarInterior(dc.DessiaObject):
    _standalone_in_db = True

    def __init__(self, cockpit: Cockpit, cockpit_position: vm.Point3D,
                 rear_seat: Seat, rear_seat_position: vm.Point2D,
                 passengers: List[SeatingPassenger]=None,
                 name: str=''):
        self.cockpit = cockpit
        self.cockpit_position = cockpit_position
        self.rear_seat = rear_seat
        self.rear_seat_position = rear_seat_position
        self.passengers = passengers
        self.name = name

    def plot(self):
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
        ax2.set_title('Front passenger')


        if self.passengers and self.passengers[0]:
            npassengers = len(self.passengers)
            ax1.set_title(
                'Driver seat_x={}mm seat_z={}mm'.format(
                    round(1000 * self.passengers[0].seat_x_setting),
                    round(1000 * self.passengers[0].seat_z_setting)))
            self.cockpit.plot(ax=ax1, seat_x_setting=self.passengers[0].seat_x_setting,
                              seat_z_setting=self.passengers[0].seat_z_setting)

            self.passengers[0].plot(ax=ax1)
            if npassengers >= 2:
                self.passengers[1].plot(ax=ax2)
                self.cockpit.seat.plot(ax=ax2,
                                       x_setting = self.passengers[1].seat_x_setting,
                                       z_setting = self.passengers[1].seat_z_setting)
        else:
            self.cockpit.plot(ax=ax1)
            self.cockpit.seat.plot(ax=ax2)



    def volmdlr_primitives(self):

        primitives = []
        cockpit_frame = vm.Frame3D(self.cockpit_position,
                                           vm.X3D, vm.Y3D, vm.Z3D)


        front_right_seat_frame = cockpit_frame.copy()
        front_right_seat_frame.origin.y = -cockpit_frame.origin.y
        primitives.extend(
            self.cockpit.seat.volmdlr_primitives(frame=front_right_seat_frame))

        xrs, zrs = self.rear_seat_position
        rear_seat_frame = vm.Frame3D(vm.Point3D(xrs, 0, zrs),
                                     vm.X3D, vm.Y3D, vm.Z3D)
        primitives.extend(
            self.rear_seat.volmdlr_primitives(frame=rear_seat_frame))
        


        if self.passengers and self.passengers[0]:
            npassengers = len(self.passengers)
            primitives.extend(
                self.cockpit.volmdlr_primitives(frame=cockpit_frame,
                    seat_x_setting = self.passengers[0].seat_x_setting,
                    seat_z_setting = self.passengers[0].seat_z_setting))
            primitives.extend(self.passengers[0].volmdlr_primitives(cockpit_frame))

            if npassengers >= 2:
                primitives.extend(
                    self.passengers[1].volmdlr_primitives(front_right_seat_frame))
        else:
            primitives.extend(
                self.cockpit.volmdlr_primitives(frame=cockpit_frame))




        return primitives

class SeatMark(dc.DessiaObject):

    def __init__(self, seating_passenger:SeatingPassenger,
                 elbow_mark:float,
                 knee_mark:float, roof_clearance_mark:float,
                 name:str=''):
        self.seating_passenger = seating_passenger
        self.elbow_mark = elbow_mark
        self.knee_mark = knee_mark
        self.roof_clearance_mark = roof_clearance_mark

        self.total = (self.elbow_mark + self.knee_mark ) /2.
        self.name = name


class SeatErgonomyAnalysis(dc.DessiaObject):
    _standalone_in_db = True

    def __init__(self, seat:Union[Seat, Cockpit], population:Population,
                 marks:List[SeatMark],
                 name:str=''):
        self.seat = seat
        self.marks = marks
        self.population = population
        self.marking_population = Population([m.seating_passenger.passenger for m in marks])
        self.name = name

    def vectors(self):
        return ([m.elbow_mark for m in self.marks],
                [m.knee_mark for m in self.marks],
                [m.roof_clearance_mark for m in self.marks],
                [m.total for m in self.marks])

    def seat_settings(self):
        return ([m.seating_passenger.seat_x_setting for m in self.marks],
                [m.seating_passenger.seat_z_setting for m in self.marks])

    def extremas(self):
        best_mark = -math.inf
        worst_mark = math.inf
        for i, tm in enumerate(self.vectors()[-1]):
            if tm > best_mark:
                best_mark = tm
                best_index = i
            if tm < worst_mark:
                worst_mark = tm
                worst_index = i

        return self.marks[worst_index].seating_passenger,\
               self.marks[best_index].seating_passenger

    def plot(self):

        npop = len(self.population.passengers)
        nz = npop - len(self.marks)

        elbow_marks, knee_marks, roof_clearance_marks, totals = self.vectors()

        fig, (ax1, ax2, ax3) = plt.subplots(3)
        ax1.hist(totals, bins=50)
        ax1.set_title('Total mark: {} % of population can not seat'.format(int(100*nz/npop)))

        ax2.hist(elbow_marks, bins=50)
        ax2.set_xlabel('Elbow angle mark')

        ax3.hist(knee_marks, bins=50)
        ax3.set_xlabel('Knee angle mark')


        fig, ax = plt.subplots()
        h = [p.sitting_height for p in self.marking_population.passengers]
        ax.plot(h, totals, 'x')
        ax.set_xlabel('sitting Height')
        ax.set_ylabel('mark')

        fig, ax = plt.subplots()
        xs, zs = self.seat_settings()
        ax.plot(xs, zs, 'x')
        ax.set_xlabel('X seat setting')
        ax.set_ylabel('Z seat setting')

    def _display_angular(self):

        filters = ['elbow_mark', 'knee_mark', 'total', 'sitting_height',
                   'seat_x_setting', 'seat_z_setting']

        values = []
        for mark in self.marks:
            value = {}
            value['elbow_mark'] = mark.elbow_mark
            value['knee_mark'] = mark.knee_mark
            value['total'] = mark.total
            value['sitting_height'] = mark.seating_passenger.passenger.sitting_height
            value['seat_x_setting'] = mark.seating_passenger.seat_x_setting
            value['seat_z_setting'] = mark.seating_passenger.seat_z_setting

            values.append(value)


        displays = [{'angular_component': 'results',
                      'filters': filters,
                      'references_attribute': 'marks',
                      'values': values,
                      'datasets': [{'label' : 'seat marks',
                                    'color' : "#99b4d6",
                                    'values' : [i for i in range(len(values))]}],
                      }]
        return displays