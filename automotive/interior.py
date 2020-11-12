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

import matplotlib.pyplot as plt

class Seat(dc.DessiaObject):
    def __init__(self, cushion_length:float, cushion_angle:float, 
                 backrest_height:float, backrest_angle:float,
                 headrest_height:float, width:float,
                 floor_height:float, roof_height:float):
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


    def volmdlr_primitives(self, frame=vm.OXYZ):
        
        backrest_base_width = 0.12
        backrest_top_width = 0.08
        
        cushion_p1 = vm.O2D
        cushion_p2 = self.cushion_length * vm.X2D.rotation(vm.O2D, self.cushion_angle)
        cushion_p3 = cushion_p2 - vm.Y2D * 0.2
        cushion_p5 = cushion_p1 - vm.X2D * backrest_base_width
        cushion_p4 = vm.Point2D(cushion_p5[0], cushion_p3[1])
        
        cushion_points = [cushion_p1, cushion_p2, cushion_p3, cushion_p4, cushion_p5]
        cushion_contour = p2d.ClosedRoundedLineSegments2D(cushion_points, 
                                                          {1: 0.05})

        cushion = p3d.ExtrudedProfile(frame.origin-0.5*self.width*frame.v,
                                      frame.u, frame.w, cushion_contour, [],
                                      self.width*frame.v, name='cushion',
                                      color=(0.2, 0.2, 0.2))
        
        backrest_p1 = vm.O2D
        backrest_p4 = - backrest_base_width * vm.X2D
        backrest_p2 = self.backrest_height * vm.Y2D.copy().rotation(vm.O2D,
                                                                    self.backrest_angle)
        backrest_p3 = backrest_p2 - backrest_top_width * vm.X2D
        
        backrest_points = [backrest_p1, backrest_p2, backrest_p3, backrest_p4]
        backrest_contour = p2d.ClosedRoundedLineSegments2D(backrest_points,
                                                          {1: 0.04, 2:0.01})
        backrest = p3d.ExtrudedProfile(frame.origin-0.5*self.width*frame.v,
                                      frame.u, frame.w, backrest_contour, [],
                                      self.width*frame.v,
                                      color=(0.2, 0.2, 0.2), name='backrest')
        
        headrest_p1 = backrest_p2 + 0.15 * self.headrest_height * vm.Y2D
        headrest_p2 = backrest_p2 + self.headrest_height * vm.Y2D
        headrest_p3 = headrest_p2 - backrest_top_width*vm.X2D
        headrest_p4 = headrest_p1 - backrest_top_width*vm.X2D
        headrest_points = [headrest_p1, headrest_p2, headrest_p3, headrest_p4]
        headrest_contour = p2d.ClosedRoundedLineSegments2D(headrest_points, 
                                                           {1: 0.04, 2:0.02})

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
    def __init__(self, seat:Seat, steering_wheel_position:vm.Point2D,
                 pedals_position:vm.Point2D):
        self.seat = seat
        self.steering_wheel_position = steering_wheel_position
        self.pedals_position = pedals_position


    def volmdlr_primitives(self, frame=vm.OXYZ):
        primitives = self.seat.volmdlr_primitives(frame)

        xsw, zsw = self.steering_wheel_position
        # ysw = self.left_front_seat_position.y
        wsw = frame.w.rotation(vm.O3D, frame.v, math.radians(-65))
        usw = frame.v.cross(wsw)
        steering_wheel_frame = vm.Frame3D(frame.origin+vm.Point3D(xsw, 0., zsw),
                                          usw, frame.v, wsw)
        steering_wheel = faces.ToroidalSurface3D(steering_wheel_frame, 0.190,
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
    def __init__(self, sitting_height, sitting_shoulder_height,
                 shoulder_elbow_length, elbow_grip_center_length,
                 buttock_knee_length, buttock_popliteal_length, lower_leg_length,
                 gender):
        self.sitting_height = sitting_height
        self.sitting_shoulder_height = sitting_shoulder_height
        self.shoulder_elbow_length = shoulder_elbow_length
        self.elbow_grip_center_length = elbow_grip_center_length
        self.buttock_knee_length = buttock_knee_length
        self.buttock_popliteal_length = buttock_popliteal_length
        self.lower_leg_length = lower_leg_length
        self.gender = gender

        # Computed values
        self.head_height = self.sitting_height - self.sitting_shoulder_height
        self.upper_leg_length = 0.5*(self.buttock_knee_length + self.buttock_popliteal_length)
        # self.height = self.sitting_height + self.upper_leg_length + self.lower_leg_length

        # widths
        self.chest_width = 0.25*self.sitting_shoulder_height
        self.head_width = 0.4*self.head_height
        self.upper_leg_width = 0.2*self.upper_leg_length
        self.lower_leg_width = 0.16 * self.lower_leg_length

        # depth
        self.chest_depth = 2.2* self.chest_width
        self.head_depth = 0.45 * self.head_height
        self.upper_leg_depth = self.upper_leg_width
        self.lower_leg_depth = self.lower_leg_width



    def head_contour(self, position):
        p1 = position + vm.Point2D(0., -0.3* self.head_height)
        p2 = position + vm.Point2D(0., 0.5* self.head_height)
        p3 = position + vm.Point2D(0.8*self.head_width, 0.5* self.head_height)
        p4 = position + vm.Point2D(1.2*self.head_width, 0.15* self.head_height)
        p5 = position + vm.Point2D(0.7*self.head_width, -0.3* self.head_height)
        
        contour = p2d.ClosedRoundedLineSegments2D([p1, p2, p3, p4, p5],
                                               {1: 0.3*self.head_width,
                                                2: 0.6*self.head_width,
                                                3: 0.1*self.head_width})
        return contour

    def chest_contour(self, chest_angle=0.):

        p1 = vm.Point2D(0.8*self.chest_width, 0.)
        p2 = vm.Point2D(self.chest_width, self.sitting_shoulder_height)
        p3 = vm.Point2D(0, self.sitting_shoulder_height)
        p4 = vm.Point2D(0, 0.)

        contour = p2d.ClosedRoundedLineSegments2D([p1, p2, p3, p4],
                                               {1: 0.3*self.chest_width})
        return contour.rotation(vm.O2D, chest_angle)


    def upper_leg_contour(self, upper_leg_angle:float):

        p1 = vm.Point2D(0., 0.)
        p2 = vm.Point2D(self.upper_leg_length, 0.)
        p3 = vm.Point2D(self.upper_leg_length, self.upper_leg_width)
        p4 = vm.Point2D(0., self.upper_leg_width)

        contour = p2d.ClosedRoundedLineSegments2D([p1, p2, p3, p4],
                                               {1: 0.3*self.upper_leg_width,
                                                2: 0.3*self.upper_leg_width})
        return contour.rotation(vm.O2D, upper_leg_angle)

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

    def shoulder_center2d(self, chest_angle):
        return vm.Point2D(0, self.sitting_shoulder_height).rotation(
                                        vm.O2D, chest_angle)

    def sit(self, seat:Seat):
        """
        Make a passenger fit in a seat returns a PassengerPosture
        """
        upper_leg_angle = seat.cushion_angle
        chest_angle = seat.backrest_angle
        try:
            lower_leg_floor_angle = math.asin((seat.floor_height + 0.5*self.upper_leg_width
                                               +self.upper_leg_length*math.sin(upper_leg_angle))/self.lower_leg_length)
        except ValueError:
            return None
        # print('lower_leg_floor_angle', math.degrees(lower_leg_floor_angle))
        knee_angle = (math.pi - upper_leg_angle - lower_leg_floor_angle)



        upper_arm_angle = math.radians(5.)
        lower_arm_angle = math.radians(-5.)

        return SeatingPassenger(self, seat=seat,
                              chest_angle=seat.backrest_angle,
                              upper_leg_angle=upper_leg_angle,
                              knee_angle=knee_angle,
                              upper_arm_angle=upper_arm_angle,
                              lower_arm_angle=lower_arm_angle)

    def cockpit_sit(self, cockpit):

        seating_passenger = self.sit(cockpit.seat)
        shoulder_center = self.shoulder_center2d(seating_passenger.chest_angle)
        delta_z = shoulder_center.z - cockpit.steering_wheel_position.z
        lower_arm_angle = math.asin(delta_z/(self.shoulder_elbow_length-self.elbow_grip_center_length))
        upper_arm_angle = math.pi - lower_arm_angle
        seating_passenger.lower_arm_angle = lower_arm_angle
        seating_passenger.upper_arm_angle = upper_arm_angle


    @classmethod
    def random_population(cls, number:int, correlation:float=0.95):
        men_mean = [0.9139, 0.5978, 0.3690, 0.036, 0.6164, 0.5004, 0.5048]
        men_std_dev = [0.0296, 0.0356, 0.0179, 0.0179, 0.0299, 0.0266, 0.0276]
        number = int(0.5*number)
        women_mean = [0.8520, 0.5555, 0.3358, 0.03288, 0.5889, 0.4817, 0.4587]
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

class SeatingPassenger(dc.DessiaObject):
    acceptable_knee_angles = (math.radians(80), math.radians(150))
    acceptable_elbow_angles = (math.radians(30), math.radians(150))


    def __init__(self, passenger:Passenger,
                 seat:Seat,
                 chest_angle:float,
                 upper_leg_angle:float,
                 knee_angle:float, upper_arm_angle:float,
                 lower_arm_angle:float):
        self.passenger = passenger
        self.seat = seat
        self.chest_angle = chest_angle
        self.upper_leg_angle = upper_leg_angle
        self.knee_angle = knee_angle
        self.upper_arm_angle = upper_arm_angle
        self.lower_arm_angle = lower_arm_angle


        # Calculated values
        self.lower_leg_angle = self.upper_leg_angle + self.knee_angle - 0.5*math.pi

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

    def mark(self):
        mark = (self.knee_angle_mark()
                + self.roof_clearance_mark())

        return mark

    def volmdlr_primitives(self, frame=vm.OXYZ):
        chest_contour = self.passenger.chest_contour(self.chest_angle)
        chest = p3d.ExtrudedProfile(frame.origin-0.5*self.passenger.chest_depth*frame.v,
                                    frame.u, frame.w, chest_contour,
                                    [], self.passenger.chest_depth*frame.v)



        shoulder_center2d = self.passenger.shoulder_center2d(self.chest_angle)
        shoulder_center3d = shoulder_center2d.to_3d(frame.origin ,frame.u, frame.w)

        head_position = shoulder_center2d + 0.5*self.passenger.head_height*vm.Y2D
        head_contour = self.passenger.head_contour(head_position)
        head = p3d.ExtrudedProfile(
            frame.origin - 0.5*self.passenger.head_depth * frame.v,
            frame.u, frame.w, head_contour,
            [], self.passenger.head_depth * frame.v, name='head')


        upper_leg_contour = self.passenger.upper_leg_contour(self.upper_leg_angle)
        upper_leg_left = p3d.ExtrudedProfile(
            frame.origin + 0.3*self.passenger.upper_leg_depth * frame.v,
            frame.u, frame.w, upper_leg_contour,
            [], self.passenger.upper_leg_depth * frame.v)

        upper_leg_right = upper_leg_left.translation(-1.6*self.passenger.upper_leg_depth * frame.v)

        knee_position = vm.Point2D(self.passenger.upper_leg_length,
                                   0.5*self.passenger.upper_leg_width).rotation(
                                vm.O2D, self.upper_leg_angle)
        lower_leg_contour = self.passenger.lower_leg_contour(knee_position,
                                                             self.lower_leg_angle)

        lower_leg_left = p3d.ExtrudedProfile(
            frame.origin + 0.3*self.passenger.upper_leg_depth * frame.v,
            frame.u, frame.w, lower_leg_contour,
            [], self.passenger.lower_leg_depth * frame.v)

        lower_leg_right = lower_leg_left.translation(
            -(self.passenger.lower_leg_depth
              + 0.6 * self.passenger.upper_leg_depth) * frame.v)



        return [head, chest, upper_leg_left, upper_leg_right,
                lower_leg_left, lower_leg_right
        ]


class Population(dc.DessiaObject):
    def __init__(self, passengers):
        self.men = []
        self.women = []

        self.passengers = passengers
        for p in passengers:
            if p.gender == 'm':
                self.men.append(p)
            elif p.gender == 'f':
                self.women.append(p)
            else:
                raise NotImplementedError('gender must be m or f')

    def mark_seat(self, seat):
        marks = []
        for p in self.passengers:
            posture = p.sit(seat)
            if posture:
                marks.append(posture.mark())
            else:
                marks.append(0.)
        return SeatErgonomyAnalysis(seat, self, marks)

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
    def __init__(self, cockpit: Cockpit, cockpit_position: vm.Point3D,
                 rear_seat: Seat, rear_seat_position: vm.Point2D,
                 driver: SeatingPassenger=None):
        self.cockpit = cockpit
        self.cockpit_position = cockpit_position
        self.rear_seat = rear_seat
        self.rear_seat_position = rear_seat_position
        self.driver = driver


    def volmdlr_primitives(self):

        primitives = []
        cockpit_frame = vm.Frame3D(self.cockpit_position,
                                           vm.X3D, vm.Y3D, vm.Z3D)
        primitives.extend(
            self.cockpit.volmdlr_primitives(frame=cockpit_frame))

        front_right_seat_frame = cockpit_frame.copy()
        front_right_seat_frame.origin.y = -cockpit_frame.origin.y
        primitives.extend(
            self.cockpit.seat.volmdlr_primitives(frame=front_right_seat_frame))

        xrs, zrs = self.rear_seat_position
        rear_seat_frame = vm.Frame3D(vm.Point3D(xrs, 0, zrs),
                                     vm.X3D, vm.Y3D, vm.Z3D)
        primitives.extend(
            self.rear_seat.volmdlr_primitives(frame=rear_seat_frame))
        

        if self.driver:
            primitives.extend(self.driver.volmdlr_primitives(cockpit_frame))
        
        return primitives

class SeatErgonomyAnalysis(dc.DessiaObject):
    def __init__(self, seat, population, marks):
        self.seat = seat
        self.population = population
        self.marks = marks

    def plot(self):
        nz = self.marks.count(0.)
        npop = len(self.population.passengers)
        fig, axs = plt.subplots(3)
        axs[0].hist(self.marks, bins=50)
        axs[0].set_title('{} % of population can not seat'.format(int(100*nz/npop)))


        fig, ax = plt.subplots()
        h = [p.sitting_height for p in self.population.passengers]
        ax.plot(h, self.marks, 'x')
        ax.set_xlabel('sitting Height')
        ax.set_ylabel('mark')
