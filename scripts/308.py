#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""


"""

import math
import automotive.core as automotive
import automotive.interior as interior
import automotive.wheels as wheels
import automotive.tyres as tyres
import volmdlr as vm


hub_diameter = 0.08
rim_diameter = 0.0254*15
HUB_LENGTH = 0.065
TRANSMISSION_DIAMETER = 0.080

tyre_diameter = 0.621
tyre_loaded_diameter = 0.600
tyre_width = 0.195
rim_width = 0.165

WHEEL_OFFSET = 0.045996

rim = wheels.Rim(rim_diameter, rim_diameter+0.022, hub_diameter,
          WHEEL_OFFSET, rim_width, rim_width+0.010, 0.008, 0.003, 0.112, 0.012)
tyre = tyres.Tyre(rim_diameter, tyre_diameter, rim_width, tyre_width, 0.010, loaded_diameter=tyre_loaded_diameter)
wheel = wheels.Wheel(rim, tyre)

# wheel.babylonjs()

front_seat_cushion_length = 66*0.00674
front_seat_cushion_angle = math.radians(8.62)
front_seat_backrest_height = 86*0.00674
front_seat_backrest_angle = math.radians(21.54)
front_seat_headrest_height = 33 * 0.00674
front_seat_width = 48 * 0.00674
front_seat_x = -213 * 0.00674
front_seat_y = 51.5 * 0.00674
front_seat_z = 77 * 0.00674

back_seat_cushion_length = 66*0.00674
back_seat_cushion_angle = math.radians(9.90)
back_seat_backrest_height = 93*0.00674
back_seat_backrest_angle = math.radians(18.90)
back_seat_headrest_height = 21 * 0.00674
back_seat_width = 149 * 0.00674
back_seat_x = -337 * 0.00674
back_seat_z = 77 * 0.00674
wheelbase = 2.620
track = 1.559
seat_height = 0.29
front_roof_height = 0.88
rear_roof_height = 0.87

scale = 0.00678
steering_wheel_position = vm.Point2D(-(261-117)*scale-front_seat_x,
                                     1.457-(86-3)*scale-front_seat_z)
pedals_position = (-(191-117)*scale-front_seat_x,
                   1.457-(162-3)*scale-front_seat_z)

outside_contour_pixels = [(2, 186), (2, 134), (17, 109), (46, 93), (94, 80),
                          (162, 70), (177, 66), (232, 37), (273, 18), (303, 8),
                          (347, 4), (407, 3), (480, 7), (549, 17), (615, 76),
                          (617, 118), (624, 126), (624, 166), (606, 181),
                          (575, 185), (466, 192), (174, 195)]
outside_contour_points = []
for px, py in outside_contour_pixels:
    outside_contour_points.append(vm.Point2D(-(px-117)*scale, 1.457-(py-3)*scale))


front_seat = interior.Seat(front_seat_cushion_length,
                           front_seat_cushion_angle,
                           front_seat_backrest_height,
                           front_seat_backrest_angle,
                           front_seat_headrest_height,
                           front_seat_width,
                           seat_height,
                           front_roof_height,
                           )

cockpit = interior.Cockpit(front_seat, steering_wheel_position, pedals_position)

back_seat = interior.Seat(back_seat_cushion_length,
                          back_seat_cushion_angle,
                          back_seat_backrest_height,
                          back_seat_backrest_angle,
                          back_seat_headrest_height,
                          back_seat_width,
                          seat_height,
                          rear_roof_height)


population = interior.Passenger.random_population(2000)
# population.plot()

random_passenger = population.passengers[-1]
posture = random_passenger.sit(front_seat)
# posture.babylonjs()

interior_308 = interior.CarInterior(cockpit, vm.Point3D(front_seat_x,
                                                        front_seat_y,
                                                        front_seat_z),
                                    back_seat,
                                    vm.Point2D(back_seat_x, back_seat_z),
                                    driver = posture)





peugeot_308 = automotive.Car(front_wheel=wheel, rear_wheel=wheel,
                             wheelbase=wheelbase,
                             track=track, interior=interior_308,
                             outside_xz_contour = vm.wires.ClosedPolygon2D(outside_contour_points),
                             name='Peugeot 308')


peugeot_308.babylonjs(debug=True)

analysis = population.mark_seat(front_seat)
analysis.plot()