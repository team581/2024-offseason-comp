import dataclasses
import math

import numpy as np
import matplotlib.pyplot as plt

import pathlib
import json


@dataclasses.dataclass
class Point:
    x: float # Meters
    y: float # Meters

    def add(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def minus(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def times(self, multiplier):
        return Point(self.x * multiplier, self.y * multiplier)

    def div(self, divby):
        return Point(self.x/divby,self.y/divby)

    def dist(self, other) -> float:
        return math.dist((self.x,self.y),(other.x,other.y))

    def angle_relative_to(self,other) -> float:
        return math.atan(other.y-self.y/other.x-self.x)

    def to_m(inches):
        m = inches * 0.0254
        return m

    def to_in(m):
        inches = m / 0.0254
        return inches
@dataclasses.dataclass
class Vector:
    angle: float  # Radians
    magnitude: float  # Meters / Sec

    def getY(self):
        return self.magnitude * math.sin(self.angle)

    def getX(self):
        return self.magnitude * math.cos(self.angle)

    def topoint(self) -> Point:
        return Point(self.getX(), self.getY())

    def fromradians(rad):
        return rad * (180 / math.pi)

    def fromdegrees(deg):
        return deg * (math.pi / 180)
class ShooterInfo:
    def __init__(self, distance: float, angle: float, rpm: float):
        self.distance = distance
        self.angle = angle
        self.rpm = rpm
class Model:
    def __init__(self, rpos: Point, gpos: Point, rpm: float):
        self.rpos = rpos
        self.gpos = gpos
        self.rpm = rpm
        self.efficiency_percent = 15.0
        self.wheel_diameter = 0.1016

    def get_vel(self, rpm):
        return (math.pi  * self.wheel_diameter) * (rpm / 60) * (self.efficiency_percent/100.0)

def prunepoints(points: []) -> []:
    s = -1
    for i in range(1, len(points)):
        if points[i].y < points[i - 1].y:
            s = i
            break
    return points[:s]

class ProjectileMotion:
    a_r = 0.0

    def __init__(self, dt: float, drag: bool):
        self.dt = dt
        self.drag = drag
        if drag:
            self.a_r = 0.0015
        else:
            self.a_r = 0.0

    def getpoints(self, vector: Vector, exit: Point):
        n_p = [exit, exit]
        v = vector.topoint()
        t = 0
        theta = vector.angle

        while n_p[-1].y >= 0:

            d_f = (
                self.a_r * ((v.x ** 2) + (v.y ** 2))
            )

            a = Point(-d_f * math.cos(theta), (-9.8) + (-d_f * math.sin(theta)))
            v = Point(
                (a.x * self.dt) + v.x, (a.y * self.dt) + v.y
            )
            p = Point(n_p[-1].x + (v.x * self.dt), n_p[-1].y + (v.y * self.dt))

            theta = math.atan(p.y - n_p[-1].y/p.x - n_p[-1].x)

            n_p.append(p)
            t += self.dt

        return n_p

    def get_travel_time(self, points):
        return len(points) * self.dt

robot_wrist_length = 0.05543307086 #(14.08/2.54)/100  # meters

def getangle(model: Model, pm: ProjectileMotion):
    vel = model.get_vel(model.rpm)
    closest_angle = -1
    local_min = 10000
    final_min = 10000

    min_angle = 0
    max_angle = Vector.fromdegrees(61.5)
    angle_change = Vector.fromdegrees(0.1)
    current_angle = min_angle
    while current_angle <= max_angle:
        exitpoint = Point.add(model.rpos, Vector(current_angle, robot_wrist_length).topoint())
        points = pm.getpoints(Vector(current_angle, vel), exitpoint)
        for point in points:
            dist = point.dist(model.gpos)
            local_min = min(local_min, dist)
        if local_min < final_min:
            final_min = local_min
            closest_angle = current_angle
        current_angle += angle_change
    return closest_angle

def plot(line: [], points: []):
    max_axis = Point(5,5)
    min_axis = Point(0,0)
    special_line = line

    for i in range(len(special_line)):
        special_line[i] = (i, special_line[i])
    figure, axis = plt.subplots()
    axis.plot(np.array([pos[1].x for pos in special_line]), np.array([pos[1].y for pos in special_line]))
    for i in points:
        plt.plot(i.x, i.y, "o")

    if len(points) > 0:
        for i in range(len(points)):
            points[i] = (i,points[i])
        newpoints = split_point_list(points)
        newline = split_point_list(line)
        max_axis = Point(max(add_lists([newpoints[0], newline[0]])) + 0.1,max(add_lists([newpoints[1], newline[1]])) + 0.1)
        min_axis =Point(min(add_lists([newpoints[0], newline[0]])) - 0.02,min(add_lists([newpoints[1], newline[1]])) - 0.02)

    plt.axis([min_axis.x,max_axis.x,min_axis.y,max_axis.y])
    axis.set(xlabel="X Postion", ylabel="Y Position", title="Title")
    axis.grid()

    plt.show()

def split_point_list(points_list: []):
    newlist_x = []
    newlist_y = []
    for i in points_list:
        newlist_x.append(i[1].x)
        newlist_y.append(i[1].y)
    return [newlist_x, newlist_y]

def add_lists(list: []):
    newlist = []
    for s in list:
        for i in s:
            newlist.append(i)
    return newlist

def draw_line(points):
    for i in range(1,len(points)):
        plt.plot([points[i-1].x,points[i].x],[points[i-1].y,points[i].y],"k-")

def draw_points(points):
    for i in points:
        plt.plot(i.x, i.y, "o")

def draw_axis(xaxis, yaxis):
    plt.axis([xaxis[0],xaxis[1],yaxis[0],yaxis[1]])

def draw_segment(points):
    draw_points(points)
    draw_line(points)

def results():
    plt.grid()
    plt.show()

# Set proper goal

input_points = [
    {"distance": 1.0, "rpm": 3000}, #1
    {"distance": 2.0, "rpm": 3000}, #2
    {"distance": 2.5, "rpm": 4000},  #3
    {"distance": 3.5, "rpm": 4000},  #4
    {"distance": 4.5, "rpm": 4000},  #5
    {"distance": 5.5, "rpm": 4000},  #6
    {"distance": 6.0, "rpm": 4000},  #7
    {"distance": 6.5, "rpm": 4800},  #8
    {"distance": 8.0, "rpm": 4800}   #9
]

shooting_config = []

i = 1
for input_point in range(len(input_points)):
    rpos = Point(0,0)
    wrist_pos = Point(Point.to_m(1.29),Point.to_m(10.76)) # measure

    rpm = input_points[input_point]["rpm"]
    distance = input_points[input_point]["distance"]
    goalpos = Point(distance, 0.86)
    projectile_motion = ProjectileMotion(0.02, True)
    modelinfo = Model(rpos.add(wrist_pos), goalpos, rpm)
    angle = getangle(modelinfo, projectile_motion)
    modelinfo.rpos = rpos.add(Vector(angle, robot_wrist_length).topoint())
    points = projectile_motion.getpoints(Vector(angle, modelinfo.get_vel(rpm)), modelinfo.rpos)
    time_of_flight = projectile_motion.get_travel_time(points)

    if i == 0:
        plot(points, [modelinfo.rpos, modelinfo.gpos])

    shooting_config.append({"rpm": rpm, "distance": distance, "angle": Vector.fromradians(angle), "time_of_flight": time_of_flight})
    i+=1


shooting_config_file = pathlib.Path("src/main/java/frc/robot/generated/shooting_config.json")
shooting_config_file.write_text(json.dumps(shooting_config, indent=2))


champs_table = [ShooterInfo(distance= 1.38, angle= 58.1, rpm= 3000), #0
                ShooterInfo(distance= 2.16, angle= 47.8, rpm= 3000),
                ShooterInfo(distance= 2.5, angle= 42.0, rpm= 4000),
                ShooterInfo(distance= 3.5, angle= 33.9635, rpm= 4000),
                ShooterInfo(distance= 4.5, angle= 28.20125, rpm= 4000),
                ShooterInfo(distance= 5.5, angle= 25.84825, rpm= 4000),
                ShooterInfo(distance= 6.5, angle= 21.30525, rpm= 4800),
                ShooterInfo(distance= 7.5, angle= 20.27075, rpm= 4800),
                ShooterInfo(distance= 9.0, angle= 18.7305, rpm= 4800) #8
                ]

setgoalposx = 9

for info in champs_table:
    goalpos = Point(setgoalposx, 0.86)
    rpos = Point(setgoalposx-info.distance,0)

    test_model = Model(rpos, goalpos, info.rpm)
    test_pm = ProjectileMotion(0.02, True)
    test_vector = Vector(Vector.fromdegrees(info.angle), test_model.get_vel(info.rpm))
    wrist_vector = Vector(Vector.fromdegrees(info.angle), robot_wrist_length)
    note_exit = wrist_vector.topoint().add(rpos)
    start_duo = [rpos, note_exit]

    points_list = test_pm.getpoints(test_vector, note_exit)
    draw_line(add_lists([start_duo,points_list]))

    draw_points(start_duo)

draw_points([goalpos])

draw_axis([-0.1,9.3],[-0.1,1])

results()


