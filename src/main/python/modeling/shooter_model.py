import dataclasses
import math

import numpy as np
import matplotlib.pyplot as plt

import pathlib
import json

robot_wrist_length = 0.05543307086 #(14.08/2.54)/100  # meters
speakerheight = 2.1844 #meters

@dataclasses.dataclass
class Point:
    x: float # Meters
    y: float # Meters

    def plus(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def minus(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def times(self, multiplier):
        return Point(self.x * multiplier, self.y * multiplier)

    def div(self, divby):
        return Point(self.x/divby,self.y/divby)

    def pow(self, pow):
        return Point(self.x ** pow, self.y ** pow)

    def sqrt(self):
        return Point(math.sqrt(self.x),math.sqrt(self.y))

    def dist(self, other) -> float:
        return math.sqrt(
            math.pow(self.x - other.x, 2) + math.pow(self.y - other.y, 2)
        )

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

    

champs_table_floor = [ShooterInfo(distance= 0.0 , angle= 58.1, rpm= 1000.0), #0
                        ShooterInfo(distance= 1.0, angle= 47.8, rpm= 1000.0),
                        ShooterInfo(distance= 1.2, angle= 42.0, rpm= 1500.0),
                        ShooterInfo(distance= 3.0, angle= 33.9635, rpm= 1800.0),
                        ShooterInfo(distance= 5.8, angle= 28.20125, rpm= 2700.0),
                        ShooterInfo(distance= 6.5, angle= 25.84825, rpm= 2700.0),
                        ShooterInfo(distance= 500.0, angle= 21.30525, rpm= 2700.0),
                        ShooterInfo(distance= 581.0, angle= 20.27075, rpm= 3200.0)
                        ]
class Model:
    def __init__(self, rpos: Point, gpos: Point, rpm: float):
        self.rpos = rpos
        self.gpos = gpos
        self.rpm = rpm
        self.efficiency_percent = 90.0
        self.wheel_diameter = 0.1016

        self.speakerpoint_1 = gpos
        self.speakerpoint_2 = Point(gpos.x - 0.43,gpos.y - Point.to_m(12))

    def get_vel(self, rpm):
        return (math.pi  * self.wheel_diameter) * (rpm / 60) * (self.efficiency_percent/100.0)
class ProjectileMotion:
    a_r = 0.0
    t = 0

    def __init__(self, dt: float, drag: bool):
        self.dt = dt
        self.drag = drag
        if drag:
            self.a_r = 0.0015
        else:
            self.a_r = 0.0

    def get_points(self, vector: Vector, exit: Point) -> list[Point]:
        position = [exit]
        velocity = [vector.topoint()]
        acceleration = []
        t = 0
        theta = vector.angle

        while position[-1].y > 0:

            drag_force = (
                self.a_r * ((velocity[-1].x ** 2) + (velocity[-1].y ** 2))
            )

            current_acceleration = Point(x=-drag_force*math.cos(theta),
                                         y=(-9.81) + (-drag_force * math.sin(theta)))
            current_velocity = Point(x=velocity[-1].x + (self.dt * current_acceleration.x),
                                     y=velocity[-1].y + (self.dt * current_acceleration.y))
            current_position = Point(x=position[-1].x + (self.dt * current_velocity.x),
                                     y=position[-1].y + (self.dt * current_velocity.y))

            theta = math.atan(current_position.y - position[-1].y/current_position.x - position[-1].x)

            position.append(current_position)
            velocity.append(current_velocity)
            acceleration.append(current_acceleration)
            t += self.dt
        return position

    def get_travel_time(self, points):
        return len(points) * self.dt

#///////////////////////////////////////////////////////////

def get_angle(model: Model, pm: ProjectileMotion):
    vel = model.get_vel(model.rpm)
    closest_angle = -1
    local_min = 10000
    final_min = 10000

    min_angle = 0
    max_angle = Vector.fromdegrees(61.5)
    angle_change = Vector.fromdegrees(0.1)
    current_angle = min_angle
    while current_angle <= max_angle:
        exitpoint = Point.plus(model.rpos, Vector(current_angle, robot_wrist_length).topoint())
        points = pm.get_points(Vector(current_angle, vel), exitpoint)
        for point in points:
            dist = point.dist(model.gpos)
            local_min = min(local_min, dist)
        if local_min < final_min:
            final_min = local_min
            closest_angle = current_angle
        current_angle += angle_change
    return closest_angle

def get_angle_better(model: Model, pm: ProjectileMotion):
    vel = model.get_vel(model.rpm)
    closest_angle = -1
    local_mean = 10000
    final_min = 10000

    min_angle = 0
    max_angle = Vector.fromdegrees(61.5)
    angle_change = Vector.fromdegrees(0.1)
    current_angle = min_angle
    while current_angle <= max_angle:
        exitpoint = Point.plus(model.rpos, Vector(current_angle, robot_wrist_length).topoint())
        points = pm.get_points(Vector(current_angle, vel), exitpoint)
        for point in points:
            dist_1 = point.dist(model.speakerpoint_1)
            dist_2 = point.dist(model.speakerpoint_2)
            local_mean = min(local_mean, (dist_1 + dist_2)/2)
        if local_mean < final_min:
            final_min = local_mean
            closest_angle = current_angle
        current_angle += angle_change
    return closest_angle

def get_angle_floor(model: Model, pm: ProjectileMotion):
    vel = model.get_vel(model.rpm)
    closest_angle = -1
    local_min = 10000
    final_min = 10000

    min_angle = 0
    max_angle = Vector.fromdegrees(61.5)
    angle_change = Vector.fromdegrees(0.1)
    current_angle = min_angle
    while current_angle <= max_angle:
        exitpoint = Point.plus(model.rpos, Vector(current_angle, robot_wrist_length).topoint())
        points = pm.get_points(Vector(current_angle, vel), exitpoint)
        for point in points:
            dist = point.dist(model.gpos)
            local_min = min(local_min, dist)
        if local_min < final_min:
            final_min = local_min
            closest_angle = current_angle
        current_angle += angle_change
    return closest_angle

#//////////////////////////////////////////////////////////

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

def split_point_list(points_list: list[Point]):
    newlist_x = []
    newlist_y = []
    for i in points_list:
        newlist_x.append(i[1].x)
        newlist_y.append(i[1].y)
    return [newlist_x, newlist_y]

def add_lists(list: list):
    newlist = []
    for s in list:
        for i in s:
            newlist.append(i)
    return newlist

def draw_line(points):
    for i in range(1,len(points)):
        plt.plot([points[i-1].x,points[i].x],[points[i-1].y,points[i].y],"k-")

def draw_line_red(points):
    for i in range(1,len(points)):
        plt.plot([points[i-1].x,points[i].x],[points[i-1].y,points[i].y],"r-")

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

#//////////////////////////////////////////////////////////////////////

def test_champ_table(drag:bool):

    setgoalposx = 9

    for info in champs_table_speaker:
        goalpos = Point(setgoalposx, speakerheight)
        rpos = Point(setgoalposx-info.distance,0)

        test_model = Model(rpos, goalpos, info.rpm)

        test_pm = ProjectileMotion(0.02, drag)
        test_vector = Vector(Vector.fromdegrees(info.angle), test_model.get_vel(info.rpm))
        wrist_vector = Vector(Vector.fromdegrees(info.angle), robot_wrist_length)
        note_exit = wrist_vector.topoint().plus(rpos)
        start_duo = [rpos, note_exit]

        points_list = test_pm.get_points(test_vector, note_exit)

        draw_line(add_lists([start_duo,points_list]))
        draw_points(start_duo)
    draw_points([goalpos])

    draw_axis([-0.1,10],[-0.1,10])

    draw_line_red([Point(goalpos.x + 0.229997,0), Point(goalpos.x + 0.229997,1.984502)])
    draw_line_red([Point(goalpos.x - 0.923798,1.984502), Point(goalpos.x ,2.492502)])

    results()

def test_anglesearch(drag:bool):
    for info in champs_table_speaker:
        gpos = Point(9,speakerheight)
        rpos = Point(9-info.distance,0)

        model = Model(rpos, gpos, info.rpm)
        pm = ProjectileMotion(0.02, drag)
        vector = Vector(get_angle(model,pm), model.get_vel(info.rpm))
        exit2 = Vector(vector.angle, robot_wrist_length).topoint().plus(rpos)
        points = pm.get_points(vector,exit2)

        draw_points([rpos,exit2])
        draw_line(add_lists([[rpos,exit2],points]))
    draw_points([gpos])

    draw_axis([-0.1,10],[-0.1,10])

    draw_line_red([Point(gpos.x + 0.229997,0), Point(gpos.x + 0.229997,1.984502)])
    draw_line_red([Point(gpos.x - 0.923798,1.984502), Point(gpos.x ,2.492502)])

    results()

def test_better_anglesearch(drag:bool):
    for info in champs_table_speaker:
        gpos = Point(9,speakerheight)
        rpos = Point(9-info.distance,0)

        model = Model(rpos, gpos, info.rpm)
        pm = ProjectileMotion(0.02, drag)
        vector = Vector(get_angle_better(model,pm), model.get_vel(info.rpm))
        exit2 = Vector(vector.angle, robot_wrist_length).topoint().plus(rpos)
        points = pm.get_points(vector,exit2)

        draw_points([rpos,exit2])
        draw_line(add_lists([[rpos,exit2],points]))
    draw_points([gpos])

    draw_axis([-0.1,10],[-0.1,10])

    draw_line_red([Point(gpos.x + 0.229997,0), Point(gpos.x + 0.229997,1.984502)])
    draw_line_red([Point(gpos.x - 0.923798,1.984502), Point(gpos.x ,2.492502)])

    results()

def generate_speaker_distance_rpm():
    for info in champs_table_speaker:
        rpm = info.rpm
        distance = info.distance
        print(f'speakerDistanceToRPM.put({distance}, {rpm});')

def generate_speaker_distance_angle(drag:bool):
    for info in champs_table_speaker:
        gpos = Point(9,speakerheight)
        rpos = Point(9-info.distance,0)

        model = Model(rpos, gpos, info.rpm)
        pm = ProjectileMotion(0.02, drag)

        distance = info.distance
        angle = round(Vector.fromradians(get_angle_better(model,pm)),4)

        print(f'speakerDistanceToAngle.put({distance}, {angle});')

def generate_floor_distance_rpm():
    for info in champs_table_floor:
        rpm = info.rpm
        distance = info.distance
        print(f'floorSpotDistanceToRPM.put({distance}, {rpm});')

def generate_floor_distance_angle(drag:bool):
    for info in champs_table_floor:
        gpos = Point(9,0)
        rpos = Point(9-info.distance,0)

        model = Model(rpos, gpos, info.rpm)
        pm = ProjectileMotion(0.02, drag)

        distance = info.distance
        angle = round(Vector.fromradians(get_angle_floor(model,pm)),4)

        print(f'floorSpotDistanceToAngle.put({distance}, {angle});')

def generate_file_speaker(drag:bool):
    shooting_config = []

    for info in champs_table_speaker:
        gpos = Point(9,speakerheight)
        rpos = Point(9-info.distance,0)

        model = Model(rpos, gpos, info.rpm)
        pm = ProjectileMotion(0.02, drag)
        vector = Vector(get_angle_better(model,pm), model.get_vel(info.rpm))
        exit2 = Vector(vector.angle, robot_wrist_length).topoint().plus(rpos)
        points = pm.get_points(vector,exit2)

        rpm = info.rpm
        distance = info.distance
        angle = vector.angle
        time_of_flight = pm.get_travel_time(points)

        shooting_config.append({"rpm": rpm, "distance": distance, "angle": round(Vector.fromradians(angle),3), "time_of_flight": round(time_of_flight,2)})

    shooting_config_file = pathlib.Path("src/main/java/frc/robot/generated/speaker_shooting_config.json")
    shooting_config_file.write_text(json.dumps(shooting_config, indent=2))

def generate_file_floor(drag:bool):
    shooting_config = []

    for info in champs_table_speaker:
        gpos = Point(9,0)
        rpos = Point(9-info.distance,0)

        model = Model(rpos, gpos, info.rpm)
        pm = ProjectileMotion(0.02, drag)
        vector = Vector(get_angle_floor(model,pm), model.get_vel(info.rpm))
        exit2 = Vector(vector.angle, robot_wrist_length).topoint().plus(rpos)
        points = pm.get_points(vector,exit2)

        rpm = info.rpm
        distance = info.distance
        angle = vector.angle
        time_of_flight = pm.get_travel_time(points)

        shooting_config.append({"rpm": rpm, "distance": distance, "angle": round(Vector.fromradians(angle),3), "time_of_flight": round(time_of_flight,2)})

    shooting_config_file = pathlib.Path("src/main/java/frc/robot/generated/floor_shooting_config.json")
    shooting_config_file.write_text(json.dumps(shooting_config, indent=2))

# generate_file_speaker()

# test_better_anglesearch(True)
# test_anglesearch(True)
# test_champ_table(True)

# generate_speaker_distance_rpm()
# generate_speaker_distance_angle(True)

# generate_floor_distance_rpm()
# generate_floor_distance_angle(True)

