import dataclasses
import math

import numpy as np
import matplotlib.pyplot as plt

import pathlib
import json


@dataclasses.dataclass
class Point:
    x: float  # Meters
    y: float  # Meters

    def add(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def minus(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def dist(self, other) -> float:
        return math.dist((self.x,self.y),(other.x,other.y))


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


class Model:
    def __init__(self, rpos: Point, gpos: Point, rpm: float):
        self.rpos = rpos
        self.gpos = gpos
        self.rpm = rpm
        self.efficiency = 0.8

    def get_vel(self, rpm):
        return (math.pi * self.efficiency * 0.05 / 30) * rpm

    def getvector(self) -> Vector:
        return Vector(self.get_angle, self.get_vel(self.rpm))


def prunepoints(points: []) -> []:
    s = -1
    for i in range(1, len(points)):
        if points[i].y < points[i - 1].y:
            s = i
            break
    return points[:s]


class ProjectileMotion:
    g = 9.8
    air_resistance = 0.0

    def __init__(self, dt: float, drag: bool):
        self.dt = dt
        self.drag = drag
        if drag:
            self.air_resistance = -0.0015
        else:
            self.air_resistance = 0.0

    def getpoints(self, vector: Vector, exit: Point):
        note_position = [exit, exit]
        note_velocity = [vector.topoint(), vector.topoint()]
        t = 0
        angle = vector.angle
        while note_position[-1].y >= 0:

            drag_force = (
                -1 * self.air_resistance * (math.pow(note_velocity[-1].x, 2) + math.pow(note_velocity[-1].y, 2))
            )
            acceleration = Point(drag_force * math.cos(angle), (-9.8) + (drag_force * math.sin(angle)))
            velocity = Point(
                acceleration.x * self.dt + note_velocity[-1].x, acceleration.y * self.dt + note_velocity[-1].y
            )
            position = Point(note_position[-1].x + velocity.x * self.dt, note_position[-1].y + velocity.y * self.dt)
            angle = math.atan2(note_position[-1].y - note_position[-2].y, note_position[-1].x - note_position[-2].x)

            note_velocity.append(velocity)
            note_position.append(position)
            t += self.dt

        return note_position

    def get_travel_time(self, points):
        return len(points) / self.dt



robot_wrist_length = 0.1  # meters


def getangle(model: Model, pm: ProjectileMotion):
    vel = model.get_vel(model.rpm)
    closest_angle = -1
    local_min = 10000
    final_min = 10000

    min_angle = 0
    max_angle = Vector.fromdegrees(61.5)
    angle_change = Vector.fromdegrees(0.2)

    current_angle = min_angle
    while current_angle <= max_angle:
        exitpoint = Point.add(model.rpos, Vector(current_angle, robot_wrist_length).topoint())
        points = prunepoints(pm.getpoints(Vector(current_angle, vel), exitpoint))
        for point in points:
            dist = point.dist(model.gpos)
            local_min = min(local_min, dist)
        if local_min < final_min:
            final_min = local_min
            closest_angle = current_angle
        current_angle += angle_change
    print(Vector.fromradians(closest_angle))
    print(exitpoint)
    return closest_angle


def plot(line: [], points: []):
    for i in range(len(line)):
        line[i] = (i, line[i])
    figure, axis = plt.subplots()
    axis.plot(np.array([pos[1].x for pos in line]), np.array([pos[1].y for pos in line]))
    for i in points:
        plt.plot(i.x, i.y, "o")

    plt.axis([0,4.5,0,4.5])
    axis.set(xlabel="X Postion", ylabel="Y Position", title="Title")
    axis.grid()

    plt.show()


# Set proper goal

input_points = [
    # {"distance": 1.38, "rpm": 3000},
    # {"distance": 2.0, "rpm": 3000},
    # {"distance": 2.5, "rpm": 4000},
    {"distance": 4.0, "rpm": 4000},
    # {"distance": 4.5, "rpm": 4000},
    # {"distance": 6.0, "rpm": 4000},
    # {"distance": 6.5, "rpm": 4800},
    # {"distance": 8.0, "rpm": 4800}
]

shooting_config = []


for input_point in range(len(input_points)):
    rpm = input_points[input_point]["rpm"]
    distance = input_points[input_point]["distance"]
    goalpos = Point(distance, 0.84)
    rpos = Point(0, 0)
    projectile_motion = ProjectileMotion(0.02, True)
    modelinfo = Model(rpos, goalpos, rpm)
    angle = getangle(modelinfo, projectile_motion)
    modelinfo.rpos = rpos.add(Vector(angle, robot_wrist_length).topoint())
    points = projectile_motion.getpoints(Vector(angle, modelinfo.get_vel(rpm)), modelinfo.rpos)
    time_of_flight = projectile_motion.get_travel_time(points)

    plot(points, [modelinfo.rpos, modelinfo.gpos])

    shooting_config.append({"rpm": rpm, "distance": distance, "angle": angle, "time_of_flight": time_of_flight})


shooting_config_file = pathlib.Path("src/main/java/frc/robot/generated/shooting_config.json")
shooting_config_file.write_text(json.dumps(shooting_config, indent=2))


# goal = Point(0,0.86)
# robotPosition = Point(1,0)
# setrpm = 4000
# info = Model(robotPosition,goal,setrpm)
# robot_shooter = ProjectileMotion(0.02, true)
# setangle = getangle(info,robot_shooter)


# plot(robot_shooter.getpoints(Vector((setangle*(math.pi/180)),info.get_vel(setrpm)),info.rpos), info.rpos,info.gpos,robot_shooter.dt)
