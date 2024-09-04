import dataclasses
import math
import numerical_data as nd
import enum


@dataclasses.dataclass
class Point:
    x: float  # Meters
    y: float  # Meters

    def plus(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def minus(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def times(self, multiplier):
        return Point(self.x * multiplier, self.y * multiplier)

    def div(self, divby):
        return Point(self.x / divby, self.y / divby)

    def pow(self, pow):
        return Point(self.x**pow, self.y**pow)

    def sqrt(self):
        return Point(math.sqrt(self.x), math.sqrt(self.y))

    def dist(self, other) -> float:
        return math.sqrt(math.pow(self.x - other.x, 2) + math.pow(self.y - other.y, 2))

    def angle_relative_to(self, other) -> float:
        return math.atan(other.y - self.y / other.x - self.x)

    def to_m(inches):
        m = inches * 0.0254
        return m

    def to_in(m):
        inches = m / 0.0254
        return inches

    def from_tuple(tuple: tuple):
        return Point(tuple[0], tuple[1])


@enum.unique
class PruneType(enum.Enum):
    NONE = 1
    PRE_APEX = 2
    POST_APEX = 3


def prune_points(prune_type: PruneType, points: list[Point]):
    if prune_type == PruneType.NONE:
        return points
    s = 0
    while points[s].y <= points[s + 1].y:
        s += 1
    if prune_type == PruneType.PRE_APEX:
        return points[s:]
    if prune_type == PruneType.POST_APEX:
        return points[:s]


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
        self.efficiency_percent = nd._EFFICIENCY
        self.wheel_diameter = nd._FLYWHEEL_DIAMETER

        self.speakerpoint_1 = Point.from_tuple(nd._SPEAKER_POINT_1)
        self.speakerpoint_2 = Point.from_tuple(nd._SPEAKER_POINT_2)
        self.floorspot = Point.from_tuple(nd._FLOOR_SPOT)

    def get_vel(self, rpm):
        return (math.pi * self.wheel_diameter) * (rpm / 60) * (self.efficiency_percent / 100.0)


class ProjectileMotion:
    a_r = 0.0
    t = 0

    def __init__(self, dt: float, drag: bool):
        self.dt = dt
        self.drag = drag
        if drag:
            self.a_r = nd._AIR_RESISTANCE
        else:
            self.a_r = 0.0

    def get_points(self, vector: Vector, exit: Point) -> list[Point]:
        position = [exit]
        velocity = [vector.topoint()]
        acceleration = []
        t = 0
        theta = vector.angle

        while position[-1].y > 0:

            drag_force = self.a_r * ((velocity[-1].x ** 2) + (velocity[-1].y ** 2))

            current_acceleration = Point(x=-drag_force * math.cos(theta), y=(-9.81) + (-drag_force * math.sin(theta)))
            current_velocity = Point(
                x=velocity[-1].x + (self.dt * current_acceleration.x),
                y=velocity[-1].y + (self.dt * current_acceleration.y),
            )
            current_position = Point(
                x=position[-1].x + (self.dt * current_velocity.x), y=position[-1].y + (self.dt * current_velocity.y)
            )

            theta = math.atan(current_position.y - position[-1].y / current_position.x - position[-1].x)

            position.append(current_position)
            velocity.append(current_velocity)
            acceleration.append(current_acceleration)
            t += self.dt
        return position

    def get_travel_time(self, points):
        return len(points) * self.dt


_SHOOTER_OFFSET_TO_ROBOT_CENTER = Point(
    nd._SHOOTER_X_OFFSET_RELATIVE_TO_ROBOT_CENTER, nd._SHOOTER_Y_OFFSET_RELATIVE_TO_ROBOT_CENTER
)


def angle_search(model: Model, pm: ProjectileMotion, prune: PruneType):
    vel = model.get_vel(model.rpm)
    closest_angle = -1
    local_min = 10000
    final_min = 10000

    min_angle = Vector.fromdegrees(nd._MIN_ANGLE)
    max_angle = Vector.fromdegrees(nd._MAX_ANGLE)
    angle_change = Vector.fromdegrees(nd._ANGLE_CHANGE)
    current_angle = min_angle
    while current_angle <= max_angle:
        exitpoint = model.rpos.plus(Vector(current_angle, nd._SHOOTER_LENGTH).topoint()).plus(
            _SHOOTER_OFFSET_TO_ROBOT_CENTER
        )
        points = pm.get_points(Vector(current_angle, vel), exitpoint)
        points = prune_points(prune, points)
        for point in points:
            dist_1 = point.dist(model.speakerpoint_1)
            dist_2 = point.dist(model.speakerpoint_2)
            local_min = min(local_min, (dist_1 + dist_2) / 2)
        if local_min < final_min:
            final_min = local_min
            closest_angle = current_angle
        current_angle += angle_change
    return closest_angle
