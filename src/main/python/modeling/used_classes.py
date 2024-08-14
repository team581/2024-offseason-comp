import dataclasses
import math

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

@dataclasses.dataclass
class ShooterInfo:
    distance: float
    angle: float
    rpm: float

    def get_table_floor(self):
        champs_table_floor = [ShooterInfo(distance= 0.0 , angle= 58.1, rpm= 1000.0), #0
                        ShooterInfo(distance= 1.0, angle= 47.8, rpm= 1000.0),
                        ShooterInfo(distance= 1.2, angle= 42.0, rpm= 1500.0),
                        ShooterInfo(distance= 3.0, angle= 33.9635, rpm= 1800.0),
                        ShooterInfo(distance= 5.8, angle= 28.20125, rpm= 2700.0),
                        ShooterInfo(distance= 6.5, angle= 25.84825, rpm= 2700.0),
                        ShooterInfo(distance= 500.0, angle= 21.30525, rpm= 2700.0),
                        ShooterInfo(distance= 581.0, angle= 20.27075, rpm= 3200.0)
                        ]
        return champs_table_floor
    def get_speaker_table(self):
        champs_table_speaker = [ShooterInfo(distance= 1.38, angle= 58.1, rpm= 3000.0), #0
                                ShooterInfo(distance= 2.16, angle= 47.8, rpm= 3000.0),
                                ShooterInfo(distance= 2.5, angle= 42.0, rpm= 4000.0),
                                ShooterInfo(distance= 3.5, angle= 33.9635, rpm= 4000.0),
                                ShooterInfo(distance= 4.5, angle= 28.20125, rpm= 4000.0),
                                ShooterInfo(distance= 5.5, angle= 25.84825, rpm= 4000.0),
                                ShooterInfo(distance= 6.5, angle= 21.30525, rpm= 4800.0),
                                ShooterInfo(distance= 7.5, angle= 20.27075, rpm= 4800.0),
                                ShooterInfo(distance= 9.0, angle= 18.7305, rpm= 4800.0) #8
                                ]
        return champs_table_speaker


