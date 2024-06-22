import dataclasses
import math

import numpy as np
import matplotlib.pyplot as plt

@dataclasses.dataclass
class Vector():
  angle: float  # Radians
  velocity: float  # Meters / Sec

  def getY(self):
    return self.velocity * math.sin(self.angle)
  def getX(self):
    return self.velocity * math.cos(self.angle)
@dataclasses.dataclass
class Point():
  x: float  # Meters
  y: float  # Meters

  def add(self, other):
    return Point(self.x + other.x, self.y + other.y)

  def dist(self, other) -> float:
    return math.sqrt(math.pow(self.x-other.x,2)+math.pow(self.y-other.y,2))
class Model():
  def __init__(self, rpos: Point, gpos: Point, rpm: float):
    self.rpos = rpos
    self.gpos = gpos
    self.rpm = rpm

  def get_vel(self, rpm):
    return (math.pi * 0.5 * 0.05 / 30) * rpm

  def get_angle(self):
    g = 9.8
    gpos = self.gpos
    rpos = self.rpos
    rpm = self.rpm

    vel = self.get_vel(rpm)

    b = gpos.x-rpos.x
    a = -(g/2)*(math.pow(b,2)/math.pow(vel,2))
    c = (rpos.y-gpos.y)+a
    calculated_angle = math.atan((-b + math.sqrt(math.pow(b,2) - 4 * a * c))/(2 * a))
    return calculated_angle

  def getvector(self) -> Vector:
    return Vector(self.get_angle,self.get_vel(self.rpm))

def prunepoints(points: []) -> []:
  s = -1
  for i in range(1,len(points)):
    if points[i].y < points[i-1].y:
      s=i
      break
  return points[ :s]
class ProjectileMotion:
  g = 9.8
  def __init__(self, dt: float):
    self.dt = dt

  def getpoints(self, vector: Vector, exit: Point) -> []:
    t = 0
    note_position = [exit,exit]
    note_velocity = [Point(vector.getX(),vector.getY()), Point(vector.getX(),vector.getY())]

    while note_position[-1].y > 0:

      angle = math.atan2(note_position[-1].x - note_position[-2].x,note_position[-1].y - note_position[-2].y)
      acceleration = Point((-0.0015*(math.pow(note_velocity[-1].x,2) + math.pow(note_velocity[-1].y,2)))*math.cos(angle),(-9.8)+(-0.0015*(math.pow(note_velocity[-1].x,2) + math.pow(note_velocity[-1].y,2))*math.sin(angle)))
      velocity = Point(acceleration.x * self.dt + note_velocity[-2].x, acceleration.y * self.dt + note_velocity[-2].y)
      position = Point(note_position[-2].x + velocity.x * self.dt, note_position[-2].y + velocity.y * self.dt)


      note_velocity.append(velocity)
      note_position.append(position)
      t = t + self.dt

    return note_position

def getexit(vector: Vector):
  return Point(vector.getX(), vector.getY())

def getangle(model: Model, pm: ProjectileMotion, rpm: float):
  vel = model.get_vel(rpm)
  closestangle = -1
  local_min = 10000
  finalmin = 10000

  for i in range(0,math.ceil(85*(1/pm.dt))):
    angle = i * (math.pi/180) * pm.dt
    exitpoint = Point.add(model.rpos, getexit(Vector(angle,0.1)))
    points = prunepoints(pm.getpoints(Vector(angle,vel), exitpoint))
    for point in range(len(points)):
      dist = points[point].dist(model.gpos)
      local_min = min(local_min, dist)
    finalmin = min(local_min,finalmin)
    if finalmin <= local_min:
      closestangle = angle
  print(finalmin)
  print(closestangle)
  return closestangle

def plot(points: [], start: Point, end: Point, dt: float):
  for i in range(len(points)):
      points[i] = (i * dt,points[i])
  figure, axis = plt.subplots()
  axis.plot(np.array([pos[1].x for pos in points]),
              np.array([pos[1].y for pos in points]))
  plt.plot([start.x,end.x],[start.y,end.y],'o')

  axis.set(xlabel="X Postion",
             ylabel="Y Position",
             title="Title")
  axis.grid()

  plt.show()

goal = Point(1,1)
robotPosition = Point(0,0)
setrpm = 4000
info = Model(robotPosition,goal,setrpm)
robot_shooter = ProjectileMotion(0.02)
setangle = getangle(info,robot_shooter,setrpm)

plot(robot_shooter.getpoints(Vector((setangle*(math.pi/180)),info.get_vel(setrpm)),info.rpos), info.rpos,info.gpos,robot_shooter.dt)
