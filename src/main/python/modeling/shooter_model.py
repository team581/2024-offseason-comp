import dataclasses
import math

import numpy as np
import matplotlib.pyplot as plt

@dataclasses.dataclass
class Vector():
  angle: float  # Degrees
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

def prunepoints(points: []) -> Point:
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
    points = []
    t = 0
    note_position = [exit,exit]
    note_velocity = [Point(vector.getX(),vector.getY()), Point(vector.getX(),vector.getY())]

    for i in range(100):

      angle = math.atan2(note_position[-1].x - note_position[-2].x,note_position[-1].y - note_position[-2].y)
      acceleration = Point((-0.0015*(math.pow(note_velocity[-1].x,2) + math.pow(note_velocity[-1].y,2)))*math.cos(angle),(-9.8)+(-0.0015*(math.pow(note_velocity[-1].x,2) + math.pow(note_velocity[-1].y,2))*math.sin(angle)))
      velocity = Point(acceleration.x * self.dt + note_velocity[-2].x, acceleration.y * self.dt + note_velocity[-2].y)
      position = Point(note_position[-2].x + velocity.x * self.dt, note_position[-2].y + velocity.y * self.dt)


      note_velocity.append(velocity)
      note_position.append(position)
      t = t + self.dt

      points.append(position)
      if points[-1].y < 0:
        break
    return note_position



class ShooterModel:
  def __init__(self, dt: float):
    self.dt = dt
  def graph_note(self, model: Model):
    rpos = model.rpos
    gpos = model.gpos
    angle = model.get_angle()
    vel = Point(model.getvector().getX(), model.getvector().getY())

    print(angle * (180/math.pi))

    note_position = [rpos,rpos]
    note_velocity = [vel, vel]


    for i in range(100):
      t = i * self.dt

      angle = math.atan2(note_position[-1].x - note_position[-2].x,note_position[-1].y - note_position[-2].y)

      acceleration = Point((-0.0015*(math.pow(note_velocity[-1].x,2) + math.pow(note_velocity[-1].y,2)))*math.cos(angle),(-9.8)+(-0.0015*(math.pow(note_velocity[-1].x,2) + math.pow(note_velocity[-1].y,2))*math.sin(angle)))
      velocity = Point(acceleration.x * self.dt + note_velocity[-2].x, acceleration.y * self.dt + note_velocity[-2].y)
      position = Point(note_position[-2].x + velocity.x * self.dt, note_position[-2].y + velocity.y * self.dt)

      note_velocity.append(velocity)
      note_position.append(position)

      if position.y < 0:
        break

    for i in range(len(note_position)):
      note_position[i] = (t,note_position[i])

    figure, axis = plt.subplots()
    axis.plot(np.array([pos[1].x for pos in note_position]),
              np.array([pos[1].y for pos in note_position]))
    plt.plot([rpos.x,gpos.x],[rpos.y,gpos.y],'o')

    axis.set(xlabel="X Postion",
             ylabel="Y Position",
             title="Title")
    axis.grid()

    plt.show()

def getexit(vector: Vector):
  return Point(vector.getX(), vector.getY())


def getangle(model: Model, pm: ProjectileMotion, rpm: float, goal: Point, robot:Point):
  vel = model.get_vel(rpm)
  closestangle = -1
  lineslist = []
  for angle in range(1,85):
    exitpoint = Point.add(robot, getexit(Vector(angle,0.1)))
    points = prunepoints(pm.getpoints(vel, angle, exitpoint))
    lineslist.append(points)
  local_min = 10000
  dist = 10000
  closestdist = []
  angles = []
  for angle in range(len(lineslist)):
    for line in range(len(lineslist[angle][line])):
      dist = lineslist[angle][line].dist(goal)
      local_min = min(local_min, dist)
    closestdist.append(local_min)
    angles.append(angle)
  finalmin = min(closestdist)
  for i in range(len(angles)):
    if closestdist[i] == finalmin:
      closestangle = i
      break
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


goal = Point(1,2)
setangle = -30
robotPosition = Point(0,0)
setrpm = 3500
info = Model(robotPosition,goal,setrpm)

robot_shooter = ProjectileMotion(0.02)

plot(robot_shooter.getpoints(Vector(setangle,info.get_vel(setrpm)),info.rpos) , info.rpos,info.gpos,robot_shooter.dt)
