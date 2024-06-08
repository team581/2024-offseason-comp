import dataclasses
import math

import numpy as np
import matplotlib.pyplot as plt

@dataclasses.dataclass
class VelocityVector():
  angle: float  # Degrees
  velocity: float  # Meters / Sec

@dataclasses.dataclass
class Position():
  x: float  # Meters
  y: float  # Meters


class Model():
  def __init__(self, rpos, gpos, vel):
    self.rpos = rpos
    self.gpos = gpos
    self.vel = vel


  def get_angle(self):
    g = 9.8
    gpos = self.gpos
    rpos = self.rpos
    vel = self.vel
    b = gpos.x-rpos.x
    a = -(g/2)*(math.pow(b,2)/math.pow(vel,2))
    c = (rpos.y-gpos.y)+a
    calculated_angle = math.atan((-b - math.sqrt(math.pow(b,2) - 4 * a * c))/(2 * a))
    return calculated_angle


class ShooterModel:

  def __init__(self, timestep_seconds: int):
    self.timestep_seconds = timestep_seconds


  def graph_note(self, model: Model):
    vel = model.vel
    rpos = model.rpos
    gpos = model.gpos
    angle = model.get_angle()

    print(angle * (180/math.pi))

    note_position = []
    for i in range(100):
      t = i * self.timestep_seconds
      position = Position((t*(vel * math.cos(angle)) + rpos.x),
                              (-(1/2)*9.8*math.pow(t,2)) + (t*(vel * math.sin(angle))) + rpos.y)
      note_position.append((t, position))

      if position.y < 0:
        break


    figure, axis = plt.subplots()
    axis.plot(np.array([pos[1].x for pos in note_position]),
              np.array([pos[1].y for pos in note_position]))
    plt.plot([rpos.x,gpos.x],[rpos.y,gpos.y],'o')

    axis.set(xlabel="X Postion",
             ylabel="Y Position",
             title="Title")
    axis.grid()

    plt.show()



robot_shooter = ShooterModel(0.02)

robot_shooter.graph_note(Model(Position(0,0),Position(1,5),10))
