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
  def __init__(self, rpos: Position, gpos: Position, rpm: float):
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
    calculated_angle = math.atan((-b - math.sqrt(math.pow(b,2) - 4 * a * c))/(2 * a))
    return calculated_angle


class ShooterModel:

  def __init__(self, dt: int):
    self.dt = dt


  def graph_note(self, model: Model):
    rpos = model.rpos
    gpos = model.gpos
    angle = model.get_angle()
    vel = Position(model.get_vel(model.rpm) * math.cos(angle), model.get_vel(model.rpm) * math.sin(angle))

    print(angle * (180/math.pi))

    note_position = [(0, rpos),(0, rpos)]
    note_velocity = [vel, vel]


    for i in range(100):
      t = i * self.dt


      print(note_position)
      angle = math.atan2(note_position[-1].x - note_position[-2].x,note_position[-1].y - note_position[-2].y)

      acceleration = Position((-0.0015*(math.pow(note_velocity[-1].x,2) + math.pow(note_velocity[-1].y,2)))*math.cos(angle),(-9.8)+(-0.0015*(math.pow(note_velocity[-1].x,2) + math.pow(note_velocity[-1].y,2))*math.sin(angle)))
      velocity = Position(acceleration.x * self.dt + note_velocity[-2].x, acceleration.y * self.dt + note_velocity[-2].y)
      position = Position(note_position[-2].x + velocity.x * self.dt, note_position[-2].y + velocity.y * self.dt)

      note_velocity.append(velocity)
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

robot_shooter.graph_note(Model(Position(1,0),Position(2,2),3500))
