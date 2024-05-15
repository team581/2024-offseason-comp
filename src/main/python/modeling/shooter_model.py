import dataclasses
import math

import numpy as np
import matplotlib.pyplot as plt

@dataclasses.dataclass
class VelocityVector():
  angle: float  # Degrees
  velocity: float  # Meters / Sec

@dataclasses.dataclass
class NotePosition():
  x: float  # Meters
  y: float  # Meters

class ShooterModel:

  def __init__(self, timestep_seconds: int):
    self.timestep_seconds = timestep_seconds


  def graph_note(self, note_exit_vector: VelocityVector):
    print(note_exit_vector.angle)
    print(note_exit_vector.velocity)

    note_position = []
    for t in range(100):
      position = NotePosition((t*(note_exit_vector.velocity * math.cos(note_exit_vector.angle * (math.pi/180)))) + 5,
                              (-(1/2)*9.8*math.pow(t,2)) + (t*(note_exit_vector.velocity * math.sin(note_exit_vector.angle * (math.pi/180)))) + 5)
      note_position.append((t, position))

      if position.y <= 0:
        break


    figure, axis = plt.subplots()
    axis.plot(np.array([pos[1].x for pos in note_position]),
              np.array([pos[1].y for pos in note_position]))

    axis.set(xlabel="X Postion",
             ylabel="Y Position",
             title="Title")
    axis.grid()

    plt.show()



robot_shooter = ShooterModel(0.02)

robot_shooter.graph_note(VelocityVector(60, 100))
