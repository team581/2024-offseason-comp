// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_map_manager.pathfinding;

import edu.wpi.first.math.geometry.Translation2d;

record CollisionPoint(String label, Translation2d point, double radius) {}
