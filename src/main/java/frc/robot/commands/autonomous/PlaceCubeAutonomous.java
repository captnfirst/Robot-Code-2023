// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.commands.DriveToAprilTagCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Score Pre-Loaded Cube) ******************************************************
 * This is an autonomous routine  for scoring a pre-loaded cube by autonomously driving to the nearest Apriltag */
public class PlaceCubeAutonomous extends SequentialCommandGroup {
  // private ExtenderSubsystem m_extenderSubsystem = Robot.m_extenderSubsystem;
  // GrabberSubsystem m_grabberSubsystem = Robot.m_grabberSubsystem;

  public PlaceCubeAutonomous() { // List commands here sequentially
    addCommands(new DriveToAprilTagCommand(0.5)); //find and drive to aprilTags
  }
}