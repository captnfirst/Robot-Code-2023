// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.commands.DriveForDistanceCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * This basic autonomous routine drives forward 1 meter using encoder feedback */
public class Drive1MeterAuto extends SequentialCommandGroup {

  // List commands here sequentially
  public Drive1MeterAuto() { // List commands here sequentially
    addCommands(new DriveForDistanceCommand(1, 0.3));
  }
}