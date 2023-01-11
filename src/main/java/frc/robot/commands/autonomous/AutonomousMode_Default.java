// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.commands.TimedGyroDriveStraightCommand;
import frc.robot.commands.GyroTurnToAngleCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * Bu bizim en temel rutinimizdir. 2 saniye ileri gider, 180 derece döner ve sonra geri gider. */
public class AutonomousMode_Default extends SequentialCommandGroup {

  // List commands here sequentially
  public AutonomousMode_Default() { // List commands here sequentially
    addCommands(new TimedGyroDriveStraightCommand(2, 0.3));

    addCommands( new GyroTurnToAngleCommand(180));

    addCommands(new TimedGyroDriveStraightCommand(2, 0.3));
  }
}