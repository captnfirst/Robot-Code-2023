// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.commands.BalanceOnBeamCommand;
import frc.robot.commands.TimedGyroDriveStraightCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Self-Balancing) ******************************************************
 * Bu, şarj istasyonuna sürmek ve ardından geri bildirim olarak jiroskop perdesini kullanarak kendi kendini dengelemek için otonom bir rutindir. */
public class BalanceBeamAutonomous extends SequentialCommandGroup {
  
  public BalanceBeamAutonomous () { // List commands here sequentially
    addCommands(new TimedGyroDriveStraightCommand(1, .2));

    addCommands(new BalanceOnBeamCommand());
  }
}