// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TimedGyroDriveStraightCommand;
import frc.robot.commands.GyroTurnToAngleCommand;

public class SquareAutonomous extends SequentialCommandGroup{
    public SquareAutonomous(){
    addCommands(new TimedGyroDriveStraightCommand(2, 0.2));
    addCommands(new GyroTurnToAngleCommand(90));
    addCommands(new TimedGyroDriveStraightCommand(2, 0.2));
    addCommands(new GyroTurnToAngleCommand(90));
    addCommands(new TimedGyroDriveStraightCommand(2, 0.2));
    addCommands(new GyroTurnToAngleCommand(90));
    addCommands(new TimedGyroDriveStraightCommand(2, 0.2));
    addCommands(new GyroTurnToAngleCommand(90));
    }
}