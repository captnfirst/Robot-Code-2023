// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForDistanceCommand;
import frc.robot.commands.DriveInFrontOfTag;
import frc.robot.commands.GyroTurnToAngleCommand;

public class AdvancedAuto extends SequentialCommandGroup {
  /** Bu, gelişmiş bir otonom rutinin nasıl görüneceğine bir örnektir! */
  public AdvancedAuto() {
    addCommands(new DriveInFrontOfTag(0.5)); // Tag'e sür
    addCommands(new DriveForDistanceCommand(-0.4, 0.7)); // Backup to turn
    addCommands(new GyroTurnToAngleCommand(90)); // Pozitif mi negatif mi anlayın, 90'a döner
    addCommands(new DriveForDistanceCommand(0.5, 0.8)); // Yanlara doğru sürer
    addCommands(new GyroTurnToAngleCommand(90)); // Pozitif mi negatif mi anlayın, 90'a döner
    addCommands(new DriveForDistanceCommand(2, 0.8)); // En yakın küpe doğru ilerler
    addCommands(new DriveForDistanceCommand(-2, 0.8)); // Geri dön
    addCommands(new GyroTurnToAngleCommand(-90));
    addCommands(new DriveForDistanceCommand(0.2, 0.5));
    addCommands(new GyroTurnToAngleCommand(-90));
    addCommands(new DriveInFrontOfTag(0.5));
    addCommands(new DriveForDistanceCommand(-0.4, 0.7)); // Backup to turn
    addCommands(new GyroTurnToAngleCommand(180)); //Pozitif mi negatif mi anlayın, 90'a döner
    addCommands(new DriveForDistanceCommand(1, 0.5)); // Yanlara doğru sürer
    addCommands(new BalanceBeamAutonomous());
  }
}
