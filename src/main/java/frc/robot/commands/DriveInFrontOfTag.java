// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveInFrontOfTag extends CommandBase {

  private DriveSubsystem m_driveSubsystem;
  private VisionSubsystem m_visionSubsystem;
  private boolean foundTarget;
  private double tagDistance;
  private double tagZAngle;
  private double distanceToDrive;
  private double angleToDriveIn;
  private double targetDistanceInFrontOfTag;
  private double initialDistance;
  private double initialAngle;
  private DRIVE_STATE driveState;
  private boolean done;

  /** Creates a new DriveInFrontOfTag. */
  public DriveInFrontOfTag(double targetDistanceInFrontOfTag) {
    m_driveSubsystem = Robot.m_driveSubsystem;
    m_visionSubsystem = Robot.m_visionSubsystem;
    this.targetDistanceInFrontOfTag = targetDistanceInFrontOfTag;
    addRequirements(m_driveSubsystem, m_visionSubsystem);
    System.out.println("COMMAND INSTANTIATED");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("COMMAND STARTING");
    driveState = DRIVE_STATE.ROTATING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("COMMAND LOOP");
    if (!foundTarget && m_visionSubsystem.getHasTarget()) {
      Transform3d targetPose = m_visionSubsystem.getBestTarget().getBestCameraToTarget();
      tagDistance = targetPose.getTranslation().getX();
      double reportedTagZAngle = -Math.toDegrees(targetPose.getRotation().getZ());
      tagZAngle = Math.copySign(180 - Math.abs(reportedTagZAngle), reportedTagZAngle);
      double x2d = Math.toDegrees(Math.sin(Math.toRadians(90 - tagZAngle))) / (tagDistance * Math.toDegrees(Math.sin(Math.toRadians(tagZAngle)))); // Sinüs yasasını kullanarak hedeflemek için orijin x çevirisine göre 2d elde edin
      double equationSlope = -tagDistance / x2d; // Apriltag aracılığıyla doğrunun eğiminin denklemini alın
      double targetX = targetDistanceInFrontOfTag * (1 / Math.sqrt(1 + equationSlope * equationSlope)); // vektör denklemini çözerek Y konumunu elde edin
      double targetY = tagDistance + targetDistanceInFrontOfTag * (equationSlope / Math.sqrt(1 + equationSlope * equationSlope)); // Y pozisyonunu al
      distanceToDrive = Math.hypot(targetX, targetY); // Sürüş mesafesini bulmak için hedef X ve Y'nin hipotenüsü
      double division = targetY / targetX;
      angleToDriveIn = 90 - Math.toDegrees(Math.atan(division));
      initialDistance = m_driveSubsystem.getLeftDistance();
      initialAngle = m_driveSubsystem.getAngle();
      foundTarget = true;

      System.out.println("Tag Distance" + tagDistance);
      System.out.println("TagZAngle " + tagZAngle);
      System.out.println("x2d " + x2d);
      System.out.println("equationSlope" + equationSlope);
      System.out.println("Target X" + targetX);
      System.out.println("Target Y " + targetY);
      System.out.println("Distance to drive " + distanceToDrive);
      System.out.println("Division " + division);
      System.out.println("Geater than +- pi/2?" + Math.asin(division));
      System.out.println("degrees " + Math.toDegrees(Math.asin(division)));
      System.out.println("Angle to drive in " + angleToDriveIn);
      System.out.println("Initiail Distance " +initialDistance);
   }

   if (foundTarget) {
    System.out.println("BULUNAN HEDEF");
    switch (driveState) {
      case ROTATING:
        System.out.println("COMMAND ROTATING");
        double error = initialAngle + angleToDriveIn - m_driveSubsystem.getAngle();
        double value = error * Constants.GYRO_TURN_KP; // Motor yüzde gücünü yüzde 1 ile 100 arasında belirlemek için ölçekleme faktörü kp ile çarpın
        if (Math.abs(value) > 0.75) {
          value = Math.copySign(0.75, value);
        }
        if (Math.abs(value) < 0.15) { // Minimum sürüş değeri
          value = Math.copySign(0.15, value);
        }
        m_driveSubsystem.drive(value, -value); // hesaplanan değerleri m_DriveSubsystem'a gönder

        if (error < Constants.DRIVE_TURNING_THRESHOLD_DEGREES) {
          driveState = DRIVE_STATE.DRIVING;
        }
        System.out.println(m_driveSubsystem.getAngle());
        System.out.println("error" + error);
        System.out.println("target" + angleToDriveIn);
        System.out.println("degrees to turn" + angleToDriveIn);
        System.out.println("value" + value);
      break;
      case DRIVING:
        System.out.println("COMMAND DRIVING");
        error = initialDistance + distanceToDrive - m_driveSubsystem.getRightDistance();
        double powerPct = error * Constants.TRACKED_TAG_DISTANCE_DRIVE_KP;
        if (powerPct > Constants.APRILTAG_POWER_CAP) {
          powerPct = Constants.APRILTAG_POWER_CAP;
        } else if (powerPct < -Constants.APRILTAG_POWER_CAP) {
          powerPct = -Constants.APRILTAG_POWER_CAP;
        }
        m_driveSubsystem.drive(powerPct, powerPct);

        if (error < 0.1) {
          done = true;
        }
        System.out.println("ERROR " + error);
        System.out.println("INITIAL DISTANCE: " + initialDistance);
        System.out.println("GOAL DISTANCE: " + (distanceToDrive + initialDistance));
        System.out.println("CURRENT DISTANCE: " + m_driveSubsystem.getRightDistance());
        System.out.println("POWER: " + powerPct);
      break;
      default:
      break;
    }
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("COMMAND ENDED");
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }

  enum DRIVE_STATE {
    ROTATING,
    DRIVING
  }
}
