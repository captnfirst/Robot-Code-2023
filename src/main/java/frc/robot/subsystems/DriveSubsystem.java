// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  // Drivetrain Motor Controllers
  PWMTalonSRX mLefTalonSRX;
  PWMTalonSRX mRightTalonSRX;
  
  // Drivetrain Encoders
  Encoder leftDriveEncoder;
  Encoder rightDriveEncoder;

  DifferentialDriveOdometry odometry;
  DifferentialDriveKinematics kinematics;
  DifferentialDriveWheelSpeeds wheelSpeeds;  

  int direction = 1;

  private AHRS navx = new AHRS(SerialPort.Port.kUSB); // NavX GYRO
  private static ADIS16470_IMU imu = new ADIS16470_IMU(); // IMU GYRO

  // İstenen sürücü ölçeğini seçmek için bir seçici oluşturun
  SendableChooser<Double> driveScaleChooser = new SendableChooser<Double>();
  public double CURRENT_DRIVE_SCALE;
    
  public DriveSubsystem() {
    mLefTalonSRX = new PWMTalonSRX(Constants.LEFT_MASTER_DRIVE_MOTOR_ID);
    mRightTalonSRX = new PWMTalonSRX(Constants.RIGHT_MASTER_DRIVE_MOTOR_ID);  

    leftDriveEncoder = new Encoder(2, 3);
    rightDriveEncoder = new Encoder(0, 1, true);  
    leftDriveEncoder.setDistancePerPulse(Constants.WHEEL_CIRCUMFERENCE / Constants.LEFT_ENCODER_COUNTS_PER_REV);
    leftDriveEncoder.setDistancePerPulse(Constants.WHEEL_CIRCUMFERENCE / Constants.RIGHT_ENCODER_COUNTS_PER_REV);

    resetEncoders();

    odometry = new DifferentialDriveOdometry(
      navx.getRotation2d(),
      leftDriveEncoder.getDistance(), 
      rightDriveEncoder.getDistance()
    );
    kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      leftDriveEncoder.getRate(),
      rightDriveEncoder.getRate(),
      Math.toRadians(navx.getRate())
    );
    wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    // Drive Scale Options //
    driveScaleChooser.addOption("100%", 1.0);
    driveScaleChooser.setDefaultOption("75%", 0.75);
    driveScaleChooser.addOption("50%", 0.5);
    driveScaleChooser.addOption("25%", 0.25);

    SmartDashboard.putData("Drivetrain Speed", driveScaleChooser);    
  }

    /* Set power to the drivetrain motors */
    public void drive(double leftPercentPower, double rightPercentPower) {
      mLefTalonSRX.set(direction * leftPercentPower);
      mRightTalonSRX.set(direction * rightPercentPower);
    }
    public void stop() {
      drive(0, 0);
    }

    // NavX Gyroscope Methods //
    public void calibrateGyro() {
      navx.calibrate();
    }
    public void zeroGyro() {
      System.out.println("NavX Connected: " + navx.isConnected());
      navx.reset();
    }
    public double getYaw() {
      return navx.getYaw();
    }
    public double getPitch() {
      return navx.getPitch();
    }
    public double getRoll() {
      return navx.getRoll();
    }
    public double getAngle() {
      return navx.getAngle();
    }    
  
    // IMU Gyroscope Methods //
    public void calibrateIMUGyro() {
      imu.calibrate();
    }
    public void zeroIMUGyro() {
      System.out.println("IMU Connected: " + imu.isConnected());
      imu.reset();
    }
    public double getAccelX() {
      return imu.getAccelX();
    }
    public double getAccelY() {
      return imu.getAccelY();
    }
    public double getAccelZ() {
      return imu.getAccelZ();
    }
    public double getIMUAngle() {
      return imu.getAngle();
    }
  
    // Speed will be measured in meters/second
    public double getLeftSpeed() {
      return leftDriveEncoder.getRate() / 1000; // Milimetreden metreye dönüştürmek için 1000 ile çarpın
    }
  
    public double getRightSpeed() {
      return rightDriveEncoder.getRate() / 1000; // Milimetreden metreye dönüştürmek için 1000 ile çarpın
    }
  
    public double getLeftKinematicsSpeed() {
      return wheelSpeeds.leftMetersPerSecond;
    }
  
    public double getRightKinematicsSpeed() {
      return wheelSpeeds.rightMetersPerSecond;
    }
  
    public DifferentialDriveKinematics getKinematics() {
      return kinematics;
    }
  
    public double getAverageEncoderSpeed() {
      return (getLeftSpeed() + getRightDistance()) / 2;
    }
  
    // Distance will be measured in meters
    public double getLeftDistance() {
      return leftDriveEncoder.getDistance() / 1000; // Milimetreden metreye dönüştürmek için 1000 ile çarpın
    }
    public double getRightDistance() {
      return rightDriveEncoder.getDistance() / 1000; // Milimetreden metreye dönüştürmek için 1000 ile çarpın
    }
  
    public double getAverageEncoderDistance() {
      return (getLeftDistance() + getRightDistance()) / 2;
    }
  
    public void resetOdometry() {
      odometry.resetPosition(
        navx.getRotation2d(), 
        leftDriveEncoder.getDistance(), 
        rightDriveEncoder.getDistance(),
        new Pose2d()
      );
    }
  
    public void resetEncoders() { // Zero the drivetrain encoders
      leftDriveEncoder.reset();
      rightDriveEncoder.reset();
    }
    // These return values are measured in raw encoder counts
    public double getLeftRaw() {
      return leftDriveEncoder.get();
    }
    public double getRightRaw() {
      return rightDriveEncoder.get();
    }
  
    public void toggleDirection() {
      this.direction *= -1;
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      navx.getRotation2d(), 
      leftDriveEncoder.getDistance(), 
      rightDriveEncoder.getDistance()
    );

    CURRENT_DRIVE_SCALE = driveScaleChooser.getSelected(); // İstenen sürücü ölçeğini sürekli güncelleyin
    SmartDashboard.putNumber("Left Drive Encoder", leftDriveEncoder.getRaw()); // Encoder Datasını Shuffleboard aktarma
    SmartDashboard.putNumber("Right Drive Encoder", rightDriveEncoder.getRaw()); // PEncoder Datasını Shuffleboard aktarma
  }
}
