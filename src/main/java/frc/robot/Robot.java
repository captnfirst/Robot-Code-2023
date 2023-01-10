// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BalanceOnBeamCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveInFrontOfTag;
import frc.robot.commands.autonomous.AutonomousMode_Default;
import frc.robot.commands.autonomous.BalanceBeamAutonomous;
import frc.robot.commands.autonomous.Drive1MeterAuto;
import frc.robot.commands.autonomous.PlaceCubeAutonomous;
import frc.robot.commands.autonomous.SquareAutonomous;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

	SendableChooser<CommandBase> autonChooser = new SendableChooser<CommandBase>(); // Otonom bir komut seçmek için bir seçici oluşturun

  public static SendableChooser<Boolean> toggleExtenderPID = new SendableChooser<Boolean>(); // Extender varsayılan komutunun çalışıp çalışmayacağını değiştirmek için bir seçici oluşturun

  public static final Joystick controller = new Joystick(Constants.CONTROLLER_USB_PORT_ID); // Joystick etkileşimi için tanımlama 

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem(); // Drivetrain subsystem
  public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem(); // Photonvision ile etkileşim için subsystem
  public static final LedSubsystem m_LEDSubsystem = new LedSubsystem(); // Subsytem for controlling the LED module



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    configureButtonBindings(); // Configure the button bindings

    // Add our Autonomous Routines to the chooser //
		autonChooser.setDefaultOption("Default Auto", new AutonomousMode_Default());
		autonChooser.addOption("Balance Beam Auto", new BalanceBeamAutonomous());
		autonChooser.addOption("Place Object Auto", new PlaceCubeAutonomous());
    autonChooser.addOption("Square Auto", new SquareAutonomous());
    autonChooser.addOption("Drive 1 Meter", new Drive1MeterAuto());
		SmartDashboard.putData("Auto Mode", autonChooser);

    // Add chooser options for toggling the Extender default command on/off //
    toggleExtenderPID.setDefaultOption("OFF", false);
    toggleExtenderPID.addOption("ON", true);

    SmartDashboard.putData("Extender PID Control", toggleExtenderPID);

    m_driveSubsystem.setDefaultCommand(new DriveCommand());
    //m_extenderSubsystem.setDefaultCommand(new ExtenderControlCommand());

    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    System.out.println("AUTONOMOUS MODE STARTED");

    m_autonomousCommand = autonChooser.getSelected();
    
    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();

    // schedule the selected autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();
    m_LEDSubsystem.lightTheLED(0, 255, 0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void configureButtonBindings() {
    // Drivetrain Controls //
    new Trigger(() -> controller.getRawButton(Constants.Y_BUTTON)).onTrue(new InstantCommand(() -> m_driveSubsystem.toggleDirection()));
    new Trigger(() -> controller.getRawButton(Constants.X_BUTTON)).whileTrue(new BalanceOnBeamCommand());
    new Trigger(() -> controller.getRawButton(Constants.B_BUTTON)).whileTrue(new DriveInFrontOfTag(0.3));
  }
}
