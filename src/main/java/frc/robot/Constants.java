// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) 
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Fiziksel Robot Sabitleri //
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); // Tekerlek Çapı
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // Tekerlerk Çevresi
	  public static final double TRACK_WIDTH = Units.inchesToMeters(21.75); // Sağ ve sol tekerleklerin merkezleri arasındaki mesafe 

    // Joystick Kontrolleri //
    public static final int CONTROLLER_USB_PORT_ID = 0; // Joystick
    public static final int RIGHT_X_JOYSTICK_AXIS = 4;
    public static final int RIGHT_Y_JOYSTICK_AXIS = 3;
    public static final int LEFT_X_JOYSTICK_AXIS = 0;
    public static final int LEFT_Y_JOYSTICK_AXIS = 1;
    public static final int X_BUTTON = 3;
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int LEFT_TRIGGER_AXIS = 9;
    public static final int RIGHT_TRIGGER_AXIS = 10;
    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;

    // Motor CAN IDs //
    //public static final int CUBE_MOTOR_1_ID = 0;

    // Motor PWM Ports //
    public static final int LEFT_MASTER_DRIVE_MOTOR_ID = 0;
    public static final int RIGHT_MASTER_DRIVE_MOTOR_ID = 1;
    
    // PCM (Pneumatics Control Module) Channels //
    //public static final int GRABBER_SOLENOID_ID_1 = 4;
    //public static final int GRABBER_SOLENOID_ID_2 = 5;
    //public static final int LOWER_THE_EXTENDER_ID_1 = 3;
    //public static final int LOWER_THE_EXTENDER_ID_2 = 1;
    
    // DIO (Digital Input/Output) Channels //
    public static final int LED_PWM_ID = 9;

    // Sürücü Sabitleri //
    public static final boolean DRIVE_INVERT_LEFT = false;
    public static final boolean DRIVE_INVERT_RIGHT = true;
    public static final double GYRO_TURN_KP = 0.007;
    public static final double TRACKED_TAG_ROATION_KP = 0.0175;
    public static final double TRACKED_TAG_DISTANCE_DRIVE_KP = 0.3;
    public static final double TRACKED_TAG_AREA_DRIVE_KP = 0.2;
    public static final double APRILTAG_POWER_CAP = 0.75;
    public static final double BEAM_BALANACED_DRIVE_KP = 0.015;
    public static final double BEAM_BALANCED_ANGLE_DEGREES = 0;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
    public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.35;
    public static final double DRIVE_TURNING_THRESHOLD_DEGREES = 3;
    public static final int LEFT_ENCODER_COUNTS_PER_REV = 2048; // Encoderin bir tam devrine eşittir
    public static final int RIGHT_ENCODER_COUNTS_PER_REV = 2048; // Encoderin bir tam devrine eşittir

    // Extender Constants //
    public static final boolean EXTENDER_INVERT = true;
    public static final double EXTENDER_POWER = 0.8;
    public static final double EXTENDER_SETPOINT_INTAKE = 0;
    public static final double EXTENDER_SETPOINT_1 = 50;
    public static final double EXTENDER_SETPOINT_2= 100; 
    public static final double EXTENDER_SETPOINT_3 = 150; 
    public static final double EXTENDER_SETPOINT_4 = 200; 
    public static final double EXTENDER_TOLERANCE = 20.0;  

    // Konum Sabitleri //
    public static final int EXTENDER_TIMEOUT = 0; // ms cinsinden zaman aşımı
	  public static final int EXTENDER_PIDIDX = 0; // PID içindir
	  public static final int EXTENDER_ENCODER_COUNTS_PER_REV = 2048; // Encoderin bir tam devrine eşittir
	  public static final boolean EXTENDER_SENSOR_PHASE = true;
    public static final double EXTENDER_KP = 0.03;

    // Vision Sabitleri //
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(7); // Kamera Yüksekliği 17,78 cm
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(18.5); // Hedefin Yüksekliği 47cm
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(18); // Kamera Derece Aralığı
    public static final String USB_CAMERA_NAME = "USB_Camera-B4.09.24.1"; // Kamera Adı
}