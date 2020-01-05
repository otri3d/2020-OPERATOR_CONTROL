/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  // CAN BUS motor IDS

  // Drivetrain Motors
  public static int leftfront = 1;
  public static int leftback = 2;
  public static int rightfront = 3;
  public static int rightback = 4;

  //Stage 1 Arm (Shoulder) Motors
  public static int shoulderFront = 5;
  public static int shoulderBack = 6;

  // Stage 2 Arm (Wrist) Motors
  public static int wristMotor = 7;

  //Stage 3 Arm (Intake) motors
  public static int intake = 8;

  // encoders (plugged into DIO ports on RoboRIO)
  public static int LeftDrivetrain_A = 0;
  public static int LeftDrivetrain_B = 1;
  public static int RightDriveTrain_A = 2;
  public static int RightDriveTrain_B = 3;
  public static int ArmGearBox_A = 4;
  public static int ArmGearBox_B = 5;
  public static int wrist_A = 6;
  public static int wrist_B = 7;

}
