/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import frc.robot.Constants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.PIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  //Name all of the Motor Controllers in the subsystem
  //Example: 
  //private VictorSPX leftFrontMotor;

  //Name all PID controllers used in subsystem
  //Example:
  //private frc.robot.PIDController drivePID;

  //initialize final P,I,D,F and tolerance constants

  // private static final double kP = 0.03;
  // private static final double kI = 0.01;
  // private static final double kD = 0.08;
  // private static final double kF = 0.00;

  // private static final double kToleranceDegrees = 2.0f;

  // NavX gyro
  // private AHRS ahrs; 

  // Identify encoders on subsystem
  // private Encoder leftDriveEncoder;
  // private Encoder rightDrivEncoder;

  //Drivetrain encoder constants
  // private static final int drivewheelRadius = 4;
  // private static final int PulsePerRotation = 128;
  // private static final double gearRatio = 5.85/1;
  // private static final double EncoderPulsePerRot = PulsePerRotation*gearRatio;

  public DriveSubsystem() {
    //initalize motor controllers according to Constant.java constants
    //Example:
    //leftFrontMotor = new VictorSPX(Constants.leftFront);

    //initialize encoders
    // Example:
    // leftDriveEncoder = new Encoder(Constants.LeftDrivetrain_A, Constants.LeftDrivetrain_B,true, EncodingType.k4X);
    // leftDriveEncoder.setDistancePerPulse(EncoderPulsePerRot);

    //initialize PID Controllers
    //Example:
    //drivePID = new PIDController(kP,kI,kD);

    //attempt to initialize gyro
    /*
    try {
      ahrs = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException e){
      DriverStation.reportError("NAVX error" + e.getMessage(), true);
    }
    */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    // methods to set motor speed in driver control
    // public void leftGearBox(double power) {
    //   leftbackMotor.set(ControlMode.PercentOutput, power);
    //   leftfrontMotor.set(ControlMode.PercentOutput, power);
    // }
  
    // public void rigthGearBox(double power) {
    //   rightbackMotor.set(ControlMode.PercentOutput, power);
    //   rightfrontMotor.set(ControlMode.PercentOutput, power);
    // }

      // encoder methods
  // public double getAverageDistance(){
  //   return drivewheelRadius*(leftDriveEncoder.getDistance() + rightDrivEncoder.getDistance())/2;
  // }
  // public void resetEncoders(){
  //   leftDriveEncoder.reset();
  //   rightDrivEncoder.reset();
  // }
}
