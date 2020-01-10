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
import frc.robot.PIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  //Name all of the Motor Controllers in the subsystem
  //Example: 
  //private VictorSPX leftFrontMotor;

  //Name all PID controllers used in subsystem
  //Example:
  //private PIDController upPID;
  //private PIDController downPID;

  //initialize final P,I,D,F and tolerance constants

  // private static final double up_kP = 0.02;
  // private static final double up_kI = 0.00;
  // private static final double up_kD = 0.08;
 
  // private static final double down_kP = 0.01;
  // private static final double down_kI = 0.00;
  // private static final double down_kD = 0.00;

  // Identify encoders on subsystem
  // private Encoder climbEncoder;

  //Drivetrain encoder constants
  // private static final int drivewheelRadius = 4;
  // private static final int PulsePerRotation = 128;
  // private static final double gearRatio = 5.85/1;
  // private static final double EncoderPulsePerRot = PulsePerRotation*gearRatio;

  public ClimberSubsystem() {
    //initalize motor controllers according to Constant.java constants
    //Example:
    //leftFrontMotor = new VictorSPX(Constants.leftFront);

    //initialize encoders
    // Example:
    // climbEncoder = new Encoder(Constants.climber);
    // climbEncoder.setDistancePerPulse(EncoderPulsePerRot);

    //initialize PID Controllers
    //Example:
    //upPID = new PIDController(up_kP,up_kI,up_kD);
    //downPID = new PIDController(down_kP,down_kI,down_kD);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    // methods to set motor speed in driver control
    // public void climberGearBox(double power) {
    //   leftFrontMotor.set(ControlMode.PercentOutput, power);
    // }
  
    // public double encoderDistance(){
    //   return climbEncoder.getDistance();
    // }

  // public void resetEncoders(){
  // climbEncoder.reset()
  // }
}
