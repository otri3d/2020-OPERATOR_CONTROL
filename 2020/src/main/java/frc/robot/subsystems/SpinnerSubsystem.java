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

public class SpinnerSubsystem extends SubsystemBase {

  //initialize motor controller for spinner
  // private VictorSPX spinnerMotor;

  //initialize PID and Encoders
  //private PIDController spinnerPID;
  //private Encoder spinnerEncoder;

  //Initialize P,I,D
  // private static final double kP = 0.02;
  // private static final double kI = 0.00;
  // private static final double kD = 0.08;

  //Encoder Constants
  // private static final int PulsePerRotation = 128;
  // private static final double gearRatio = 150/1;
  // private static final double EncoderPulsePerRot = PulsePerRotation*gearRatio;


  public SpinnerSubsystem() {
    //initalize motor controllers according to Constants.java constants
    //Example:
    //FrontMotor = new VictorSPX(Constants.leftFront);

    //initialize encoders
    // Example:
    // leftSpinnerEncoder = new Encoder(RobotMap.LeftDrivetrain_A, RobotMap.LeftDrivetrain_B,true, EncodingType.k4X);
    // leftSpinnerEncoder.setDistancePerPulse(EncoderPulsePerRot);

    //initialize PID Controllers
    //Example:
    //spinnerPID = new PIDController(kP,kI,kD);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // @Override
  // public void spinnerMovement(double power){
  //   FrontMotor.set(ControlMode.PercentOutput, power);
  // }

  // @Override
  // public void rotateUpToSetPoint(double setPoint, double currentValue, double const_multiplier, double tolerance){
  //    double output = spinnerPID.calcPID(setPoint, currentValue, tolerance);
  //    spinnerMovement(output);
  // }

}
