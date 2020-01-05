/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PIDController;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class WristSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX WristMotor;

  private PIDController wristPID;
  private Encoder wristEncoder;

  private static final double kP = 0.02;
  private static final double kI = 0.00;
  private static final double kD = 0.08;

  private static final int PulsePerRotation = 128;
  private static final double gearRatio = 150/1;
  private static final double EncoderPulsePerRot = PulsePerRotation*gearRatio;

  // Put in your motors (Victor, Spark, whatnot)
  // private Spark Motor; 

  public WristSubsystem(){
    if (RobotBase.isReal()) {
    WristMotor = new VictorSPX(RobotMap.wristMotor);
    
    wristEncoder = new Encoder(RobotMap.ArmGearBox_A, RobotMap.ArmGearBox_B, false, EncodingType.k4X);
    wristEncoder.setDistancePerPulse(EncoderPulsePerRot);
    }
    wristPID = new PIDController(kP, kI, kD);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic(){

  }

  public void wristMovement(double power) {
	  WristMotor.set(ControlMode.PercentOutput, power);
  }

  public void rotateUpToSetpoint(double setPoint, double currentValue, double const_multiplier, double tolerance){
    double output = wristPID.calcPID(setPoint, currentValue, tolerance);

    wristMovement(output);
  }

}

