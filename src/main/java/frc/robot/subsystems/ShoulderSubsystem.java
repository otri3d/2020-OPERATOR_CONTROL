/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PIDController;
import frc.robot.RobotMap;
// import sun.util.resources.cldr.ext.CurrencyNames_mas;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class ShoulderSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX FrontMotor; 
  private VictorSPX BackMotor;

  private PIDController upPID;
  private PIDController downPID;
  private Encoder shoulderEncoder;

  private static final double up_kP = 0.02;
  private static final double up_kI = 0.00;
  private static final double up_kD = 0.08;

  private static final double down_kP = 0.01;
  private static final double down_kI = 0.00;
  private static final double down_kD = 0.00;

  private static final int PulsePerRotation = 128;
  private static final double gearRatio = 198.68/1;
  private static final double EncoderPulsePerRot = PulsePerRotation*gearRatio;

  // Put in your motors (Victor, Spark, whatnot)
  // private Spark FrontMotor; 
  // private Spark BackMotor;
  
  public ShoulderSubsystem(){
    if (RobotBase.isReal()) {
    FrontMotor = new VictorSPX(RobotMap.shoulderFront);
    BackMotor = new VictorSPX(RobotMap.shoulderBack);
    }
    shoulderEncoder = new Encoder(RobotMap.ArmGearBox_A, RobotMap.ArmGearBox_B, false, EncodingType.k4X);
    shoulderEncoder.setDistancePerPulse(EncoderPulsePerRot);

    upPID = new PIDController(up_kP, up_kI, up_kD);
    downPID = new PIDController(down_kP, down_kI, down_kD);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic(){

  }

  public void shoulderGearBox(double power){
    FrontMotor.set(ControlMode.PercentOutput, power);
    BackMotor.set(ControlMode.PercentOutput, power);
  }

  public void rotateUpToSetpoint(double setPoint, double currentValue, double const_multiplier, double tolerance){
    double output = upPID.calcPID(setPoint, currentValue, tolerance);

    shoulderGearBox(output);
  }

  public double encoderDistance(){
    return shoulderEncoder.getDistance();
  }

  public void resetEncoder(){
    shoulderEncoder.reset();
  }

}
