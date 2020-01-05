package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class intakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX Motor; 

  // Put in your motors (Victor, Spark, whatnot)
  // private Spark FrontMotor; 
  // private Spark BackMotor;
  
  public intakeSubsystem(){
    if (RobotBase.isReal()) {
    Motor = new VictorSPX(RobotMap.intake);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic(){

  }

  public void intakespeed(double power){
    Motor.set(ControlMode.PercentOutput, power);
  }
}