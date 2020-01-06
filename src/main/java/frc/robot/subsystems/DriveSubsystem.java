/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ArcadeDrive;
import frc.robot.PIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;


public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Identify names of all motor controllers in the subsystem
  private VictorSPX leftfrontMotor;
  private VictorSPX leftbackMotor;
  private VictorSPX rightfrontMotor;
  private VictorSPX rightbackMotor;

  // Identify all PID controllers used in the subsystem
  // Drivetrain has gyroPID and drivePID PID controllers
  private frc.robot.PIDController gyroPID;
  private frc.robot.PIDController drivePID;

  // initialize final P,I,D,F, and tolerance constants.
  private static final double kP = 0.03;
  private static final double kI = 0.01;
  private static final double kD = 0.08;
  private static final double kF = 0.00;

  private static final double kToleranceDegrees = 2.0f;

  // NavX gyro
  private AHRS ahrs; 

  // Identify encoders on subsystem
  private Encoder leftDriveEncoder;
  private Encoder rightDrivEncoder;

  //Drivetrain encoder constants
  private static final int drivewheelRadius = 4;
  private static final int PulsePerRotation = 128;
  private static final double gearRatio = 5.85/1;
  private static final double EncoderPulsePerRot = PulsePerRotation*gearRatio;


  public DriveSubsystem() {

    // initialize motor controllers according to RobotMap constants.
    // VictorSPX uses the CANBUS communication so use Pheonix Turner to assign ID before code
    if (RobotBase.isReal()) {
      // motorLeft = new TalonSRX(0);
      // motorRight = new TalonSRX(1);
    
    leftfrontMotor = new VictorSPX(RobotMap.leftfront);
    leftbackMotor = new VictorSPX(RobotMap.leftback);
    rightfrontMotor = new VictorSPX(RobotMap.rightfront);
    rightbackMotor = new VictorSPX(RobotMap.rightback);
  }
    // initalize encoders
    leftDriveEncoder = new Encoder(RobotMap.LeftDrivetrain_A, RobotMap.LeftDrivetrain_B,true, EncodingType.k4X);
    leftDriveEncoder.setDistancePerPulse(EncoderPulsePerRot);

    rightDrivEncoder = new Encoder(RobotMap.RightDriveTrain_A, RobotMap.RightDriveTrain_B, false, EncodingType.k4X);
    rightDrivEncoder.setDistancePerPulse(EncoderPulsePerRot);

	// initalize PID Controllers
    drivePID = new PIDController(kP,kI,kD); 
    gyroPID = new PIDController(kP,kI,kD);

    // try to initialize the NavX gyro 
    try {
      ahrs = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException e){
      DriverStation.reportError("NAVX error" + e.getMessage(), true);
    }

  }

  @Override
  public void initDefaultCommand() {
    // Set default command. We use tank drive but arcade drive is included as well

    //setDefaultCommand(new ArcadeDrive());
     setDefaultCommand(new TankDrive());
  }

  @Override
  public void periodic() {

  }

  // methods to set motor speed in driver control
  public void leftGearBox(double power) {
    leftbackMotor.set(ControlMode.PercentOutput, power);
    leftfrontMotor.set(ControlMode.PercentOutput, power);
  }

  public void rigthGearBox(double power) {
    rightbackMotor.set(ControlMode.PercentOutput, power);
    rightfrontMotor.set(ControlMode.PercentOutput, power);
  }

  // autonomous methods 

  public void DriveStraightNoSensors(double power, double time){
    {
      leftGearBox(-power);
      rigthGearBox(power);
    }
    Timer.delay(time);
  }

  public void turnRight(double power, double time){
    {
      leftGearBox(-power);
      rigthGearBox(0);
    }
    Timer.delay(time);
  }

  public void turnLeft(double power, double time){
    {
      leftGearBox(0);
      rigthGearBox(power);
    }
    Timer.delay(time);
  }

  public void DriveStraightEncodersGyro(double setPoint, double epsilon, double const_multiplier){
    double output = drivePID.calcPID(setPoint, getAverageDistance(), epsilon);

    leftGearBox((output - Pitch()*kP)*const_multiplier);
    leftGearBox((output + Pitch()*kP)*const_multiplier);
  }

  public void turnToAngle(double setPoint, double currentValue, double const_multiplier ){
    double output = gyroPID.calcPID(setPoint, currentValue, 1.5);

    leftGearBox(-output*const_multiplier);
    rigthGearBox(output*const_multiplier);
  }

  // gyro methods
  public double angle(){
    return ahrs.getAngle();
  }
  public double Yaw(){ // z axis
    return ahrs.getYaw();
  }
  public double Pitch(){ // x axis 
    return ahrs.getPitch();
  }
  public double Roll(){ // Y axis
    return ahrs.getRoll();
  }
  public AHRS getAhrs(){
    return ahrs;
  }
  public double rate(){
    return ahrs.getRate();
  }
  public void resetGyro()
  {
    ahrs.reset();
  }

  // encoder methods
  public double getAverageDistance(){
    return drivewheelRadius*(leftDriveEncoder.getDistance() + rightDrivEncoder.getDistance())/2;
  }
  public void resetEncoders(){
    leftDriveEncoder.reset();
    rightDrivEncoder.reset();
  }
}
