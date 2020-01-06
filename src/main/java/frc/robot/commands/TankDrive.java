/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class TankDrive extends CommandBase {

  private static final double DELTA_LIMIT = 0.75;
  private static final double RAMP_UP_CONSTANT = 0.05;
  private static final double RAMP_DOWN_CONSTANT = 0.05;

  double deltaL = 0;
  double deltaR = 0;
  double prevInputL = 0;
  double prevInputR = 0;
  double inputR = 0;
  double inputL = 0;

  public TankDrive() {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(Robot.dt);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    inputL = Robot.m_robotContainer.leftY() * -1;
    inputR = Robot.m_robotContainer.rightY();
    deltaL = inputL - prevInputL;
    deltaR = inputR - prevInputR;

    if (deltaL >= DELTA_LIMIT){
      inputL += RAMP_UP_CONSTANT;
    }
    else if (deltaL<= -DELTA_LIMIT){
      inputL -= RAMP_DOWN_CONSTANT;
    }

    if (deltaR >= DELTA_LIMIT){
      inputR += RAMP_UP_CONSTANT;
    }
    else if (deltaR<= -DELTA_LIMIT){
      inputR -= RAMP_DOWN_CONSTANT;
    }

    //These will have to be determined based on Constants.java for 2020 bot
    // Robot.dt.leftGearBox(inputL);
    // Robot.dt.rigthGearBox(inputR);

    prevInputL = inputL;
    prevInputR = inputR;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
