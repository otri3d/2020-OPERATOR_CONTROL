/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ArcadeDrive extends Command {

private static final double DELTA_LIMIT = 0.75;
private static final double RAMP_UP_CONSTANT = 0.05;
private static final double RAMP_DOWN_CONSTANT = 0.05;

double deltaY =0;
double deltaX = 0;
double prevInputY = 0;
double prevInputX  =0;
double inputY = 0;
double inputX = 0;

  public ArcadeDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.dt);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    inputY = Robot.m_oi.leftY();
    inputX = Robot.m_oi.rightX();
    deltaY = inputY - prevInputY;
    deltaX = inputX - prevInputX;

    if (deltaY >= DELTA_LIMIT){
      inputY += RAMP_UP_CONSTANT;
    }
    else if (deltaY<= -DELTA_LIMIT){
      inputY -= RAMP_DOWN_CONSTANT;
    }

    if (deltaX >= DELTA_LIMIT){
      inputX += RAMP_UP_CONSTANT;
    }
    else if (deltaX<= -DELTA_LIMIT){
      inputX -= RAMP_DOWN_CONSTANT;
    }

    Robot.dt.leftGearBox(-inputY+inputX); // *-1 left
    Robot.dt.rigthGearBox(inputY-inputX);

    prevInputY = inputY;
    prevInputX = inputX;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
