/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command.  You can replace me with your own command.
 */
public class climberDown extends CommandBase {


  public climberDown() {
    // Use requires() here to declare subsystem dependencies
    addRequirements(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Robot.shoulder.shoulderGearBox(-0.4);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (Robot.m_robotContainer.getRawButtonReleased(2)){
      return true;
    }
    else{
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // Robot.shoulder.shoulderGearBox(0); // Stop the shoulder once the button is not pressed
  }
}
