/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public Joystick driver = new Joystick(0);
  
  // private Button spinner = new JoystickButton(driver, 3); //x
  private Button extendClimber = new JoystickButton(driver, 2); //circle, toggle
  // private Button un_extendClimber = new JoystickButton(driver, 7); //Some arbritary button for now
  private Button extendClimb = new JoystickButton(driver, 3); //triangle
  // private Button un_extendClimb = new JoystickButton(driver, 9); //Some arbritary button for now
  private Button intake = new JoystickButton(driver, 6); //Left trigger
  private Button outtake = new JoystickButton(driver, 7); //Right trigger
  // private Button shooter = new JoystickButton(driver, 4); //y 

  public RobotContainer(){
    // this.spinner.whileHeld(new frc.robot.commands.spinner());
    this.extendClimber.whileHeld(new frc.robot.commands.climberUp());
    // this.un_extendClimber.whileHeld(new frc.robot.commands.climberDown());
    this.extendClimb.whileHeld(new frc.robot.commands.climbUp());
    // this.un_extendClimb.whileHeld(new frc.robot.commands.climbDown());
    this.intake.whileHeld(new frc.robot.commands.Intake());
    this.outtake.whileHeld(new frc.robot.commands.outtake());
    // this.shooter.whileHeld(new frc.robot.commands.shooter());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public double leftY() {
    double leftdrivestick = driver.getRawAxis(1);
    if (Math.abs(leftdrivestick) < 0.05)
      return 0.0;
    else
      return leftdrivestick;
  }

  public double rightY() {
    double rightdrivestick = driver.getRawAxis(3);
    if (Math.abs(rightdrivestick) < 0.05)
      return 0.0;
    else
      return rightdrivestick;
  }

  public double rightX() {
    double joy = driver.getRawAxis(2);
    if (Math.abs(joy) < 0.05)
      return 0;
    else
      return joy;
  }

  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
