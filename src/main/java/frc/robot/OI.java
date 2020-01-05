/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ShoulderDown;
import frc.robot.commands.ShoulderUp;
import frc.robot.commands.WristDown;
import frc.robot.commands.WristUp;
import frc.robot.commands.intake;
import frc.robot.commands.outtake;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public Joystick driver = new Joystick(0);

  private Button shoulderUp = new JoystickButton(driver, 5); // 5 (left bumper)
  private Button shoulderDown = new JoystickButton(driver, 6); // 6 (right bumper)
  private Button wristUp = new JoystickButton(driver, 4); // 4 (Y)
  private Button wristDown = new JoystickButton(driver, 1); // 1 (A)
  private Button intake = new JoystickButton(driver, 3); // x
  private Button outtake = new JoystickButton(driver, 2); // b
  private Button allignUsingVision = new JoystickButton(driver, 7);

  public OI() {
    this.shoulderUp.whileHeld(new ShoulderUp());
    this.shoulderDown.whileHeld(new ShoulderDown());
    this.wristUp.whileHeld(new WristUp());
    this.wristDown.whileHeld(new WristDown());
    this.intake.whileHeld(new intake());
    this.outtake.whileHeld(new outtake());

  }

  public double leftY() {
    double leftdrivestick = driver.getRawAxis(1);
    if (Math.abs(leftdrivestick) < 0.05)
      return 0.0;
    else
      return leftdrivestick;
  }

  public double rightY() {
    double rightdrivestick = driver.getRawAxis(5);
    if (Math.abs(rightdrivestick) < 0.05)
      return 0.0;
    else
      return rightdrivestick;
  }

  public double rightX() {
    double joy = driver.getRawAxis(4);
    if (Math.abs(joy) < 0.05)
      return 0;
    else
      return joy;
  }
}
