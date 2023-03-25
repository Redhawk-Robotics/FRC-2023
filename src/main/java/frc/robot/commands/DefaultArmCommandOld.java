// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants;
import frc.robot.constants.Ports;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.test.testClawMotors;

public class DefaultArmCommandOld extends CommandBase {

  private testClawMotors driveSubsystem;

  // private XboxController controller = new XboxController(Ports..CONTROLLER_1_PORT);

  private double left, right;

  SlewRateLimiter leftRateLimiter = new SlewRateLimiter(1);
  SlewRateLimiter rightRateLimiter = new SlewRateLimiter(1);


  /** Creates a new DefaultDemandCommand. */
  public DefaultArmCommandOld(testClawMotors drive) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveSubsystem = drive;
    // this.controller = con;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    left = 0;
    right = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setSpeed(leftRateLimiter.calculate(left), rightRateLimiter.calculate(right));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
