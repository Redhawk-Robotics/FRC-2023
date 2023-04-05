// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testSetpoint;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.test.extenderTest;

@Deprecated
public class ExtenderSetPoint extends CommandBase {
  /** Creates a new ExtenderSetPoint. */
  private extenderTest extender;
  double encoderValue;

  public ExtenderSetPoint(extenderTest extender, double encoderValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extender = extender;
    this.encoderValue = encoderValue;

    addRequirements(extender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (extender.extenderEncoder() > encoderValue) {
      extender.downExtender();
    } else if (extender.extenderEncoder() < encoderValue) {
      extender.upExtender();
    } else {
      extender.stopExtender();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extender.stopExtender();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(encoderValue - extender.extenderEncoder()) < 1) {
      extender.stopExtender();
      return true;
    }
    return false;
    // return true;
  }
}
