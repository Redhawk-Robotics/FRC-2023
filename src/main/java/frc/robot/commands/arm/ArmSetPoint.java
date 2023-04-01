// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.test.armTest;

public class ArmSetPoint extends CommandBase {
  /** Creates a new ArmSetPoint. */
  // public ArmSubsystem armSubsystem;
  private armTest arm;
  double encoderValue;

  public ArmSetPoint(armTest arm, double encoderValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    // this.armSubsystem = armSubsystem;
    this.arm = arm;
    this.encoderValue = encoderValue;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.armEncoder() > encoderValue) {
      arm.downGoArm();
    } else if (arm.armEncoder() < encoderValue) {
      arm.upGoArm();
    } else {
      arm.stopArm();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(encoderValue - arm.armEncoder()) < 1) {
      arm.stopArm();
      return true;
    }
    return false;
  }
}
