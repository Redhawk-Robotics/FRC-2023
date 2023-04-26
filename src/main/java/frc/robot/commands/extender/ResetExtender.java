// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.extenderSubsystem;

public class ResetExtender extends CommandBase {
  private extenderSubsystem extenderSubsystem;
  private double targetPosition;

  /** Creates a new ResetExtender. */
  public ResetExtender(extenderSubsystem extenderSubsystem, double targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extenderSubsystem = extenderSubsystem;
    this.targetPosition = targetPosition;
    addRequirements(extenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (extenderSubsystem.getCurrentPosition() < targetPosition) {
      extenderSubsystem.setMotor(1);
    } else if (extenderSubsystem.getCurrentPosition() > targetPosition) {
      extenderSubsystem.setMotor(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("ResetExtender ended");
    extenderSubsystem.setPosition(targetPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(extenderSubsystem.getCurrentPosition() - targetPosition) < 1.5;
  }
}
