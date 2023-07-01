// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons.TimedBased;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class PoseSwerveTest extends CommandBase {
  Pose2d currentPose2d;
  Pose2d setPose2d;
  SwerveSubsystem swerveSubsystem;
  /** Creates a new PoseSwerveTest. */
  public PoseSwerveTest(SwerveSubsystem swerveSubsystem, Pose2d setPose2d) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    currentPose2d = new Pose2d(0, 0, swerveSubsystem.getYaw());
    this.setPose2d = setPose2d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
