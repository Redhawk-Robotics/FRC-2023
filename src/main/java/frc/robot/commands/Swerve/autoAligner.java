// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class autoAligner extends CommandBase {
  /** Creates a new autoAligner. */
  private SwerveSubsystem Swerve;
  private Limelight Light;

  public autoAligner(SwerveSubsystem Swerve, Limelight Light)  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Swerve = Swerve;
    this.Light = Light;

    addRequirements(Swerve, Light);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
