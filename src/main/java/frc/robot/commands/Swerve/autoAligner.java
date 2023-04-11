// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class autoAligner extends CommandBase {
  /** Creates a new autoAligner. */
  private SwerveSubsystem Swerve;
  private Limelight Light;
  private double headingError;
  private double steering_adjust;
  private double trackingKp;
  private double min_command;
  private double allowedError;
  

  public autoAligner(SwerveSubsystem Swerve, Limelight Light) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Swerve = Swerve;
    this.Light = Light;

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").setInteger(0);

    headingError = 0.0;
    steering_adjust = 0.0;
    trackingKp = -0.065;
    min_command = -0.15;
    allowedError = 1.5;

    addRequirements(Swerve, Light);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Light.getX() > allowedError) {

      steering_adjust = trackingKp * headingError - min_command;

    } else if (Light.getX() < allowedError) {

      steering_adjust = trackingKp * headingError + min_command;

    } else {
     
    Swerve.drive(new Translation2d(0,0), steering_adjust, true, true);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Swerve.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
