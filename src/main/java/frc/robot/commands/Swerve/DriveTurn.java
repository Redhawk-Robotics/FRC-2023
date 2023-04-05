// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveTurn extends CommandBase {
  /** Creates a new Drive. */
  private SwerveSubsystem Swerve;
  private double distance, veloicty;
  private Timer timer;
  private double time;
  private double startTime;

  public DriveTurn(
      // Use addRequirements() here to declare subsystem dependencies.
      SwerveSubsystem s_Swerve, double distance, double velocity, double time) {
    this.Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.distance = distance;
    this.veloicty = velocity;
    // time = Math.abs(this.distance / this.veloicty);
    this.time = time;
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // MOVES ONLY IN THE FIELDS X-DIRECTION
    /* Drive */
    Swerve.drive(
        new Translation2d(0, this.veloicty),
        0,
        true,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double velo = this.veloicty;
    double bigStart = this.timer.get();
    // while (this.timer.get() - bigStart < 1 && Math.abs(velo) > 0) {
    // double start = this.timer.get();
    // while (this.timer.get() - start < 0.01) {
    // // pause
    // }
    // Swerve.drive(new Translation2d(velo,0), 0, true, true);
    // if (this.veloicty > 0) {
    // velo -= 3;
    // } else {
    // velo += 3;
    // }

    // }
    Swerve.drive(new Translation2d(0, 0), 0, true, true);
    this.timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.timer.get() > this.time;
    // return Timer.getFPGATimestamp() - this.startTime >= time;
  }

}
