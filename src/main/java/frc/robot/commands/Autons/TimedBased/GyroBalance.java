// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons.TimedBased;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.modules.PigeonModule;
import frc.robot.subsystems.modules.SwerveModule;

public class GyroBalance extends CommandBase {
  private double velocity;
  private PigeonModule pigeonModule;
  private SwerveModule[] SwerveMods;
  private SwerveSubsystem swerveSubsystem;

  /** Creates a new GyroBalance. */
  public GyroBalance(SwerveSubsystem swerveSubsystem) {
    this.pigeonModule = PigeonModule.getPigeonModule();
    this.swerveSubsystem = swerveSubsystem;
    this.SwerveMods = swerveSubsystem.getSwerveModules();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("GYRO ROLL", pigeonModule.getRoll());
    // velocity = pigeonModule.getRoll() > 8 ? 6 : (pigeonModule.getRoll() < -8 ? -8
    // : 0);
    SmartDashboard.putNumber("VELO GRYO", velocity);
    velocity = pigeonModule.getRoll() * .55;
    if (Math.abs(pigeonModule.getRoll()) < 7) { // 8
      velocity = 0;
    }
    swerveSubsystem.drive(new Translation2d(velocity, 0), 0.0, true, true);

    // for (SwerveModule mod: SwerveMods)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ended");
    // TODO Auto-generated method stub
    swerveSubsystem.drive(
        new Translation2d(0, 0),
        0,
        true,
        true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
