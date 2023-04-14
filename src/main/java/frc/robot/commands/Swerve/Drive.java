// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Setting;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.modules.PigeonModule;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  private SwerveSubsystem Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowSpeedSup;
  private Field2d field;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public Drive(
      // Use addRequirements() here to declare subsystem dependencies.
      SwerveSubsystem s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier slowSpeedSup) {
    this.Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.slowSpeedSup = slowSpeedSup;
    this.field = new Field2d();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    field.setRobotPose(Swerve.getPose());
    double speedMultiplier = slowSpeedSup.getAsBoolean() ? 0.5 : 1.0;// try 1.0 : 0.5

    /* Get Values, Deadband */
    double translationVal = translationLimiter.calculate(
        speedMultiplier *
            MathUtil.applyDeadband(translationSup.getAsDouble(), Ports.stickDeadband));
    double strafeVal = strafeLimiter.calculate(
        speedMultiplier *
            MathUtil.applyDeadband(strafeSup.getAsDouble(), Ports.stickDeadband));
    double rotationVal = rotationLimiter.calculate(
        speedMultiplier *
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Ports.stickDeadband));

    SmartDashboard.putNumber("gyro angle", PigeonModule.getPigeonModule().getYaw());
    /* Drive */
    Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Setting.maxVelocityMetersPerSecond),
        rotationVal * Setting.maxAngularVelocityRadiansPerSecond,
        !robotCentricSup.getAsBoolean(),
        true);
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
