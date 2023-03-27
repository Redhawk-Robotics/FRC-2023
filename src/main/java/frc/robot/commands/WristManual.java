// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.test.testMotorCommand;
//import frc.robot.Constants;
import frc.robot.constants.Ports;
import frc.robot.constants.Setting;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.test.testClawMotors;

public class WristManual extends CommandBase {

  private WristSubsystem armSubsystem;
  private BooleanSupplier up, down;

  SlewRateLimiter leftRateLimiter = new SlewRateLimiter(1);
  SlewRateLimiter rightRateLimiter = new SlewRateLimiter(1);

  /** Creates a new DefaultDemandCommand. */
  public WristManual(WristSubsystem arm, BooleanSupplier up, BooleanSupplier down) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = arm;
    addRequirements(armSubsystem);

    this.up = up;
    this.down = down;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = up.getAsBoolean() ? Setting.wristSetting.wristSpeed
        : (down.getAsBoolean() ? Setting.wristSetting.wristSpeedReverse : 0);
    armSubsystem.setMotor(power);
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
