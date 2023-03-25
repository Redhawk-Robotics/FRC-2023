// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Setting;
import frc.robot.subsystems.extenderSubsystem;

public class ExtenderManual extends CommandBase {
  public extenderSubsystem extenderSubsystem;
  public BooleanSupplier extend, retract;
  /** Creates a new Extender. */
  public ExtenderManual(extenderSubsystem extenderSubsystem, BooleanSupplier extend, BooleanSupplier retract) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extenderSubsystem = extenderSubsystem;
    addRequirements(this.extenderSubsystem);

    this.extend = extend;
    this.retract = retract;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = extend.getAsBoolean() ? Setting.extenderSetting.extenderSpeed: (retract.getAsBoolean() ? Setting.extenderSetting.extenderSpeedReverse: 0);
    extenderSubsystem.setMotor(power); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
