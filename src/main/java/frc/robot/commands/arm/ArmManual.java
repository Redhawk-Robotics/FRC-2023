// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManual extends CommandBase {
  public ArmSubsystem armSubsystem;
  public BooleanSupplier goUp, goDown;

  /** Creates a new Arm. */
  public ArmManual(ArmSubsystem armSub, BooleanSupplier up, BooleanSupplier down) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSub;
    addRequirements(this.armSubsystem); 

    this.goUp = up;
    this.goDown = down;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = goUp.getAsBoolean() ? .3: (goDown.getAsBoolean() ? -.1: 0);
    armSubsystem.setMotor(power);
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
