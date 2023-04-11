// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManual extends CommandBase {
  // public ArmSubsystem armSubsystem;
  private ArmSubsystem tester;
  private DoubleSupplier speed;

  /** Creates a new Arm. */
  public ArmManual(ArmSubsystem tester, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tester = tester;
    this.speed = speed;

    addRequirements(tester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double speedrevert = speed.getAsDouble();
    tester.setMotor(-speed.getAsDouble() * .5);
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
