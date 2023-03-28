// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Setting;
import frc.robot.subsystems.extenderSubsystem;
import frc.robot.test.testWhatever;

public class ExtenderManual extends CommandBase {
  // public extenderSubsystem extenderSubsystem;
  private testWhatever tester;
  private DoubleSupplier speed;
  /** Creates a new Extender. */
  public ExtenderManual(testWhatever tester, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tester = tester;
    this.speed = speed;

    addRequirements(tester); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tester.extenderController(speed.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tester.extenderController(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
