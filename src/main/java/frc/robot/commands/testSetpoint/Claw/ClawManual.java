// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testSetpoint.Claw;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Setting;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.test.CompressorModule;
import frc.robot.test.clawTest;

@Deprecated

public class ClawManual extends CommandBase {
  /** Creates a new Claw. */
  private ClawSubsystem claw;
  private BooleanSupplier coneIntake, cubeIntake, leftOutTake, rightOutTake;

  // FINISH the stuff here

  public ClawManual(ClawSubsystem claw, BooleanSupplier coneIntake, BooleanSupplier cubeIntake,
      BooleanSupplier leftOutTake, BooleanSupplier rightOutTake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    addRequirements(this.claw);

    this.coneIntake = coneIntake;
    this.cubeIntake = cubeIntake;
    this.leftOutTake = leftOutTake;
    this.rightOutTake = rightOutTake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ClawManual Activated:))");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (coneIntake.getAsBoolean()) {
      claw.coneIntake();
    } else if (cubeIntake.getAsBoolean()) {
      claw.cubeIntake();
    } else {
      claw.stopClaw();
    }

    if (leftOutTake.getAsBoolean() || rightOutTake.getAsBoolean()) {
      claw.outTake();
    }
  }

  // ADD TRIGGERS FOR BUTTON
  // WHEN PRESSED OPEN CLAW AND MOVE THE MOTORS

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
