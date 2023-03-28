// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.test.armTest;
import frc.robot.test.testWhatever;

public class ArmSetPoint extends CommandBase {
  /** Creates a new ArmSetPoint. */
  // public ArmSubsystem armSubsystem;
  private armTest tester;
  double encoderValue;

  public ArmSetPoint(armTest tester, double encoderValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    // this.armSubsystem = armSubsystem;
    this.tester = tester;
    this.encoderValue = encoderValue;

    addRequirements(tester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (encoderValue - tester.armEncoder() > 0) {
      tester.upGoArm();
    } else {
      tester.downGoArm();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tester.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(encoderValue - tester.armEncoder()) < 5) {
      return true;
    }
    return false;
  }
}
