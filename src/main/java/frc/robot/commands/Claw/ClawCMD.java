// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.test.clawTest;

public class ClawCMD extends CommandBase {
  /** Creates a new ClawCMD. */
  private clawTest claw;
  private boolean open;

  public ClawCMD(clawTest claw,  boolean open) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    this.open = open;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (open) {
      claw.coneIntake();
    } else {
      claw.outTake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
