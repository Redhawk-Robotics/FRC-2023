// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDInterface;

public class SetPIDPosition extends CommandBase {
  private PIDInterface sys;
  private double pos;

  /** Creates a new SetPIDPosition. */
  public SetPIDPosition(PIDInterface sys, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sys = sys;
    this.pos = pos;
    addRequirements(sys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.sys.setPosition(this.pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(sys.getCurrentPosition() - pos) < 2.5;
  }
}
