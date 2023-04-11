// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CheckCompressor extends CommandBase {
  private Compressor compressor;
  private double setPSi;

  /** Creates a new CheckCompressor. */
  public CheckCompressor(Compressor compressor, double setPSi) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.compressor = compressor;
    this.setPSi = setPSi;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    compressor.enableAnalog(100, 120);
    SmartDashboard.putNumber("PRESSURE", compressor.getPressure());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return compressor.getPressure() > setPSi;
  }
}
