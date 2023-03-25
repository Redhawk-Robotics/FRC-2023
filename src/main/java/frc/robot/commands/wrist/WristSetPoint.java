// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristSetPoint extends CommandBase {
  /** Creates a new WristSetPoint. */
  private final WristSubsystem wristSubsystem;
  private final PIDController PIDWristController;

  public WristSetPoint(WristSubsystem wristSubsystem, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wristSubsystem = wristSubsystem;
    this.PIDWristController = new PIDController(0,0,0);
  
    PIDWristController.setSetpoint(setPoint);

    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WristSetPointCMD started!");
    PIDWristController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  double speed = PIDWristController.calculate(wristSubsystem.getEncoderMeters());
  wristSubsystem.setMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setMotor(0);
    System.out.println("WristSetPointCMD ended!");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
