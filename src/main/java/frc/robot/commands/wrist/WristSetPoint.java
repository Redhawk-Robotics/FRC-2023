// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.test.wristTest;

public class WristSetPoint extends CommandBase {
  /** Creates a new WristSetPoint. */
  private final wristTest wrist;
  // private final PIDController PIDWristController;
  double encoderValue;

  public WristSetPoint(wristTest wrist, double encoderValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    // this.PIDWristController = new PIDController(0,0,0);
    this.encoderValue = encoderValue;
    // PIDWristController.setSetpoint(setPoint);

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WristSetPointCMD started!");
    // PIDWristController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  // double speed = PIDWristController.calculate(wristSubsystem.getCurrentPosition());
  // wristSubsystem.setMotor(speed);
  if (wrist.wristEncoder() > encoderValue) {
    wrist.downGoWrist();
  } else if (wrist.wristEncoder() < encoderValue) {
    wrist.upGoWrist();
  } else {
    wrist.stopWrist();
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // wristSubsystem.setMotor(0);
    System.out.println("WristSetPointCMD ended!");
    wrist.stopWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(encoderValue - wrist.wristEncoder()) < 1.5) {
      wrist.stopWrist();
      return true;
    }
    return false;
    // return true;
  }
}
