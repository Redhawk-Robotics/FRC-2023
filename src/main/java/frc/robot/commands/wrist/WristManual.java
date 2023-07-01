// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristManual extends CommandBase {
  /** Creates a new Wrist. */
  private final WristSubsystem wristSubsystem;
  private final DoubleSupplier speed;
  private final SparkMaxPIDController wristContoller;

  public WristManual(WristSubsystem wristSubsystem, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wristSubsystem = wristSubsystem;
    this.speed = speed;
    wristContoller = wristSubsystem.getWristMotor().getPIDController();
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("WristManual Activated:))");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double motorSpeed = speed.getAsDouble();
    wristSubsystem.setMotor(Math.abs(speed.getAsDouble()) < .1 ? 0: -speed.getAsDouble());
    // wristContoller.setReference(wristSubsystem.getCurrentPosition(), CANSparkMax.ControlType.kSmartMotion);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setMotor(0);
    System.out.println("WristManual Ended:))");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
