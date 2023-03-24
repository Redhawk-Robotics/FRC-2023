// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Setting;
import frc.robot.subsystems.modules.CompressorModule;
import frc.robot.test.clawTest;

public class ClawManual extends CommandBase {
  /** Creates a new Claw. */
  private clawTest claw;
  private final double speed;
  
  // FINISH the stuff here

  public ClawManual(clawTest claw, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    this.speed = speed.getAsDouble();

    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ClawManual Activated:))");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double motorSpeed = speed;
    
    // this.claw.stopMotors();
    this.claw.closeClaw();
  }

  // ADD TRIGGERS FOR BUTTON
  // WHEN PRESSED OPEN CLAW AND MOVE THE MOTORS

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("WristManual ended:((");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
