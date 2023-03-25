// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.test.testWhatever;

public class testMotorCommand extends CommandBase {
  /** Creates a new testMotorCommand. */
  private testWhatever tester;
  private boolean power;
  double setPower = 0;

  public testMotorCommand(testWhatever tester, boolean power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tester = tester;
    this.power = power;
    addRequirements(tester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("testMotorCommand started!");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (power == true) {
    //   setPower = .1;
    // } else {
    //   setPower = 0;
    // }
    //tester.setMotor(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
