// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.test.testMotorCommand;
//import frc.robot.Constants;
import frc.robot.constants.Ports;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.test.testClawMotors;


public class DefaultArmCommand extends CommandBase {

  private testClawMotors armSubsystem;

  private XboxController controller = new XboxController(Ports.Gamepad.OPERATOR);

  private double leftArmPower, rightArmPower;

  SlewRateLimiter leftRateLimiter = new SlewRateLimiter(1);
  SlewRateLimiter rightRateLimiter = new SlewRateLimiter(1);


  /** Creates a new DefaultDemandCommand. */
  public DefaultArmCommand(testClawMotors arm, XboxController con) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSubsystem = arm;
    this.controller = con;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    leftArmPower = 0;
    rightArmPower = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    leftArmPower  = XboxController.Axis.kLeftTrigger.value;
    rightArmPower = XboxController.Axis.kRightTrigger.value;

    armSubsystem.setSpeed(leftRateLimiter.calculate(leftArmPower), rightRateLimiter.calculate(rightArmPower));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    armSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
