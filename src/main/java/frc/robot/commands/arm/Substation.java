// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.test.armTest;
import frc.robot.test.extenderTest;
import frc.robot.test.wristTest;

public class Substation extends CommandBase {
  private armTest arm;
  private extenderTest extender;
  private wristTest wrist;

  /** Creates a new Substation. */
  public Substation(armTest arm, extenderTest extender, wristTest wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.extender = extender;
    this.wrist = wrist;

    addRequirements(arm, extender, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (wrist.wristEncoder() < 4 && extender.extenderEncoder() < 0) {
      wrist.upGoWrist();
      extender.upExtender();
      if (extender.extenderEncoder() > 0) {
        extender.stopExtender();
      }
    } else {
      wrist.stopWrist();
      if (arm.armEncoder() < 42 && wrist.wristEncoder() > -30 && extender.extenderEncoder() > -260) {
        arm.upGoArm();
        wrist.downGoWrist();
        extender.downExtender();
        if (extender.extenderEncoder() > 0) {
          extender.stopExtender();
        }
      } else {
        arm.stopArm();
        wrist.stopWrist();
      }
    }

    // if (wrist.wristEncoder() > -30 && extender.extenderEncoder() > -260) {
    //   wrist.downGoWrist();
    //   extender.downExtender();
    //   if (extender.extenderEncoder() > 0) {
    //     extender.stopExtender();
    //   }
    // } else {
    //   wrist.stopWrist();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.armEncoder() >= 40 ? true : false;
  }
}
