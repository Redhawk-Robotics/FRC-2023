// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.test.armTest;
import frc.robot.test.clawTest;
import frc.robot.test.extenderTest;
import frc.robot.test.wristTest;

@Deprecated
public class StoweAway extends CommandBase {
  private armTest arm;
  private extenderTest extender;
  private wristTest wrist;
  // private boolean checker1, checker2;

  /** Creates a new StoweAway. */
  public StoweAway(armTest arm, extenderTest extender, wristTest wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.extender = extender;
    this.wrist = wrist;
    // this.checker1 = false;
    // this.checker2 = false;
    addRequirements(arm, extender, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (wrist.wristEncoder() < 4) {
    //   wrist.upGoWrist();
    // } else {
    //   wrist.stopWrist();
    //   if (extender.extenderEncoder() < 0) {
    //     extender.upExtender();
    //   } else {
    //     extender.stopExtender();
    //   }
    // }

    if (wrist.wristEncoder() < 4 && extender.extenderEncoder() < 0) {
      wrist.upGoWrist();
      extender.upExtender();
      if (extender.extenderEncoder() > -1) {
        extender.stopExtender();
        // checker2 = true;
      }
    } else {
      wrist.stopWrist();
      // checker1 = true;
    }

    // checker1 && checker2
    if (wrist.wristEncoder() >= 4 && extender.extenderEncoder() >= -100) {
      if (arm.armEncoder() > 0) {
        arm.downGoArm();
      } else {
        arm.stopArm();
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.armEncoder() < 5 ? true: false;
  }
}
