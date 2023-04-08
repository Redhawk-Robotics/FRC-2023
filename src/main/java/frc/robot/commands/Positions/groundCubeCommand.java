// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.extenderSubsystem;

public class groundCubeCommand extends SequentialCommandGroup {
  /** Creates a new groundCubeCommand. */
  public groundCubeCommand(extenderSubsystem extender, ArmSubsystem arm, WristSubsystem wrist,
      ClawSubsystem claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(

        new InstantCommand(() -> extender.setPosition(0)),

        new InstantCommand(() -> arm.setPosition(18)),
        new ParallelDeadlineGroup( // FIXME TAKE OUT THIS AFTER TESTING

            new InstantCommand(() -> extender.setPosition(0)),

            new InstantCommand(() -> claw.cubeIntake())),
        new WaitCommand(
            1),
        new InstantCommand(() -> wrist.setPosition(-20))

    );
  }
}
