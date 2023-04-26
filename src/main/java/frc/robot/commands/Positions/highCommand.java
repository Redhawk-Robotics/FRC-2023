// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.extender.ResetExtender;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.extenderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class highCommand extends SequentialCommandGroup {
  /** Creates a new highCommand. */
  public highCommand(extenderSubsystem extender, ArmSubsystem arm, WristSubsystem wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(extender, arm, wrist);
    addCommands(
        new InstantCommand(() -> wrist.setPosition(5)),
        // new ParallelCommandGroup(
        // new InstantCommand(() -> extender.setPosition(0)),
        // new InstantCommand(() -> wrist.setPosition(5))),

        new InstantCommand(() -> arm.setPosition(66)),
        new WaitCommand(1.5),
        // new InstantCommand(() -> extender.setPosition(194)), // 194 when neo was 27:1
        // ratio
        new ResetExtender(extender, 36) // 194 when neo was 27:1
    // ratio
    // new InstantCommand(() -> wrist.setPosition(5))
    );
  }
}
