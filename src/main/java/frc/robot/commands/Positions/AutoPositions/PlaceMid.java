// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions.AutoPositions;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Positions.CheckCompressor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.extenderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceMid extends SequentialCommandGroup {
  /** Creates a new PlaceMid. */
  public PlaceMid(extenderSubsystem extender, ArmSubsystem arm, WristSubsystem wrist, ClawSubsystem claw,
      Compressor compressor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new CheckCompressor(compressor, 10),
        new InstantCommand(() -> wrist.setPosition(1)),
        new WaitCommand(.5),
        new InstantCommand(() -> claw.closeClaw()),
        new ParallelCommandGroup(
            new InstantCommand(() -> extender.setPosition(0)),
            new InstantCommand(() -> wrist.setPosition(5))),
        new InstantCommand(() -> arm.setPosition(43)),
        new ParallelCommandGroup(
            new InstantCommand(() -> wrist.setPosition(-28)),
            new InstantCommand(() -> extender.setPosition(0)),
            new InstantCommand(() -> arm.setPosition(43))),
        new WaitCommand(1),
        new ParallelCommandGroup(
            new InstantCommand(() -> claw.openClaw()),
            new InstantCommand(() -> arm.setPosition(43))));
  }
}
