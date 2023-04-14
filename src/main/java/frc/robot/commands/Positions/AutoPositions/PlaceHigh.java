// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions.AutoPositions;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Positions.CheckCompressor;
import frc.robot.commands.Positions.highAuto;
import frc.robot.commands.Positions.stowAway;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.extenderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceHigh extends SequentialCommandGroup {
  /** Creates a new PlaceHigh. */
  public PlaceHigh(extenderSubsystem extender, ArmSubsystem arm, WristSubsystem wrist, ClawSubsystem claw,
      Compressor compressor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new CheckCompressor(compressor, 10),
        new InstantCommand(() -> wrist.setPosition(1)),
        new WaitCommand(.3),
        new InstantCommand(() -> claw.coneIntake()),
        new WaitCommand(1),
        new highAuto(extender, arm, wrist),
        new InstantCommand(() -> claw.stopClaw()),
        new WaitCommand(1.25),
        new InstantCommand(() -> wrist.setPosition(-29)),
        new WaitCommand(.5),
        new InstantCommand(() -> claw.openClaw()),
        new stowAway(extender, arm, wrist));
  }
}
