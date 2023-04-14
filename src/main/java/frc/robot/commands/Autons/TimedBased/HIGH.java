// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons.TimedBased;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Positions.highCommand;
import frc.robot.commands.Positions.stowAway;
import frc.robot.commands.Swerve.DriveForward;
import frc.robot.commands.Swerve.DriveTurn;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.extenderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HIGH extends SequentialCommandGroup {
  /** Creates a new HIGH. */
  public HIGH(SwerveSubsystem SwerveDrive, extenderSubsystem extender, ArmSubsystem arm, WristSubsystem wrist,
      ClawSubsystem claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new WaitCommand(6),
        new InstantCommand(() -> claw.closeClaw()),
        new stowAway(extender, arm, wrist),
        new highCommand(extender, arm, wrist),
        new InstantCommand(() -> wrist.setPosition(-29)),
        new WaitCommand(2),
        new InstantCommand(() -> claw.openClaw()),
        new DriveForward(SwerveDrive, 0, -3, 2),
        new stowAway(extender, arm, wrist),
        new DriveTurn(SwerveDrive, 0, -35, 1.5));
  }
}
