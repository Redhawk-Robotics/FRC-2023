// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons.TimedBased;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Positions.stowAway;
import frc.robot.commands.Swerve.DriveForward;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.extenderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeNoBack extends SequentialCommandGroup {
  /** Creates a new ChargeNoBack. */
  public ChargeNoBack(SwerveSubsystem SwerveDrive, extenderSubsystem extender, ArmSubsystem arm,
      WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new stowAway(extender, arm, wristSubsystem),
        new DriveForward(SwerveDrive, 0, 35, 2), // tilt charge pad
        new DriveForward(SwerveDrive, 0, 20, 2), // continue past charge pad
        new WaitCommand(.5),
        new DriveForward(SwerveDrive, 0, -30, 2.1), // reverse back onto the pad
        new WaitCommand(.25),
        // new DriveForward(SwerveDrive, 0, -10, 3),
        new GyroBalance(SwerveDrive));
  }
}
