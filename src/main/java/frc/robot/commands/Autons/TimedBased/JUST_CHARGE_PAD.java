// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons.TimedBased;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Swerve.DriveForward;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.modules.PigeonModule;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JUST_CHARGE_PAD extends SequentialCommandGroup {
  /** Creates a new JUST_CHARGE_PAD. */
  private PigeonModule pigeonModule;

  public JUST_CHARGE_PAD(SwerveSubsystem SwerveDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    pigeonModule = pigeonModule.getPigeonModule();
    addCommands(
        new DriveForward(SwerveDrive, 0, 35, 2), // tilt charge pad
        new DriveForward(SwerveDrive, 0, 20, 3), // continue past charge pad
        new WaitCommand(1),
        new DriveForward(SwerveDrive, 0, -30, 2), // reverse back onto the pad
        new WaitCommand(1),
        new DriveForward(SwerveDrive, 0, -10, 4)
    // reverse back onto the pad
    );
  }
}
