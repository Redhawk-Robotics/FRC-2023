// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Setting;
import frc.robot.constants.Setting.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBase extends SequentialCommandGroup {
  /** Creates a new AutoBase. */
  private SwerveSubsystem swerve;

  public AutoBase(SwerveSubsystem swerveDrive) {
    swerve = swerveDrive;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }

  public SwerveAutoBuilder CustomSwerveAutoBuilder() {
    return new SwerveAutoBuilder(
        swerve::getPose, // pose2d supplier
        swerve::resetOdometry, // reset odometry at the beginning of auto
        Setting.mKinematics, // swerve kinematics
        new PIDConstants(52, 0, 0), // x y controller
        new PIDConstants(11, 0, .001), // theta controller
        swerve::setModuleStates,
        AutoConstants.EventMap,
        true,
        swerve);
  }

  public PPSwerveControllerCommand CustomPathControllerCommand(PathPlannerTrajectory trajectory) {
    return new PPSwerveControllerCommand(
        trajectory,
        swerve::getPose,
        Setting.mKinematics,
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        new PIDController(AutoConstants.kPThetaController, 0, 0),
        swerve::setModuleStates,
        swerve);
  }
}
