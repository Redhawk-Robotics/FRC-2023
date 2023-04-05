// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class TestPathPlannerAuton extends AutoBase {
    List<PathPlannerTrajectory> autoPathGroup = PathPlanner.loadPathGroup("Test", new PathConstraints(2.0, 1.5));

    TestPathPlannerAuton(SwerveSubsystem swerveDrive) {
        super(swerveDrive);

        SwerveAutoBuilder autoBuilder = CustomSwerveAutoBuilder();

        addCommands(new WaitCommand(2),

                // autoBuilder.fullAuto(autoPathGroup),
                autoBuilder.fullAuto(autoPathGroup));

        // new AutoBalanceRenew(swerveDrive);//try later
    }
}
