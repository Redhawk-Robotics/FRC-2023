package frc.robot.subsystems.modules;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Autons.AutoBase;
import frc.robot.commands.Positions.groundConeCommand;
import frc.robot.commands.Positions.groundCubeCommand;
import frc.robot.commands.Positions.stowAway;
import frc.robot.commands.Positions.AutoPositions.PlaceHigh;
import frc.robot.commands.Positions.AutoPositions.PlaceMid;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.extenderSubsystem;

public class AutoFactory extends CommandBase {
    private HashMap<String, Command> eventMap;
    private Double[] pathConstraints = { 4.0, 3.0 }; // velo, accel
    private final extenderSubsystem extender;
    private SwerveAutoBuilder autoBuilder;
    private final Compressor compressor;
    private final WristSubsystem wrist;
    private final ClawSubsystem claw;
    private final ArmSubsystem arm;
    private String pathFileName;

    public AutoFactory(ArmSubsystem arm, extenderSubsystem extender,
            WristSubsystem wrist, ClawSubsystem claw,
            Compressor compressor, AutoBase autoBase) {
        this.eventMap = new HashMap<String, Command>();
        eventMap();
        this.autoBuilder = autoBase.CustomSwerveAutoBuilder();
        this.compressor = compressor;
        this.extender = extender;
        this.wrist = wrist;
        this.claw = claw;
        this.arm = arm;
        addRequirements(arm, extender, wrist, claw);
    }

    private List<EventMarker> getPathMarkers() {
        return PathPlanner.loadPath(pathFileName,
                new PathConstraints(pathConstraints[0], pathConstraints[1])).getMarkers();
    }

    private List<PathPlannerTrajectory> pathGroup() {
        return PathPlanner.loadPathGroup(pathFileName,
                new PathConstraints(pathConstraints[0], pathConstraints[1]));
    }

    private Command pathFollowingCommand() {
        return autoBuilder.fullAuto(pathGroup());
    }

    private void setPath(String path) {
        this.pathFileName = path;
    }

    private void eventMap() {
        eventMap.put("Stow", new stowAway(extender, arm, wrist));
        eventMap.put("Run Comp", new InstantCommand(() -> compressor.enableAnalog(100, 120)));
        eventMap.put("Fix Wrist", new InstantCommand(() -> wrist.setPosition(5)));
        eventMap.put("Cone Start", new groundConeCommand(extender, arm, wrist, claw));
        eventMap.put("Cube Start", new groundCubeCommand(extender, arm, wrist, claw));
        eventMap.put("High", new PlaceHigh(extender, arm, wrist, claw, compressor));
        eventMap.put("Mid", new PlaceMid(extender, arm, wrist, claw, compressor));
        eventMap.put("Throw Cone", new InstantCommand(() -> claw.outTakeCone()));
        eventMap.put("Throw Cube", new InstantCommand(() -> claw.outTakeCube()));
        eventMap.put("Cone Claw", new InstantCommand(() -> claw.coneIntake()));
        eventMap.put("Cube Claw", new InstantCommand(() -> claw.cubeIntake()));
        eventMap.put("Stop Claw", new InstantCommand(() -> claw.stopClaw()));
    }

    public Command createAuto(String path) {
        eventMap();
        setPath(path);
        return new FollowPathWithEvents(pathFollowingCommand(), getPathMarkers(), eventMap);
    }
}