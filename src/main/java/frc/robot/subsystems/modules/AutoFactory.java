package frc.robot.subsystems.modules;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Autons.AutoBase;

@Deprecated
public class AutoFactory extends CommandBase { 
    // private HashMap<String, Command> eventMap;
    // private Double[] pathConstraints = { 4.0, 3.0 }; // velo, accel

    // private SwerveAutoBuilder autoBuilder;
    // private String pathFileName;

    // public AutoFactory(AutoBase autoBase, SwerveAutoBuilder autoBuilder, HashMap<String, Command> eventMap) {
    //     this.eventMap = eventMap;
    //     this.autoBuilder = autoBuilder;
    //     // eventMap();
    // }

    // // First Method Called
    // private void setPath(String path) {
    //     this.pathFileName = path;
    // }

    // // Second Method Called
    // private Command pathFollowingCommand() {
    //     return autoBuilder.fullAuto(pathGroup());
    // }

    // // Third Method Called
    // private List<EventMarker> getPathMarkers() {
    //     return PathPlanner.loadPath(pathFileName,
    //             new PathConstraints(pathConstraints[0], pathConstraints[1])).getMarkers();
    // }

    // // Forth Method Called
    // private List<PathPlannerTrajectory> pathGroup() {
    //     return PathPlanner.loadPathGroup(pathFileName,
    //             new PathConstraints(pathConstraints[0], pathConstraints[1]));
    // }



    // // private void eventMap() {
    // //     eventMap.put("Stow", new stowAway(extender, arm, wrist));
    // //     eventMap.put("Run Comp", new InstantCommand(() -> compressor.enableAnalog(100, 120)));
    // //     eventMap.put("Fix Wrist", new InstantCommand(() -> wrist.setPosition(5)));
    // //     eventMap.put("Cone Start", new groundConeCommand(extender, arm, wrist, claw));
    // //     eventMap.put("Cube Start", new groundCubeCommand(extender, arm, wrist, claw));
    // //     eventMap.put("High", new PlaceHigh(extender, arm, wrist, claw, compressor));
    // //     eventMap.put("Mid", new PlaceMid(extender, arm, wrist, claw, compressor));
    // //     eventMap.put("Throw Cone", new InstantCommand(() -> claw.outTakeCone()));
    // //     eventMap.put("Throw Cube", new InstantCommand(() -> claw.outTakeCube()));
    // //     eventMap.put("Cone Claw", new InstantCommand(() -> claw.coneIntake()));
    // //     eventMap.put("Cube Claw", new InstantCommand(() -> claw.cubeIntake()));
    // //     eventMap.put("Stop Claw", new InstantCommand(() -> claw.stopClaw()));
    // // }

    // public Command createAuto(String path) {
    //     // eventMap();
    //     // System.out.println("EVENTMAP 1" + eventMap.isEmpty());
    //     // System.out.println("EVENTMAP 2" + eventMap.toString());
    //     // System.out.println("EVENTMAP 3" + eventMap.isEmpty());

    //     setPath(path);
    //     return new FollowPathWithEvents(pathFollowingCommand(), getPathMarkers(), eventMap);
    // }
}