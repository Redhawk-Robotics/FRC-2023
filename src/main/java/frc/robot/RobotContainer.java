// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.ArmManual;
import frc.robot.commands.Autons.AutoBase;
import frc.robot.commands.Autons.DoNothingAuton;
import frc.robot.commands.Autons.TimedBased.BLUE_LEFT_PLACE_MID_DIP;
import frc.robot.commands.Autons.TimedBased.BLUE_RIGHT_PLACE_MID_DIP;
import frc.robot.commands.Autons.TimedBased.CONE_MOBILITY;
import frc.robot.commands.Autons.TimedBased.ChargeNoBack;
import frc.robot.commands.Autons.TimedBased.ChargeNoEngage;
import frc.robot.commands.Autons.TimedBased.JUST_CHARGE_PAD;
import frc.robot.commands.Autons.TimedBased.LOW_ENGAGE;
import frc.robot.commands.Autons.TimedBased.MID_TURN_GRAB;
import frc.robot.commands.Autons.TimedBased.PP_UNTESTED_PLS_CHANGE;
import frc.robot.commands.Autons.TimedBased.THROW_DIP;
import frc.robot.commands.Positions.CheckCompressor;
import frc.robot.commands.Positions.doubleSubstation;
import frc.robot.commands.Positions.groundConeCommand;
import frc.robot.commands.Positions.groundCubeCommand;
import frc.robot.commands.Positions.highCommand;
import frc.robot.commands.Positions.keepCone;
import frc.robot.commands.Positions.midCommand;
import frc.robot.commands.Positions.releaseCone;
import frc.robot.commands.Positions.shootCube;
import frc.robot.commands.Positions.shootCone;
import frc.robot.commands.Positions.singleSubstation;
import frc.robot.commands.Positions.stowAway;
import frc.robot.commands.Positions.AutoPositions.PlaceHigh;
import frc.robot.commands.Positions.AutoPositions.PlaceMid;
import frc.robot.commands.Positions.AutoPositions.stowAwayAuto;
import frc.robot.commands.Swerve.Drive;
import frc.robot.commands.Swerve.DriveForward;
import frc.robot.commands.Wrist.WristManual;
import frc.robot.commands.extender.ExtenderManual;
import frc.robot.commands.extender.ResetExtender;
import frc.robot.constants.Ports;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.extenderSubsystem;
import frc.robot.subsystems.modules.AutoFactory;

import java.util.HashMap;
import java.util.List;

// import com.pathplanner.lib.*;
// import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private boolean demoControls = true;
  /* Subsystems */
  private final SwerveSubsystem SwerveDrive = new SwerveSubsystem();
  // private final Limelight Limelight = new Limelight();
  // private final LEDController LEDS = new LEDController();

  private final WristSubsystem wrist = new WristSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final extenderSubsystem extender = new extenderSubsystem();
  private final ClawSubsystem claw = new ClawSubsystem();
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  private HashMap<String, Command> eventMap = new HashMap<>();
  private String pathFileName;
  private Double[] pathConstraints = { 4.0, 3.0 }; // velo, accel
  
  // private final AutoBase autoBase;
  // private final AutoFactory pathPlannerBuilder = new AutoFactory(autoBase, autoBuilder, getEventMap());

  // Positions
  private final groundConeCommand groundCone = new groundConeCommand(extender, arm, wrist, claw);
  private final groundCubeCommand groundCube = new groundCubeCommand(extender, arm, wrist, claw);

  private final highCommand highCommand = new highCommand(extender, arm, wrist);
  private final midCommand midCommand = new midCommand(extender, arm, wrist);

  private final shootCone shootCone = new shootCone(extender, arm, wrist, claw);
  private final releaseCone releaseCone = new releaseCone(claw);
  private final keepCone keepCone = new keepCone(claw);
  private final shootCube shootCube = new shootCube(extender, arm, wrist, claw);

  private final singleSubstation singleSubstation = new singleSubstation(extender, arm, wrist, claw);
  private final doubleSubstation doubleSubstation = new doubleSubstation(extender, arm, wrist, claw);

  private final stowAway stowAway = new stowAway(extender, arm, wrist);

  // private final autoAligner autoAligner = new autoAligner(SwerveDrive,
  // Limelight);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /* Controllers */
  private final XboxController DRIVER = new XboxController(Ports.Gamepad.DRIVER);
  private final XboxController OPERATOR = new XboxController(Ports.Gamepad.OPERATOR);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final int rightYAxis1 = XboxController.Axis.kRightY.value;

  private final int leftTrigger1 = XboxController.Axis.kLeftTrigger.value;
  private final int rightTrigger1 = XboxController.Axis.kRightTrigger.value;

  /* Driver Buttons */
  private final Trigger driver_A_zeroGyro = new JoystickButton(DRIVER, XboxController.Button.kA.value);
  private final Trigger driver_X_robotCentric = new JoystickButton(DRIVER, XboxController.Button.kX.value);

  private final Trigger driver_slowSpeed_rightBumper = new JoystickButton(DRIVER,
      XboxController.Button.kRightBumper.value);

  // Additional buttons
  private final Trigger driver_leftBumper = new JoystickButton(DRIVER, XboxController.Button.kLeftBumper.value);

  private final Trigger driver_B = new JoystickButton(DRIVER, XboxController.Button.kB.value);

  private final Trigger driver_Y = new JoystickButton(DRIVER, XboxController.Button.kY.value);

  private final Trigger driver_BottomRightRearButton = new JoystickButton(DRIVER,
      XboxController.Button.kStart.value);
  private final Trigger driver_BottomLeftRearButton = new JoystickButton(DRIVER,
      XboxController.Button.kBack.value);

  private final Trigger driver_START = new JoystickButton(DRIVER, XboxController.Button.kStart.value);
  private final Trigger driver_BACK = new JoystickButton(DRIVER, XboxController.Button.kBack.value);

  private final Trigger dpadUpButtonDrive = new Trigger(() -> DRIVER.getPOV() == 0);
  private final Trigger dpadRightButtonDrive = new Trigger(() -> DRIVER.getPOV() == 90);
  private final Trigger dpadDownButtonDrive = new Trigger(() -> DRIVER.getPOV() == 180);
  private final Trigger dpadLeftButtonDrive = new Trigger(() -> DRIVER.getPOV() == 270);

  // Controller two - Operator
  private final Trigger dpadUpButtonOperator = new Trigger(() -> OPERATOR.getPOV() == 0);
  private final Trigger dpadRightButtonOperator = new Trigger(() -> OPERATOR.getPOV() == 90);
  private final Trigger dpadDownButtonOperator = new Trigger(() -> OPERATOR.getPOV() == 180);
  private final Trigger dpadLeftButtonOperator = new Trigger(() -> OPERATOR.getPOV() == 270);

  private final int leftYAxis2 = XboxController.Axis.kLeftY.value;
  private final int leftXAxis2 = XboxController.Axis.kLeftX.value;

  private final int rightYAxis2 = XboxController.Axis.kRightY.value;
  private final int rightXAxis2 = XboxController.Axis.kRightX.value;

  private final int leftTrigger2 = XboxController.Axis.kLeftTrigger.value;
  private final int rightTrigger2 = XboxController.Axis.kRightTrigger.value;

  private final Trigger opperator_A = new JoystickButton(OPERATOR, XboxController.Button.kA.value);
  private final Trigger opperator_B = new JoystickButton(OPERATOR, XboxController.Button.kB.value);
  private final Trigger opperator_X = new JoystickButton(OPERATOR, XboxController.Button.kX.value);
  private final Trigger opperator_Y = new JoystickButton(OPERATOR, XboxController.Button.kY.value);

  private final Trigger opperator_RightBumper = new JoystickButton(OPERATOR,
      XboxController.Button.kRightBumper.value);
  
  private final Trigger opperator_leftBumper = new JoystickButton(OPERATOR,
      XboxController.Button.kLeftBumper.value);

  private final Trigger opperator_BottomRightRearButton = new JoystickButton(OPERATOR,
      XboxController.Button.kStart.value);
      
  private final Trigger opperator_BottomLeftRearButton = new JoystickButton(OPERATOR,
      XboxController.Button.kBack.value);

  private final Trigger opperator_TopLeftRearButton = new JoystickButton(OPERATOR,
      XboxController.Button.kLeftStick.value);

  private final Trigger driver_TopLeftRearButton = new JoystickButton(DRIVER,
      XboxController.Button.kLeftStick.value);

  private final Trigger opperator_TopRightRearButton = new JoystickButton(OPERATOR,
      XboxController.Button.kRightStick.value);
  
  private final Trigger driver_TopRightRearButton = new JoystickButton(DRIVER,
      XboxController.Button.kRightStick.value);

  // Create SmartDashboard chooser for autonomous routines
  private static SendableChooser<Command> Autons = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // autoBase = new AutoBase(SwerveDrive, getEventMap());
    // autoBuilder = autoBase.CustomSwerveAutoBuilder();
    /*************/
    /*** DRIVE ***/
    /*************/

    SwerveDrive.setDefaultCommand(
        new Drive(
            SwerveDrive,
            () -> -DRIVER.getRawAxis(translationAxis),
            () -> -DRIVER.getRawAxis(strafeAxis),
            () -> -DRIVER.getRawAxis(rotationAxis),
            () -> driver_X_robotCentric.getAsBoolean(),
            () -> driver_slowSpeed_rightBumper.getAsBoolean()));

    // -------------------------------------

    // wrist.setDefaultCommand(
    //   new WristManual(wrist,
    //     () -> OPERATOR.getRawAxis(leftYAxis2) * .5));

    /*************/
    /*** EXTENDER ***/
    /*************/

    // METHODS: extend, retract
    // extender.setDefaultCommand(
    //   new ExtenderManual(extender,
    //     () -> OPERATOR.getRawAxis(rightYAxis2)));

    // arm.setDefaultCommand(new ArmManual(arm, () -> OPERATOR.getRawAxis(leftXAxis2)));

    // -----------------if robot works, dont work--------------------

    // Configure the trigger bindings, defaults, Autons
    configureDefaultCommands();
    configureButtonBindings();
    configureAutons();
    // final AutoFactory pathPlannerBuilder = new AutoFactory(arm, extender, wrist, claw, compressor, autoBase);

  }

  /****************/
  /*** DEFAULTS ***/
  /****************/

  private void configureDefaultCommands() {
    eventMap();
    // Compressor code
    compressor.enableDigital();
    System.out.println("IS COMP ON???????" + compressor.isEnabled());
    // compressor.enableAnalog(100, 120);

    // compressor.disableCompressor();

    // Camera Server
    // CameraServer.startAutomaticCapture();
  }

  /***************/
  /*** BUTTONS ***/
  /***************/

  private void configureButtonBindings() {
    if (demoControls) {
      // Driver
      driver_A_zeroGyro.onTrue(new InstantCommand(() -> SwerveDrive.zeroGyro()));
      // dpadRightButtonDrive.onTrue(new ResetExtender(extender, 36));
      // dpadLeftButtonDrive.onTrue(new ResetExtender(extender, 0));
      // driver_B.onTrue(autoAligner);// may break code

      // ------------------------------------- COMMANDS
      driver_B.onTrue(midCommand);
      driver_X_robotCentric.onTrue(highCommand);

      dpadUpButtonDrive.onTrue(groundCone);
      dpadUpButtonDrive.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      // dpadLeftButtonOperator.onTrue(singleSubstation);
      // dpadLeftButtonOperator.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      dpadDownButtonDrive.onTrue(groundCube);
      dpadDownButtonDrive.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      driver_Y.onTrue(doubleSubstation);
      driver_Y.whileTrue(new InstantCommand(() -> claw.coneIntake()));
      driver_Y.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      dpadRightButtonDrive.onTrue(stowAway);
      dpadRightButtonDrive.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      // ------------------------------------- ARM
      // opperator_leftBumper.whileTrue(shootCube);
      // opperator_leftBumper.whileFalse(new InstantCommand(() -> claw.stopClaw()));
      // opperator_leftBumper.whileTrue(keepCone);
      // opperator_leftBumper.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      // opperator_RightBumper.whileTrue(shootCone);
      // opperator_RightBumper.whileFalse(new InstantCommand(() -> claw.stopClaw()));
      driver_leftBumper.whileTrue(releaseCone);
      driver_leftBumper.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      // // ------------------------------------- CLAW
      driver_TopLeftRearButton.onTrue(new InstantCommand(() -> claw.opperatorOpenClaw()));
      driver_TopLeftRearButton.onFalse(new InstantCommand(() -> claw.stopClaw()));

      driver_BottomLeftRearButton.onTrue(new InstantCommand(() -> claw.opperatorCloseClaw()));
      driver_BottomLeftRearButton.onFalse(new InstantCommand(() -> claw.stopClaw()));

      // // ------------------------------------- WRIST
      driver_TopRightRearButton.onTrue(new InstantCommand(() -> wrist.upGoWrist()));
      driver_TopRightRearButton.onFalse(new InstantCommand(() -> wrist.stopWrist()));

      driver_BottomRightRearButton.onTrue(new InstantCommand(() -> wrist.downGoWrist()));
      driver_BottomRightRearButton.onFalse(new InstantCommand(() -> wrist.stopWrist()));
    } else {
      // Driver
      driver_A_zeroGyro.onTrue(new InstantCommand(() -> SwerveDrive.zeroGyro()));
      dpadRightButtonDrive.onTrue(new ResetExtender(extender, 36));
      dpadLeftButtonDrive.onTrue(new ResetExtender(extender, 0));
      // driver_B.onTrue(autoAligner);// may break code

      // ------------------------------------- COMMANDS
      opperator_B.onTrue(midCommand);
      opperator_X.onTrue(highCommand);

      dpadUpButtonOperator.onTrue(groundCone);
      dpadUpButtonOperator.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      // dpadLeftButtonOperator.onTrue(singleSubstation);
      // dpadLeftButtonOperator.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      dpadDownButtonOperator.onTrue(groundCube);
      dpadDownButtonOperator.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      opperator_Y.onTrue(doubleSubstation);
      opperator_Y.whileTrue(new InstantCommand(() -> claw.coneIntake()));
      opperator_Y.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      opperator_A.onTrue(stowAway);
      opperator_A.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      // ------------------------------------- ARM
      // opperator_leftBumper.whileTrue(shootCube);
      // opperator_leftBumper.whileFalse(new InstantCommand(() -> claw.stopClaw()));
      opperator_leftBumper.whileTrue(keepCone);
      opperator_leftBumper.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      // opperator_RightBumper.whileTrue(shootCone);
      // opperator_RightBumper.whileFalse(new InstantCommand(() -> claw.stopClaw()));
      opperator_RightBumper.whileTrue(releaseCone);
      opperator_RightBumper.whileFalse(new InstantCommand(() -> claw.stopClaw()));

      // // ------------------------------------- CLAW
      opperator_TopLeftRearButton.onTrue(new InstantCommand(() -> claw.opperatorOpenClaw()));
      opperator_TopLeftRearButton.onFalse(new InstantCommand(() -> claw.stopClaw()));

      opperator_BottomLeftRearButton.onTrue(new InstantCommand(() -> claw.opperatorCloseClaw()));
      opperator_BottomLeftRearButton.onFalse(new InstantCommand(() -> claw.stopClaw()));

      // // ------------------------------------- WRIST
      opperator_TopRightRearButton.onTrue(new InstantCommand(() -> wrist.upGoWrist()));
      opperator_TopRightRearButton.onFalse(new InstantCommand(() -> wrist.stopWrist()));

      opperator_BottomRightRearButton.onTrue(new InstantCommand(() -> wrist.downGoWrist()));
      opperator_BottomRightRearButton.onFalse(new InstantCommand(() -> wrist.stopWrist()));
    }
    // // ------------------------------------- EXTENDER
  }

  // /**************/
  // /*** AUTONS ***/
  // /**************/
  //
  /*
   * Velocity is negative when we are facing grid because the velocity (x-axis)
   * // * is field-centric meaning when looking down, left is negative and right
   * is
   * // postive.
   * //
   */

  public void configureAutons() {
    // final AutoFactory pathPlannerBuilder = new AutoFactory(arm, extender, wrist, claw, compressor, autoBase);


    SmartDashboard.putData("Autonomous: ", Autons);

    Autons.setDefaultOption("Do Nothing", new DoNothingAuton());

    // // ------------------------------------- CALIBRATE
    // Autons.addOption("CALI HARD PATH", createAuto("Cali Hard"));
    // Autons.addOption("CALI SIMPLE PATH", createAuto("Simple Cali"));
    // Autons.addOption("CALI BONKERS PATH", createAuto("Crazy Cali"));
    // // // -------------------------------------

    // Autons.addOption("High, Pickup, High", createAuto("pls work"));
    // Autons.addOption("High, Pickup, Engage", createAuto("Leave, Pickup,
    // Engage"));
    // Autons.addOption("HIGH_PICKUP", createAuto("HIGH_PICKUP"));
    // Autons.addOption("Blue High No Bump", createAuto("Blue High No Bump"));
    // Autons.addOption("Blue High No Bump test", createAuto("Blue High No Bump
    // test"));
    // Autons.addOption("Blue High BUMP", createAuto("Blue High BUMP"));
    // Autons.addOption("Blue Mid NO Bump", createAuto("Blue Mid NO Bump"));
    // Autons.addOption("Blue Mid Bump", createAuto("Blue Mid BUMP"));
    // Autons.addOption("mark", createAuto("mark"));

    /*
     * IMPORTANT!!!
     * IMPORTANT!!!
     * When ever calling a command preset (like the button ones), call the command
     * with new, example: new whateverCommand()...
     * Why?
     * The code break and gives startComp() error, SO MAKE SURE THAT THIS IS DONE
     */

    // TIMED BASED AUTONS//
    // pathPlannerBuilder.createAuto("mark");
    // Autons.addOption("Simple Cali", pathPlannerBuilder.createAuto("Simple Cali"));
    // Autons.addOption("BHB", pathPlannerBuilder.createAuto("Blue High BUMP"));


    Autons.addOption("[BLUE-LEFT] PLACE MID, DIP",
        new BLUE_LEFT_PLACE_MID_DIP(SwerveDrive, extender, arm, wrist, claw));

    Autons.addOption("Place High, DIP", new SequentialCommandGroup(
        new InstantCommand(() -> SwerveDrive.autoReverseGyro()),
        new CheckCompressor(compressor, 10),
        new InstantCommand(() -> wrist.setPosition(1)),
        new InstantCommand(() -> claw.closeClaw()),
        new WaitCommand(1),
        new stowAway(extender, arm, wrist),
        new highCommand(extender, arm, wrist),
        new WaitCommand(1),
        new InstantCommand(() -> wrist.setPosition(-29)),
        new WaitCommand(1),
        new InstantCommand(() -> claw.openClaw()),
        new ResetExtender(extender, 0),
        new stowAway(extender, arm, wrist),
        new WaitCommand(1),
        new DriveForward(SwerveDrive, 0, 23, 4) // Should be pos velo now because of
    ));

    Autons.addOption("[BLUE-RIGHT] PLACE MID, DIP",
        new BLUE_RIGHT_PLACE_MID_DIP(SwerveDrive, extender, arm, wrist, claw));

    Autons.addOption("THROW, DIP", new THROW_DIP(SwerveDrive, extender, arm, wrist, claw));

    Autons.addOption("MID, TURN, GRAB", new MID_TURN_GRAB(SwerveDrive, extender, arm, wrist, claw));

    Autons.addOption("[P] LOW CONE + MOBILITY + PickUp", new CONE_MOBILITY(SwerveDrive, extender, arm, wrist, claw));

    Autons.addOption("[2023-04-21][Q-77] low score, mobility, balance",
        new JUST_CHARGE_PAD(SwerveDrive, extender, arm, wrist));

    Autons.addOption("LOW_ENGAGE", new LOW_ENGAGE(SwerveDrive));

    Autons.addOption("CHARGE NO ENGAGE", new ChargeNoEngage(SwerveDrive));

    Autons.addOption("Low", new DriveForward(SwerveDrive, .5, -25, 1));

    Autons.addOption("ONLY HIGH", new SequentialCommandGroup(
        new InstantCommand(() -> SwerveDrive.autoReverseGyro()),
        new CheckCompressor(compressor, 10),
        new InstantCommand(() -> wrist.setPosition(1)),
        new InstantCommand(() -> claw.closeClaw()),
        new WaitCommand(1),
        new stowAway(extender, arm, wrist),
        new highCommand(extender, arm, wrist),
        new WaitCommand(1),
        new InstantCommand(() -> wrist.setPosition(-29)),
        new WaitCommand(2),
        new InstantCommand(() -> claw.openClaw()),
        new ResetExtender(extender, 0),
        new stowAway(extender, arm, wrist)));

    Autons.addOption("High ENGAGE", new SequentialCommandGroup(
        new CheckCompressor(compressor, 10),
        new InstantCommand(() -> wrist.setPosition(1)),
        new InstantCommand(() -> claw.closeClaw()),
        new WaitCommand(1),
        new stowAway(extender, arm, wrist),
        new highCommand(extender, arm, wrist),
        new WaitCommand(1),
        new InstantCommand(() -> wrist.setPosition(-29)),
        new WaitCommand(2),
        new InstantCommand(() -> claw.openClaw()),
        new ResetExtender(extender, 0),
        new stowAway(extender, arm, wrist),
        new JUST_CHARGE_PAD(SwerveDrive, extender, arm, wrist)));

    Autons.addOption("[TEST] MId, Mobility, Engage", new SequentialCommandGroup(
        new InstantCommand(() -> SwerveDrive.autoReverseGyro()),
        new CheckCompressor(compressor, 10),
        new InstantCommand(() -> wrist.setPosition(0)),
        new WaitCommand(.1),
        new InstantCommand(() -> claw.closeClaw()),
        new WaitCommand(.1),
        new stowAway(extender, arm, wrist),
        new midCommand(extender, arm, wrist),
        new WaitCommand(1.5),
        new InstantCommand(() -> wrist.setPosition(-29)),
        new WaitCommand(1),
        new InstantCommand(() -> claw.openClaw()),
        new ChargeNoBack(SwerveDrive, extender, arm, wrist)));

    // Autons.addOption("single", new Single(SwerveDrive, extender, arm, wrist,
    // claw));

    // Autons.addOption("high", new HIGH(SwerveDrive, extender, arm, wrist, claw));

    Autons.addOption("[PP] UNTESTED pls change",
        new PP_UNTESTED_PLS_CHANGE(SwerveDrive, extender, arm, wrist, claw));
  }

  // /**
  // * Use this to pass the autonomous command to the main {@link Robot} class.
  // *
  // * @return the command to run in autonomous
  // */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autons.getSelected();
  }

  // private List<EventMarker> getPathMarkers() {
  //   return PathPlanner.loadPath(pathFileName,
  //       new PathConstraints(pathConstraints[0], pathConstraints[1])).getMarkers();
  // }

  // private List<PathPlannerTrajectory> pathGroup() {
  //   return PathPlanner.loadPathGroup(pathFileName,
  //       new PathConstraints(pathConstraints[0], pathConstraints[1]));
  // }

  // private Command pathFollowingCommand() {
  //   return autoBuilder.fullAuto(pathGroup());
  // }

  // private void setPath(String path) {
  //   this.pathFileName = path;
  // }

  private HashMap<String, Command> getEventMap() {
    if (eventMap.isEmpty()) {
      eventMap();
    }
    return eventMap;
  }
  
  private void eventMap() {
    // eventMap.put("Safe", new stowAwayAuto(extender, arm, wrist));
    // eventMap.put("Run Comp", new InstantCommand(() -> compressor.enableAnalog(100, 120)));
    // eventMap.put("Fix Wrist", new InstantCommand(() -> wrist.setPosition(5)));
    // eventMap.put("Cone up", new groundConeCommand(extender, arm, wrist, claw));
    // eventMap.put("Cube Pickup", new groundCubeCommand(extender, arm, wrist, claw));
    // eventMap.put("High", new PlaceHigh(extender, arm, wrist, claw, compressor));
    // eventMap.put("Mid", new PlaceMid(extender, arm, wrist, claw, compressor));
    // eventMap.put("Throw Cone", new InstantCommand(() -> claw.outTakeCone()));
    // eventMap.put("Throw Cube", new InstantCommand(() -> claw.outTakeCube()));
    // eventMap.put("Cone Claw", new InstantCommand(() -> claw.coneIntake()));
    // eventMap.put("Cube Claw", new InstantCommand(() -> claw.cubeIntake()));
    // eventMap.put("Stop Claw", new InstantCommand(() -> claw.stopClaw()));
  }

  // public Command createAuto(String path) {
  //   eventMap();
  //   setPath(path);
  //   return new FollowPathWithEvents(pathFollowingCommand(), getPathMarkers(), eventMap);
  // }
}
// CODE GOBLIN STRIKES AGAIN