// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autons.AutoBase;
import frc.robot.commands.Autons.DoNothingAuton;
import frc.robot.commands.Autons.TimedBased.BLUE_LEFT_PLACE_MID_DIP;
import frc.robot.commands.Autons.TimedBased.BLUE_RIGHT_PLACE_MID_DIP;
import frc.robot.commands.Autons.TimedBased.CONE_MOBILITY;
import frc.robot.commands.Autons.TimedBased.HIGH;
import frc.robot.commands.Autons.TimedBased.JUST_CHARGE_PAD;
import frc.robot.commands.Autons.TimedBased.LOW_ENGAGE;
import frc.robot.commands.Autons.TimedBased.MID_TURN_GRAB;
import frc.robot.commands.Autons.TimedBased.PP_UNTESTED_PLS_CHANGE;
import frc.robot.commands.Autons.TimedBased.Single;
import frc.robot.commands.Autons.TimedBased.THROW_DIP;
import frc.robot.commands.Positions.CheckCompressor;
import frc.robot.commands.Positions.doubleSubstation;
import frc.robot.commands.Positions.groundConeCommand;
import frc.robot.commands.Positions.groundCubeCommand;
import frc.robot.commands.Positions.highAuto;
import frc.robot.commands.Positions.highCommand;
import frc.robot.commands.Positions.midCommand;
import frc.robot.commands.Positions.shootCube;
import frc.robot.commands.Positions.shootCone;
import frc.robot.commands.Positions.singleSubstation;
import frc.robot.commands.Positions.stowAway;
import frc.robot.commands.Swerve.Drive;
import frc.robot.commands.Swerve.DriveForward;
import frc.robot.commands.Swerve.DriveTurn;
import frc.robot.commands.Swerve.autoAligner;
import frc.robot.constants.Ports;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.extenderSubsystem;
import frc.robot.subsystems.modules.AutoFactory;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  /* Subsystems */
  private final SwerveSubsystem SwerveDrive = new SwerveSubsystem();
  // private final Limelight Limelight = new Limelight();
  // private final LEDController LEDS = new LEDController();

  private final WristSubsystem wrist = new WristSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final extenderSubsystem extender = new extenderSubsystem();
  private final ClawSubsystem claw = new ClawSubsystem();

  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  private final AutoBase autoBase = new AutoBase(SwerveDrive);
  private final AutoFactory pathPlannerBuilder = new AutoFactory(arm, extender, wrist, claw, compressor, autoBase);

  // Positions
  private final groundConeCommand groundCone = new groundConeCommand(extender, arm, wrist, claw);
  private final groundCubeCommand groundCube = new groundCubeCommand(extender, arm, wrist, claw);

  private final highCommand highCommand = new highCommand(extender, arm, wrist);
  private final midCommand midCommand = new midCommand(extender, arm, wrist);

  private final shootCone shootCone = new shootCone(extender, arm, wrist, claw);
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

  private final Trigger driver_TopRightRearButton = new JoystickButton(DRIVER,
      XboxController.Button.kRightStick.value);

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
  private final Trigger opperator_TopRightRearButton = new JoystickButton(OPERATOR,
      XboxController.Button.kRightStick.value);

  // Create SmartDashboard chooser for autonomous routines
  private static SendableChooser<Command> Autons = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
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

    // wristSubsystem.setDefaultCommand(
    // new WristManual(wristSubsystem,
    // () -> OPERATOR.getRawAxis(leftYAxis2) * .5));

    /*************/
    /*** EXTENDER ***/
    /*************/

    // METHODS: extend, retract
    // extender.setDefaultCommand(
    // new ExtenderManual(
    // extender,
    // () -> OPERATOR.getRawAxis(rightYAxis2)));

    // -------------------------------------

    // Configure the trigger bindings, defaults, Autons
    configureDefaultCommands();
    configureButtonBindings();
    configureAutons();
  }

  /****************/
  /*** DEFAULTS ***/
  /****************/

  private void configureDefaultCommands() {
    // Compressor code
    compressor.enableAnalog(100, 120);

    // compressor.disableCompressor();

    // Camera Server
    // CameraServer.startAutomaticCapture();
  }

  /***************/
  /*** BUTTONS ***/
  /***************/

  private void configureButtonBindings() {
    // Driver
    driver_A_zeroGyro.onTrue(new InstantCommand(() -> SwerveDrive.zeroGyro()));
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
    opperator_Y.whileFalse(new InstantCommand(() -> claw.stopClaw()));

    opperator_A.onTrue(stowAway);
    opperator_A.whileFalse(new InstantCommand(() -> claw.stopClaw()));

    // ------------------------------------- ARM
    opperator_leftBumper.whileTrue(shootCone);
    opperator_leftBumper.whileFalse(new InstantCommand(() -> claw.stopClaw()));

    opperator_RightBumper.whileTrue(shootCube);
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
    SwerveAutoBuilder autoBuilder = autoBase.CustomSwerveAutoBuilder();

    SmartDashboard.putData("Autonomous: ", Autons);

    Autons.setDefaultOption("Do Nothing", new DoNothingAuton());

    // // ------------------------------------- CALIBRATE
    Autons.addOption("CALI HARD PATH", pathPlannerBuilder.createAuto("Cali"));

    Autons.addOption("CALI SIMPLE PATH", pathPlannerBuilder.createAuto("Test"));
    // // -------------------------------------

    Autons.addOption("High, Pickup, High", pathPlannerBuilder.createAuto("pls work"));
    Autons.addOption("High, Pickup, Engage", pathPlannerBuilder.createAuto("Leave, Pickup, Engage"));
    Autons.addOption("HIGH_PICKUP", pathPlannerBuilder.createAuto("HIGH_PICKUP"));

    /*
     * IMPORTANT!!!
     * IMPORTANT!!!
     * When ever calling a command preset (like the button ones), call the command
     * with new, example: new whateverCommand()...
     * Why?
     * The code break and gives startComp() error, SO MAKE SURE THAT THIS IS DONE
     */

    // TIMED BASED AUTONS//
    Autons.addOption("[BLUE-LEFT] PLACE MID, DIP",
        new BLUE_LEFT_PLACE_MID_DIP(SwerveDrive, extender, arm, wrist, claw));

    Autons.addOption("[BLUE-RIGHT] PLACE MID, DIP",
        new BLUE_RIGHT_PLACE_MID_DIP(SwerveDrive, extender, arm, wrist, claw));

    Autons.addOption("THROW, DIP", new THROW_DIP(SwerveDrive, extender, arm, wrist, claw));

    Autons.addOption("MID, TURN, GRAB", new MID_TURN_GRAB(SwerveDrive, extender, arm, wrist, claw));

    Autons.addOption("[P] LOW CONE + MOBILITY", new CONE_MOBILITY(SwerveDrive));

    Autons.addOption("JUST_CHARGE_PAD", new JUST_CHARGE_PAD(SwerveDrive));

    Autons.addOption("LOW_ENGAGE", new LOW_ENGAGE(SwerveDrive));

    Autons.addOption("single", new Single(SwerveDrive, extender, arm, wrist, claw));

    Autons.addOption("high", new HIGH(SwerveDrive, extender, arm, wrist, claw));

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
}
