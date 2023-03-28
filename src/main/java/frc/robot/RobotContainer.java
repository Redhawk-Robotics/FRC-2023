// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autons.DoNothingAuton;
import frc.robot.commands.Swerve.Drive;
import frc.robot.commands.Swerve.DriveForward;
import frc.robot.commands.arm.ArmSetPoint;
import frc.robot.constants.Ports;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.modules.CompressorModule;
import frc.robot.subsystems.modules.PDH;
import frc.robot.test.testWhatever;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

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

  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /* Subsystems */
  private final SwerveSubsystem SwerveDrive = new SwerveSubsystem();
  private final PDH powerDistributionHub = new PDH();
  private final testWhatever testers = new testWhatever();
  // private final ArmSubsystem armSubsystem = new ArmSubsystem();
  // private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  // private final extenderSubsystem extenderSubsystem = new extenderSubsystem();
  // private final WristSubsystem wristSubsystem = new WristSubsystem();

  // private final WristSubsystem wristSubsystem = new WristSubsystem();

  private final CompressorModule compressor = CompressorModule.getCompressorModule(); // this needs to be put into a
                                                                                      // PneumaticsSubsystem
  // private final ClawSubsystem claw = new ClawSubsystem();

  // private PneumaticHub compressor = new PneumaticHub(1);

  /* Commands */

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
  private final Trigger driver_Y_robotCentric = new JoystickButton(DRIVER, XboxController.Button.kY.value);

  private final Trigger driver_slowSpeed_rightBumper = new JoystickButton(DRIVER, XboxController.Button.kRightBumper.value);

  // Additional buttons
  private final Trigger driver_leftBump_lock = new JoystickButton(DRIVER, XboxController.Button.kLeftBumper.value);

  private final Trigger driver_B = new JoystickButton(DRIVER, XboxController.Button.kB.value);

  private final Trigger driver_X = new JoystickButton(DRIVER, XboxController.Button.kX.value);

  private final Trigger driver_startButton1 = new JoystickButton(DRIVER, XboxController.Button.kStart.value);
  private final Trigger driver_BackButton1 = new JoystickButton(DRIVER, XboxController.Button.kBack.value);

  private final Trigger driver_LeftStickButton1 = new JoystickButton(DRIVER, XboxController.Button.kLeftStick.value);
  private final Trigger driver_RightStickButton1 = new JoystickButton(DRIVER, XboxController.Button.kRightStick.value);

  // Controller two - Operator
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

  private final Trigger opperator_RightBumper = new JoystickButton(OPERATOR, XboxController.Button.kRightBumper.value);
  private final Trigger opperator_leftBumper = new JoystickButton(OPERATOR, XboxController.Button.kLeftBumper.value);

  private final Trigger opperator_startButton = new JoystickButton(OPERATOR, XboxController.Button.kStart.value);
  private final Trigger opperator_BackButton = new JoystickButton(OPERATOR, XboxController.Button.kBack.value);

  private final Trigger opperator_LeftStick = new JoystickButton(OPERATOR, XboxController.Button.kLeftStick.value);
  private final Trigger opperator_RightStick = new JoystickButton(OPERATOR, XboxController.Button.kRightStick.value);

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
            () -> driver_Y_robotCentric.getAsBoolean(),
            () -> driver_slowSpeed_rightBumper.getAsBoolean()));

    // -------------------------------------

    /*************/
    /*** ARM ***/
    /*************/

    // up, down
    // armSubsystem.setDefaultCommand(
    //     new ArmManual(
    //         armSubsystem,
    //         () -> opperator_A.getAsBoolean(),
    //         () -> opperator_Y.getAsBoolean()));

    // -------------------------------------

    /*************/
    /*** CLAW ***/
    /*************/

    // METHODS: coneIntake, cubeIntake, leftOutTake, rightOutTake
    // clawSubsystem.setDefaultCommand(
    //     new ClawManual(
    //         clawSubsystem,
    //         () -> opperator_X.getAsBoolean(),
    //         () -> opperator_B.getAsBoolean(),
    //         () -> opperator_leftBumper.getAsBoolean(),
    //         () -> opperator_RightBumper.getAsBoolean()));

    // -------------------------------------

    /*************/
    /*** EXTENDER ***/
    /*************/

    // METHODS: extend, retract
    // extenderSubsystem.setDefaultCommand(
    //     new ExtenderManual(
    //         extenderSubsystem,
    //         () -> opperator_startButton.getAsBoolean(),
    //         () -> opperator_BackButton.getAsBoolean()));

    // -------------------------------------

    /*************/
    /*** WRIST ***/
    /*************/

    // METHODS: up, down
    // wristSubsystem.setDefaultCommand(
    //     new WristManual(
    //         wristSubsystem,
    //         () -> driver_X.getAsBoolean(),
    //         () -> driver_B.getAsBoolean()));

    // -------------------------------------

    // Configure the trigger bindings, defaults, Autons
    configureDefaultCommands();
    configureButtonBindings();
    configureAutons();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  /****************/
  /*** DEFAULTS ***/
  /****************/

  private void configureDefaultCommands() {
    //Compressor code
    compressor.enableAnalog(100, 115);
    // compressor.disableCompressor();

    //Camera Server
    // CameraServer.startAutomaticCapture();
  }

  /***************/
  /*** BUTTONS ***/
  /***************/

  private void configureButtonBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    // ------------------------------------- GYRO
    driver_A_zeroGyro.onTrue(new InstantCommand(() -> SwerveDrive.zeroGyro()));// A value for the Xbox Controller

    // ------------------------------------- ARM
    opperator_Y.whileTrue(new InstantCommand(() -> testers.upGoArm()));
    opperator_A.whileTrue(new InstantCommand(() -> testers.downGoArm()));

    opperator_Y.whileFalse(new InstantCommand(() -> testers.stopArm()));
    opperator_A.whileFalse(new InstantCommand(() -> testers.stopArm()));

    // ------------------------------------- CLAW
    opperator_X.onTrue(new InstantCommand(() -> testers.coneIntake()));
    opperator_B.onTrue(new InstantCommand(() -> testers.cubeIntake()));

    opperator_X.whileFalse(new InstantCommand(() -> testers.stopClaw()));
    opperator_B.whileFalse(new InstantCommand(() -> testers.stopClaw()));

    opperator_leftBumper.onFalse(new InstantCommand(() -> testers.outTake()));
    opperator_RightBumper.onFalse(new InstantCommand(() -> testers.outTake()));

    // ------------------------------------- WRIST
    driver_X.onTrue(new InstantCommand(() -> testers.upGoWrist()));
    driver_X.onFalse(new InstantCommand(() -> testers.stopWrist()));

    driver_B.onTrue(new InstantCommand(() -> testers.downGoWrist()));
    driver_B.onFalse(new InstantCommand(() -> testers.stopWrist()));

    // ------------------------------------- EXTENDER
    opperator_startButton.onTrue(new InstantCommand(() -> testers.upExtender()));
    opperator_startButton.onFalse(new InstantCommand(() -> testers.stopExtender()));

    opperator_BackButton.onTrue(new InstantCommand(() -> testers.downExtender()));
    opperator_BackButton.onFalse(new InstantCommand(() -> testers.stopExtender()));
    
  }

  /**************/
  /*** AUTONS ***/
  /**************/
  public void configureAutons() {
    SmartDashboard.putData("Autonomous: ", Autons);

    Autons.setDefaultOption("Do Nothing", new DoNothingAuton());

    Autons.addOption("SPPLI2 Simple Auton", SwerveDrive.followTraj(
        PathPlanner.loadPath(
            "New New Path",
            new PathConstraints(
                5,
                5)),
        true));

    Autons.addOption("Drop Cube, Leave Community FAR", new SequentialCommandGroup(
        new DriveForward(SwerveDrive, .5, -30, 1),
        new WaitCommand(1),
        new DriveForward(SwerveDrive, 4, 15, 7)));

    Autons.addOption("[UNSTABLE] drop cube, charge pad", new SequentialCommandGroup(
        new DriveForward(SwerveDrive, .5, -30, 1),
        new WaitCommand(1),
        new DriveForward(SwerveDrive, 0, 35, 2), // tilt charge pad
        new DriveForward(SwerveDrive, 0, 20, 3), // continue past charge pad
        new WaitCommand(1),
        new DriveForward(SwerveDrive, 0, -35, 4) // reverse back onto the pad
    ));

    // HAS NO REVERSE TO HIT THE GRID
    Autons.addOption("[UNSTABLE] charge pad PRACTICE", new SequentialCommandGroup(
        new DriveForward(SwerveDrive, 0, 35, 2), // tilt charge pad
        new DriveForward(SwerveDrive, 0, 20, 3), // continue past charge pad
        new WaitCommand(1),
        new DriveForward(SwerveDrive, 0, -35, 4) // reverse back onto the pad
    ));

    Autons.addOption("armSetpoint", new SequentialCommandGroup(
      new ArmSetPoint(testers, 20)
    ));

    // Autons.addOption("GO ON CHARGE PAD", new SequentialCommandGroup(
    // new DriveForward(SwerveDrive, .5, -30, 1),
    // new WaitCommand(1),
    // new DriveForward(SwerveDrive, 4, 20, 4)));
    // Autons.addOption("mobility", new mobilitytest());
    // Autons.addOption("AutoBalance", new TestPathPlannerAuton());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autons.getSelected();
  }
}
