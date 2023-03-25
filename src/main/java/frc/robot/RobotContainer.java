// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.Autons.DoNothingAuton;
import frc.robot.commands.Autons.mobility;
import frc.robot.commands.Autons.mobilitytest;
import frc.robot.commands.Swerve.Drive;
import frc.robot.commands.Swerve.DriveForward;
import frc.robot.commands.wrist.WristManual;
import frc.robot.commands.wrist.WristSetPoint;
import frc.robot.constants.Ports;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.modules.CompressorModule;
import frc.robot.subsystems.modules.PDH;
import frc.robot.test.testClawMotors;
import frc.robot.test.testWhatever;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.cameraserver.CameraServer;
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
  private final Trigger zeroGyro = new JoystickButton(DRIVER, XboxController.Button.kA.value);
  private final Trigger robotCentric = new JoystickButton(DRIVER, XboxController.Button.kY.value);

  private final Trigger slowSpeed = new JoystickButton(DRIVER, XboxController.Button.kRightBumper.value);// slowspeed
                                                                                                         // for right
                                                                                                         // side of the
                                                                                                         // controller

  // Additional buttons
  private final Trigger lock = new JoystickButton(DRIVER, XboxController.Button.kLeftBumper.value);

  private final Trigger bButton1 = new JoystickButton(DRIVER, XboxController.Button.kB.value);

  private final Trigger xButton1 = new JoystickButton(DRIVER, XboxController.Button.kX.value);

  private final Trigger startButton1 = new JoystickButton(DRIVER, XboxController.Button.kStart.value);
  private final Trigger BackButton1 = new JoystickButton(DRIVER, XboxController.Button.kBack.value);

  private final Trigger LeftStickButton1 = new JoystickButton(DRIVER, XboxController.Button.kLeftStick.value);
  private final Trigger RightStickButton1 = new JoystickButton(DRIVER, XboxController.Button.kRightStick.value);

  // Controller two - Operator
  private final int leftYAxis2 = XboxController.Axis.kLeftY.value;
  private final int leftXAxis2 = XboxController.Axis.kLeftX.value;

  private final int rightYAxis2 = XboxController.Axis.kRightY.value;
  private final int rightXAxis2 = XboxController.Axis.kRightX.value;

  private final int leftTrigger2 = XboxController.Axis.kLeftTrigger.value;
  private final int rightTrigger2 = XboxController.Axis.kRightTrigger.value;

  private final Trigger Abutton2 = new JoystickButton(OPERATOR, XboxController.Button.kA.value);
  private final Trigger Bbutton2 = new JoystickButton(OPERATOR, XboxController.Button.kB.value);
  private final Trigger Xbutton2 = new JoystickButton(OPERATOR, XboxController.Button.kX.value);
  private final Trigger Ybutton2 = new JoystickButton(OPERATOR, XboxController.Button.kY.value);

  private final Trigger RightBumper2 = new JoystickButton(OPERATOR, XboxController.Button.kRightBumper.value);
  private final Trigger leftBumper2 = new JoystickButton(OPERATOR, XboxController.Button.kLeftBumper.value);

  private final Trigger startButton2 = new JoystickButton(OPERATOR, XboxController.Button.kStart.value);
  private final Trigger BackButton2 = new JoystickButton(OPERATOR, XboxController.Button.kBack.value);

  private final Trigger LeftStickButton2 = new JoystickButton(OPERATOR, XboxController.Button.kLeftStick.value);
  private final Trigger RightStickButton2 = new JoystickButton(OPERATOR, XboxController.Button.kRightStick.value);

  // Create SmartDashboard chooser for autonomous routines
  private static SendableChooser<Command> Autons = new SendableChooser<>();

  // THIS BUTTON IS ALREADY ASSIGNED
  // private final Trigger test = new JoystickButton(DRIVER,
  // XboxController.Button.kX.value);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    SwerveDrive.setDefaultCommand(
        new Drive(
            SwerveDrive,
            () -> -DRIVER.getRawAxis(translationAxis),
            () -> -DRIVER.getRawAxis(strafeAxis),
            () -> -DRIVER.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean(),
            () -> slowSpeed.getAsBoolean()));

    // Configure the trigger bindings, defaults, Autons

    // wristSubsystem.setDefaultCommand(
    // new WristManual(wristSubsystem,
    // () -> OPERATOR.getRawAxis(leftYAxis2)));

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
    // compressor.enableCompressorAnalog(0, 120); //TODO try minpressure 100
    compressor.enableAnalog(100, 120);
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

    // DRVIER.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    zeroGyro.onTrue(new InstantCommand(() -> SwerveDrive.zeroGyro()));// A value for the Xbox Controller

    // lock.onTrue(new InstantCommand(() -> SwerveDrive.Lock()));//try this line if
    // not add whiletrue
    // lock.onTrue(new RepeatCommand(new InstantCommand(() -> SwerveDrive.Lock())));

    Abutton2.whileTrue(new InstantCommand(() -> testers.upGoArm()));
    Abutton2.whileFalse(new InstantCommand(() -> testers.stopArm()));

    Ybutton2.whileTrue(new InstantCommand(() -> testers.downGoArm()));
    Ybutton2.whileFalse(new InstantCommand(() -> testers.stopArm()));

    // Abutton2.onTrue(new InstantCommand(() -> testers.upGoClaw()));
    // Abutton2.onFalse(new InstantCommand(() -> testers.stopClaw()));

    // Bbutton2.onTrue(new InstantCommand(() -> testers.downGoClaw()));
    // Bbutton2.onFalse(new InstantCommand(() -> testers.stopClaw()));

    // solenoids

    // test.toggleOnTrue(new InstantCommand(() -> testers.coneIntake()));

    Xbutton2.onTrue(new InstantCommand(() -> testers.coneIntake()));
    Xbutton2.onFalse(new InstantCommand(()-> testers.stopClaw()));
    RightBumper2.onFalse(new InstantCommand(() -> testers.outTake()));

    Bbutton2.onTrue(new InstantCommand(() -> testers.cubeIntake()));
    Bbutton2.onFalse(new InstantCommand(()-> testers.stopClaw()));

    leftBumper2.onFalse(new InstantCommand(() -> testers.outTake()));

    xButton1.onTrue(new InstantCommand(() -> testers.upGoWrist()));
    xButton1.onFalse(new InstantCommand(() -> testers.stopWrist()));

    bButton1.onTrue(new InstantCommand(() -> testers.downGoWrist()));
    bButton1.onFalse(new InstantCommand(() -> testers.stopWrist()));

    // // extender and wrist needs another button to go back
    startButton2.onTrue(new InstantCommand(() -> testers.upExtender()));
    startButton2.onFalse(new InstantCommand(() -> testers.stopExtender()));

    BackButton2.onTrue(new InstantCommand(() -> testers.downExtender()));
    BackButton2.onFalse(new InstantCommand(() -> testers.stopExtender()));

    // BackButton1.onTrue(new InstantCommand(()-> compressor.disableCompressor()));

    // LeftStickButton2.whileFalse((new InstantCommand(()-> compressor.disable())));
    // startButton1.onTrue(new InstantCommand(()-> new WristSetPoint(wristSubsystem,
    // 0.5)));//try out cmd
  }

  public void DefaultClawMotorCommand() {

    // testClawMotors.setDefaultCommand(new DefaultArmCommand(testClawMotors,
    // OPERATOR));
  }

  /**************/
  /*** AUTONS ***/
  /**************/

  // testing this

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
    Autons.addOption("HARDCODED PATH", new SequentialCommandGroup(
        new DriveForward(SwerveDrive, .5, -30, 1),
        new WaitCommand(1),
        new DriveForward(SwerveDrive, 4, 15, 3.5)
    // new WaitCommand(1),
    // new DriveForward(SwerveDrive, 2, -15, 2)
    ));
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
