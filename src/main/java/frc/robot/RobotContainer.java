// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autons.DoNothingAuton;
import frc.robot.commands.Autons.TimedBased.CONE_MOBILITY;
import frc.robot.commands.Autons.TimedBased.CONE_MOBILITY_PICKUP;
import frc.robot.commands.Autons.TimedBased.JUST_CHARGE_PAD;
import frc.robot.commands.Autons.TimedBased.LOW_ENGAGE;
import frc.robot.commands.Autons.TimedBased.MIDSCORE;
import frc.robot.commands.Positions.highCommand;
import frc.robot.commands.Swerve.Drive;
import frc.robot.commands.Swerve.DriveForward;
import frc.robot.constants.Ports;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.extenderSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
        // private final LEDController LEDS = new LEDController();

        private final WristSubsystem wristSubsystem = new WristSubsystem();
        private final ArmSubsystem arm = new ArmSubsystem();
        private final extenderSubsystem extender = new extenderSubsystem();
        private final ClawSubsystem claw = new ClawSubsystem();

        private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

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
                // Compressor code
                compressor.enableAnalog(100, 120);
                // LEDS.setColor(LEDColor.BEAT,0);

                // compressor.disableCompressor();

                // Camera Server
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
                // Driver
                driver_A_zeroGyro.onTrue(new InstantCommand(() -> SwerveDrive.zeroGyro()));//

                // Operator
                opperator_B.onTrue(midCommand());

                // opperator_X.onTrue(highCommand());//works
                opperator_X.onTrue(new highCommand(extender, arm, wristSubsystem));

                // opperator_B.whileFalse(new SequentialCommandGroup(
                // stoweAway(),
                // new InstantCommand(() -> claw.stopClaw())));

                opperator_Y.onTrue(substationCommand()); // - good
                // opperator_Y.whileFalse(stoweAway());
                opperator_Y.whileFalse(new InstantCommand(() -> claw.stopClaw()));// - good

                opperator_A.onTrue(groundCommand()); // - good
                opperator_A.whileTrue(stoweAway()); // FIME IDK IF ITS GOOD ---------------------------------------

                opperator_A.whileFalse(new InstantCommand(() -> claw.stopClaw()));// - good

                // ------------------------------------- ARM
                // opperator_Y.whileTrue(new InstantCommand(() -> testers.upGoArm()));
                // opperator_A.whileTrue(new InstantCommand(() -> testers.downGoArm()));

                // opperator_Y.whileFalse(new InstantCommand(() -> testers.stopArm()));
                // opperator_A.whileFalse(new InstantCommand(() -> testers.stopArm()));

                // // Stoeaway
                opperator_leftBumper.whileTrue(stoweAway());
                opperator_leftBumper.whileFalse(new InstantCommand(() -> claw.stopClaw()));

                opperator_RightBumper.whileTrue((shootLow()));
                opperator_RightBumper.whileFalse(new InstantCommand(() -> claw.stopClaw()));

                // // ------------------------------------- CLAW
                opperator_TopLeftRearButton.onTrue(new InstantCommand(() -> claw.coneIntake()));
                opperator_BottomLeftRearButton.onTrue(new InstantCommand(() -> claw.outTake1()));

                opperator_TopLeftRearButton.onFalse(new InstantCommand(() -> claw.stopClaw()));
                opperator_BottomLeftRearButton.onFalse(new InstantCommand(() -> claw.stopClaw()));

                // // ------------------------------------- WRIST
                opperator_TopRightRearButton.onTrue(new InstantCommand(() -> wristSubsystem.upGoWrist()));
                opperator_TopRightRearButton.onFalse(new InstantCommand(() -> wristSubsystem.stopWrist()));

                opperator_BottomRightRearButton.onTrue(new InstantCommand(() -> wristSubsystem.downGoWrist()));
                opperator_BottomRightRearButton.onFalse(new InstantCommand(() -> wristSubsystem.stopWrist()));

                // // ------------------------------------- EXTENDER

                // // opperator_BackButton.onTrue(new InstantCommand(() ->
                // testers.upExtender()));
                // // opperator_BackButton.onFalse(new InstantCommand(() ->
                // // testers.stopExtender()));

                // // opperator_startButton.onTrue(new InstantCommand(() ->
                // // testers.downExtender()));
                // // opperator_startButton.onFalse(new InstantCommand(() ->
                // // testers.stopExtender()));

        }

        // /**************/
        // /*** AUTONS ***/
        // /**************/
        public void configureAutons() {
                SmartDashboard.putData("Autonomous: ", Autons);

                Autons.setDefaultOption("Do Nothing", new DoNothingAuton());

                // /* Velocity is negative when we are facing grid because the velocity (x-axis)
                // * is field-centric meaning when looking down, left is negative and right is
                // postive.
                // */

                Autons.addOption("TEST", new SequentialCommandGroup(
                                new InstantCommand(() -> claw.closeClaw()),
                                new InstantCommand(() -> wristSubsystem.setPosition(5)),
                                new WaitCommand(.5),
                                highCommand(),
                                new WaitCommand(1),
                                new InstantCommand(() -> wristSubsystem.setPosition(-30)),

                                stoweAway(),
                                new DriveForward(SwerveDrive, .5, -8, 5)));

                Autons.addOption(" CONE_MOBILITY", new CONE_MOBILITY(SwerveDrive));

                // Autons.addOption("CONE_MOBILITY_PICKUP", new
                // CONE_MOBILITY_PICKUP(SwerveDrive, claw,
                // new groundIntake(extender, arm, wristSubsystem), new stowAway(extender, arm,
                // wristSubsystem)));

                Autons.addOption("JUST_CHARGE_PAD", new JUST_CHARGE_PAD(SwerveDrive));
                Autons.addOption("LOW_ENGAGE", new LOW_ENGAGE(SwerveDrive));

                // Autons.addOption("MIDSCORE", new MIDSCORE(extender, arm, wristSubsystem,
                // claw, new
                // stowAway(extender, arm, wristSubsystem)));
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

        public Command autoSubstationCommand() {
                return new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                                new InstantCommand(() -> wristSubsystem.setPosition(6)),
                                                new InstantCommand(() -> extender.setPosition(0))),
                                new WaitCommand(1),
                                new InstantCommand(() -> arm.setPosition(44)),
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> extender.setPosition(0)),
                                                new InstantCommand(() -> wristSubsystem.setPosition(-30)),
                                                new InstantCommand(() -> arm.setPosition(44))));
        }

        public Command substationCommand() {
                return new SequentialCommandGroup(
                                new InstantCommand(() -> wristSubsystem.setPosition(6)),
                                new InstantCommand(() -> arm.setPosition(44)),
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> extender.setPosition(0)),
                                                new InstantCommand(() -> wristSubsystem.setPosition(-24)),

                                                new InstantCommand(() -> arm.setPosition(44))),
                                new InstantCommand(() -> claw.coneIntake()));
        }

        public Command stoweAway() {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> extender.setPosition(0)),
                                                new InstantCommand(() -> wristSubsystem.setPosition(6))),
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> arm.setPosition(0)),
                                                new InstantCommand(() -> wristSubsystem.setPosition(6)),
                                                new InstantCommand(() -> extender.setPosition(0))));
        }

        public Command groundCommand() {
                return new SequentialCommandGroup(
                                new InstantCommand(() -> extender.setPosition(0)),

                                new InstantCommand(() -> arm.setPosition(12)),
                                new ParallelDeadlineGroup(

                                                new InstantCommand(() -> extender.setPosition(0)),

                                                new InstantCommand(() -> claw.coneIntake())),
                                new WaitCommand(
                                                1),
                                new InstantCommand(() -> wristSubsystem.setPosition(-20))

                );
        }

        public Command midCommand() {
                return new SequentialCommandGroup(
                                new InstantCommand(() -> wristSubsystem.setPosition(6)),
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> extender.setPosition(0)),
                                                new InstantCommand(() -> wristSubsystem.setPosition(6))),

                                new InstantCommand(() -> arm.setPosition(38)),
                                new InstantCommand(() -> extender.setPosition(0)));
        }

        public Command highCommand() {
                return new SequentialCommandGroup(
                                new InstantCommand(() -> wristSubsystem.setPosition(6)),
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> extender.setPosition(0)),
                                                new InstantCommand(() -> wristSubsystem.setPosition(6))),

                                new InstantCommand(() -> arm.setPosition(45)),
                                new InstantCommand(() -> extender.setPosition(0))

                );
        }

        public Command shootLow() {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> extender.setPosition(0)),
                                                new InstantCommand(() -> wristSubsystem.setPosition(6))),
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> arm.setPosition(0)),
                                                new InstantCommand(() -> wristSubsystem.setPosition(6)),
                                                new InstantCommand(() -> extender.setPosition(0)),
                                                new InstantCommand(() -> claw.outTake())));
        }

        public Command singleSubstation() {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> extender.setPosition(0)),
                                                new InstantCommand(() -> wristSubsystem.setPosition(6))),
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> arm.setPosition(0)),
                                                new InstantCommand(() -> wristSubsystem.setPosition(6)),
                                                new InstantCommand(() -> extender.setPosition(0)),
                                                new InstantCommand(() -> claw.coneIntake())));
        }
}
