package frc.robot.subsystems.modules;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.constants.Setting;
import frc.robot.lib.SwerveModuleConstants;
import frc.robot.lib.math.OnboardModuleState;
import frc.robot.lib.util.CANCoderUtil;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANCoderUtil.CCUsage;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

public class SwerveModule {
    /**
     * @param moduleNumber           The # of the swerveDrive modules
     * 
     * @param angleOffset            The offset of the swerveDrive modules
     * @param lastAngle              In degrees, the last angle that was inputted
     *                               within the code
     * 
     * @param driveMotor             Sparkmax motor for drive
     * @param angleMotor             Sparkmax motor for the Angle
     * 
     * @param driveEncoder           Encoder of the driveMotor
     * @param integratedAngleEncoder Encoder of the AngleMotor
     * 
     * @param angleEncoder           CanCoder for angleMotor desired Location
     * 
     * @param driveController        PID for the Drive Motor speed
     * @param angleController        PID for the Optimal angle for the angleMotor
     */
    public int moduleNumber;

    public Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder integratedAngleEncoder;

    private final CANCoder angleEncoder;

    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController angleController;

    private double chassisAngularOffset = 0;

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    // Creating a FeedForward setting for a reference tracking for the desired
    // output of the motor
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Setting.driveKS, Setting.driveKV, Setting.driveKA);// FIXME
                                                                                                                       // find
                                                                                                                       // values

    // Creating the swerveModules of the number and the constants
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        // Lastangle of each swerveModules
        lastAngle = getState().angle;
    }

    public CANSparkMax getDriverMotor() {
        return driveMotor;
    }

    /**
     * try the set desired states
     */
    // Creates desiredState of the Swervemodules which is WPILIB solution for
    // swerveDrive kinematics
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous
        // controller which
        // REV and CTRE are not
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredStateOld(SwerveModuleState desiredState) {// switch if not desired for driving
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(angleEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        driveController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        angleController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        m_desiredState = desiredState;
    }

    // Method to reset the CanCoder in order to get the AbsolutePosition
    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    // Method for the Config of the AngleEncoder
    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    // Method of the Config of the Angle Motor
    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(Setting.angleContinousCurentLimit);
        angleMotor.setInverted(Setting.angleMotorInvert);
        angleMotor.setIdleMode(Setting.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Setting.angleConversionFactor);
        angleController.setP(Setting.angleKP);
        angleController.setI(Setting.angleKI);
        angleController.setD(Setting.angleKD);
        angleController.setFF(Setting.angleKFF);
        angleMotor.enableVoltageCompensation(Setting.voltageComp);
        angleMotor.burnFlash();
        Timer.delay(1);
        resetToAbsolute();
    }

    // Method of the Config of the Drive Motor
    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(Setting.driveContinousCurrentLimit);
        driveMotor.setIdleMode(Setting.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Setting.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Setting.driveConversionPositionFactor);
        driveController.setP(Setting.driveKP);
        driveController.setI(Setting.driveKI);
        driveController.setD(Setting.driveKD);
        driveController.setFF(Setting.driveKFF);
        driveMotor.enableVoltageCompensation(Setting.voltageComp);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    // Method of the desired or inputted speed of the SwerveDrive
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Setting.maxVelocityMetersPerSecond;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedForward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    // Method in order to get the Angle for the desired State of the SwerveDrive
    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Setting.maxVelocityMetersPerSecond * 0.01))
                ? lastAngle
                : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    // Method in order to get the Angle of the Encoder value within the Angle Motor
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    // Method in order to get the Angle of the Encoder value within the CanCoder
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    // Method in order to get the States of the SwerveDrive module of the velocity
    // and angle of the Drive Motor
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    // SmartDashBoard information of the Encoder values of the Integrated and
    // CanCoder of Angle Motor
    public SwerveModulePosition getPosition() {
        SmartDashboard.putNumber("Encoder position " + moduleNumber, angleEncoder.getPosition());
        SmartDashboard.putNumber("Offset degrees " + moduleNumber, angleOffset.getDegrees());

        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                Rotation2d.fromDegrees(angleEncoder.getPosition() - angleOffset.getDegrees()));
    }
}
