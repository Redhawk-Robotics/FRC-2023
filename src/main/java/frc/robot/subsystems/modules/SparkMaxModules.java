package frc.robot.subsystems.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.Ports;
import frc.robot.constants.Setting;

public class SparkMaxModules {
    public static CANSparkMax leftArm, rightArm;
    public static CANSparkMax extender;
    public static CANSparkMax wrist, leftClaw, rightClaw;
    public static RelativeEncoder leftArmEncoder, rightArmEncoder, extenderEncoder, wristEncoder;

    public SparkMaxModules() {
        // ------------------------------------- ARM
        leftArm = new CANSparkMax(Ports.Arm.leftArm, MotorType.kBrushless); // 9
        leftArmEncoder = leftArm.getEncoder();
        leftArm.setInverted(false);
        leftArm.setIdleMode(IdleMode.kBrake);
        leftArm.restoreFactoryDefaults();

        rightArm = new CANSparkMax(Ports.Arm.rightArm, MotorType.kBrushless); // 10
        rightArmEncoder = rightArm.getEncoder();
        rightArm.setIdleMode(IdleMode.kBrake);
        rightArm.restoreFactoryDefaults();
        leftArm.follow(rightArm, true); // <= follow

        // ------------------------------------- EXTENDER
        extender = new CANSparkMax(Ports.Extender.extender, MotorType.kBrushless); // 11
        extenderEncoder = extender.getEncoder();
        extender.restoreFactoryDefaults();

        extender.setIdleMode(IdleMode.kBrake);
        extender.setInverted(false);

        // ------------------------------------- WRIST
        wrist = new CANSparkMax(Ports.Wrist.wrist, MotorType.kBrushless); // 14
        wristEncoder = wrist.getEncoder();
        wrist.restoreFactoryDefaults();
        wrist.setIdleMode(IdleMode.kBrake);
        wrist.setInverted(false);

        // ------------------------------------- CLAW

        leftClaw = new CANSparkMax(Ports.Claw.leftClaw, MotorType.kBrushless); // 12
        leftClaw.setInverted(true);
        // leftClaw.setIdleMode(IdleMode.kCoast);

        // RIGHT CLAW
        rightClaw = new CANSparkMax(Ports.Claw.rightClaw, MotorType.kBrushless); // 13
        rightClaw.setInverted(true);
        // rightClaw.setIdleMode(IdleMode.kCoast);

        // ------------------------------------- SOFT-LIMITS
        setSoftLimit(rightArm, Setting.SoftLimits.armForwardLimit, Setting.SoftLimits.armReverseLimit);
        setSoftLimit(extender, Setting.SoftLimits.extenderForwardLimit, Setting.SoftLimits.extenderReverseLimit);
        setSoftLimit(wrist, Setting.SoftLimits.wristForwardLimit, Setting.SoftLimits.wristReverseLimit);

    }

    public void setSoftLimit(CANSparkMax motor, float forwardLimit, float reverseLimit) {
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forwardLimit);
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, reverseLimit);
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    }

    // public CANSparkMax getLeftArm() {
    //     return leftArm;
    // }

    // public RelativeEncoder getLeftArmEncoder() {
    //     return leftArmEncoder;
    // }

    // public CANSparkMax getRightArm() {
    //     return rightArm;
    // }

    // public RelativeEncoder getRightEncoder() {
    //     return rightArmEncoder;
    // }

    // public CANSparkMax getExtender() {
    //     return extender;
    // }

    // public RelativeEncoder getExtenderEncoder() {
    //     return extenderEncoder;
    // }


}
