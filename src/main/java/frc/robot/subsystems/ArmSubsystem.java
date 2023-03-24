// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Setting;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax leftArmMotor, rightArmMotor;
  private final RelativeEncoder leftArmEncoder, rightArmEncoder;

  private final SparkMaxPIDController armAngleController;

  // private double leftArmEncoderValue;
  // private double rightArmEncoderValue;

  public ArmSubsystem() {

    leftArmMotor = new CANSparkMax(Ports.Arm.leftArm, MotorType.kBrushless);
    leftArmEncoder = leftArmMotor.getEncoder();

    rightArmMotor = new CANSparkMax(Ports.Arm.rightArm, MotorType.kBrushless);
    rightArmEncoder = rightArmMotor.getEncoder();

    rightArmMotor.follow(leftArmMotor, false); // TODO check the invert

    // armAngleController = leftArmMotor.getPIDController();
    armAngleController = leftArmMotor.getPIDController();

    configArmMotor(leftArmMotor, leftArmEncoder, armAngleController, Ports.Arm.leftArmMotorInvert);// May not need this
                                                                                                   // becuase its
                                                                                                   // following the
                                                                                                   // other motor
    configArmMotor(rightArmMotor, rightArmEncoder, armAngleController, Ports.Arm.rightArmMotorInvert);

    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);// TODO check the value for both forward and
                                                                           // reverse
    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    // SmartDashboard.putNumber("Forward Soft Limit",
    // leftArmMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));

    // SmartDashboard.putNumber("Reverse Soft Limit",
    // leftArmMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));

    SmartDashboard.putBoolean("Forward Soft Limit",
        leftArmMotor.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kForward));

    SmartDashboard.putBoolean("Reverse Soft Limit",
        leftArmMotor.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kReverse));
    resetEncoder();

    // enableMotors(true);//TODO test later

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RightArm Encoder Value", getEncoderMetersLeft());
    SmartDashboard.putNumber("LeftArm Encoder Value", getEncoderMetersRight());

    // only if we need for debugging
    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
        SmartDashboard.getBoolean("Forward Soft Limit Enabled", true));
    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
        SmartDashboard.getBoolean("Reverse Soft Limit Enabled", true));

    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
        (float) SmartDashboard.getNumber("Forward Soft Limit", 15));
    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
        (float) SmartDashboard.getNumber("Reverse Soft Limit", 0));
  }

  // Methods for config for the motors used in this subsystems
  private void configArmMotor(CANSparkMax ArmMotor, RelativeEncoder ArmEncoder,
      SparkMaxPIDController armAngleController, boolean Invert) {
    ArmMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(ArmMotor, Usage.kPositionOnly);
    ArmMotor.setSmartCurrentLimit(Setting.armSetting.armContinousCurrentLimit);
    ArmMotor.setInverted(Invert);
    ArmMotor.setIdleMode(Setting.armSetting.armNeutralMode);
    ArmEncoder.setPositionConversionFactor(Setting.armSetting.armConversionFactor);
    armAngleController.setP(Setting.armSetting.armP);
    armAngleController.setI(Setting.armSetting.armI);
    armAngleController.setD(Setting.armSetting.armD);
    armAngleController.setFF(Setting.armSetting.armFF);
    ArmMotor.enableVoltageCompensation(Setting.armSetting.maxVoltage);
    ArmMotor.burnFlash();
    Timer.delay(1);
    // resetToAbsolute();//FIXME if we are adding a canCODER to the shaft of the arm
  }

  public double getEncoderMetersLeft() {
    double positionLeft = leftArmEncoder.getPosition() * Setting.armSetting.kEncoderTick2Meter;
    return positionLeft;
  }

  public double getEncoderMetersRight() {
    double positionRight = rightArmEncoder.getPosition() * Setting.armSetting.kEncoderTick2Meter;
    return positionRight;
  }

  // TODO try with the wrist that if its in code that its coast, and moves freely,
  // then this method is not needed
  public void enableMotors(boolean on) {
    IdleMode mode;
    if (on) {
      mode = IdleMode.kBrake;
    } else {
      mode = IdleMode.kCoast;
    }
    leftArmMotor.setIdleMode(mode);
    rightArmMotor.setIdleMode(mode);
  }

  public void setMotor(double speed) {
    leftArmMotor.set(speed);
    rightArmMotor.set(speed);

    leftArmMotor.enableSoftLimit(SoftLimitDirection.kForward, SmartDashboard.getBoolean("Forward Soft Limit", false));
    leftArmMotor.enableSoftLimit(SoftLimitDirection.kForward, SmartDashboard.getBoolean("Forward Soft Limit", false));
    rightArmMotor.enableSoftLimit(SoftLimitDirection.kReverse,
        SmartDashboard.getBoolean("Reverse Soft Limit Enabled", false));
    rightArmMotor.enableSoftLimit(SoftLimitDirection.kReverse,
        SmartDashboard.getBoolean("Reverse Soft Limit Enabled", false));
  }

  public void resetEncoder() {
    leftArmEncoder.setPosition(0);
    rightArmEncoder.setPosition(0);
  }

}
