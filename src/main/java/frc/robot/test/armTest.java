// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Setting;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

public class armTest extends SubsystemBase {
  /** Creates a new armTest. */
  private final CANSparkMax leftArmMotor, rightArmMotor;
  private final RelativeEncoder leftArmEncoder, rightArmEncoder;

  private final SparkMaxPIDController armAngleController;
  private double leftArmEncoderValue, rightArmEncoderValue;

  public armTest() {
    leftArmMotor = new CANSparkMax(Ports.Arm.leftArm, MotorType.kBrushless);
    leftArmEncoder = leftArmMotor.getEncoder();

    rightArmMotor = new CANSparkMax(Ports.Arm.rightArm, MotorType.kBrushless);
    rightArmEncoder = rightArmMotor.getEncoder();

    // armAngleController = leftArmMotor.getPIDController();
    armAngleController = rightArmMotor.getPIDController();

    configArmMotor(leftArmMotor, leftArmEncoder, armAngleController, Ports.Arm.leftArmMotorInvert);
    configArmMotor(rightArmMotor, rightArmEncoder, armAngleController, Ports.Arm.rightArmMotorInvert);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RightArm Encoder Value", getEncoderMetersLeft(leftArmEncoderValue));
    SmartDashboard.putNumber("LeftArm Encoder Value", getEncoderMetersRight(rightArmEncoderValue));
  }

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

  public void setMotor(double speed) {
    leftArmMotor.set(speed);
    rightArmMotor.set(speed);
  }

  public double getEncoderMetersLeft(double positionLeft) {
    positionLeft = leftArmEncoder.getPosition() * Setting.armSetting.kEncoderTick2Meter;
    return positionLeft;
  }

  public double getEncoderMetersRight(double positionRight) {
    positionRight = rightArmEncoder.getPosition() * Setting.armSetting.kEncoderTick2Meter;
    return positionRight;
  }

  public void resetEncoder() {
    leftArmEncoder.setPosition(0);
    rightArmEncoder.setPosition(0);
  }

}
