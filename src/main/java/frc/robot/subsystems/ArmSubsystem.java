// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Setting;

public class ArmSubsystem extends PIDInterface {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax leftArmMotor, rightArmMotor;
  private final RelativeEncoder leftArmEncoder, rightArmEncoder;

  private final SparkMaxPIDController armAngleController;

  public ArmSubsystem() {

    leftArmMotor = new CANSparkMax(Ports.Arm.leftArm, MotorType.kBrushless);
    leftArmMotor.setIdleMode(IdleMode.kCoast);
    leftArmEncoder = leftArmMotor.getEncoder();

    rightArmMotor = new CANSparkMax(Ports.Arm.rightArm, MotorType.kBrushless);
    rightArmEncoder = rightArmMotor.getEncoder();

    leftArmMotor.follow(rightArmMotor, true);

    armAngleController = rightArmMotor.getPIDController();

    configArmMotor(rightArmMotor, rightArmEncoder, armAngleController, Ports.Arm.rightArmMotorInvert);

    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 66);

    rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -2);

    // resetEncoder();

    // enableMotors(true);//TODO test later

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LeftArm Encoder Value", leftArmEncoder.getPosition());
    SmartDashboard.putNumber("RightArm Encoder Value", rightArmEncoder.getPosition());
  }

  // Methods for config for the motors used in this subsystems
  private void configArmMotor(CANSparkMax ArmMotor, RelativeEncoder ArmEncoder,
      SparkMaxPIDController armAngleController, boolean Invert) {
    ArmMotor.restoreFactoryDefaults();
    ArmMotor.clearFaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(ArmMotor, Usage.kPositionOnly);
    ArmMotor.setSmartCurrentLimit(Setting.armSetting.armContinousCurrentLimit);
    ArmMotor.setInverted(Invert);
    ArmMotor.setIdleMode(Setting.armSetting.armNeutralMode);
    ArmMotor.enableVoltageCompensation(Setting.armSetting.maxVoltage);
    armAngleController.setFeedbackDevice(ArmEncoder);
    ArmMotor.burnFlash();
    Timer.delay(1);
    // resetToAbsolute();//FIXME if we are adding a canCODER to the shaft of the arm
  }

  private void pidUp() {
    armAngleController.setP(Setting.armSetting.armPup);
    armAngleController.setI(Setting.armSetting.armIup);
    armAngleController.setD(Setting.armSetting.armDup);
    armAngleController.setFF(Setting.armSetting.armFFup);
  }

  private void pidDown() {
    armAngleController.setP(Setting.armSetting.armPdown);
    armAngleController.setI(Setting.armSetting.armIdown);
    armAngleController.setD(Setting.armSetting.armDdown);
    armAngleController.setFF(Setting.armSetting.armFFdown);
  }

  private void manageMotion(double targetPosition) {
    double currentPosition = getCurrentPosition();
    if (currentPosition < targetPosition) {
      pidUp();
    } else {
      pidDown();
    }
  }

  @Override
  public void setPosition(double targetPosition) {
    manageMotion(targetPosition);
    armAngleController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  public void setSmartMotionPosition(double targetPosition) {
    manageMotion(targetPosition);
    armAngleController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  // Getters
  @Override
  public double getCurrentPosition() {
    return rightArmEncoder.getPosition();
  }

  public CANSparkMax getRightMotor() {
    return rightArmMotor;
  }

  public void setMotor(double speed) {
    rightArmMotor.set(speed);
  }

  public double getArmVelocity() {
    return rightArmMotor.getEncoder().getVelocity();
  }

  public double getArmCurrent() {
    return rightArmMotor.getOutputCurrent();
  }

  // TODO try with the wrist that if its in code that its coast, and moves freely,
  // then this method is not needed
  public void enableMotors(boolean on) {
    IdleMode mode = on ? IdleMode.kBrake : IdleMode.kCoast;

    leftArmMotor.setIdleMode(mode);
    rightArmMotor.setIdleMode(mode);
  }

  public void resetEncoder() {
    leftArmEncoder.setPosition(0);
    rightArmEncoder.setPosition(0);
  }

}
