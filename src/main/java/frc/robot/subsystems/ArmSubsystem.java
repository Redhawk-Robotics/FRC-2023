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

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax leftArmMotor, rightArmMotor;
  private final RelativeEncoder leftArmEncoder, rightArmEncoder;

  private final SparkMaxPIDController armAngleController;

  public ArmSubsystem() {

    leftArmMotor = new CANSparkMax(Ports.Arm.leftArm, MotorType.kBrushless);
    leftArmEncoder = leftArmMotor.getEncoder();

    rightArmMotor = new CANSparkMax(Ports.Arm.rightArm, MotorType.kBrushless);
    rightArmEncoder = rightArmMotor.getEncoder();

    leftArmMotor.follow(rightArmMotor, true); // TODO check the invert

    // armAngleController = leftArmMotor.getPIDController();
    armAngleController = rightArmMotor.getPIDController();

    // configArmMotor(leftArmMotor, leftArmEncoder, armAngleController, Ports.Arm.leftArmMotorInvert);
    
    configArmMotor(rightArmMotor, rightArmEncoder, armAngleController, Ports.Arm.rightArmMotorInvert);

    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);// TODO check the value for both forward and
                                                                           // reverse
    rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    // SmartDashboard.putNumber("Forward Soft Limit",
    // leftArmMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));

    // SmartDashboard.putNumber("Reverse Soft Limit",
    // leftArmMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));

    // SmartDashboard.putBoolean("Forward Soft Limit",
    //     leftArmMotor.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kForward));

    // SmartDashboard.putBoolean("Reverse Soft Limit",
    //     leftArmMotor.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kReverse));
    
    //resetEncoder();

    // enableMotors(true);//TODO test later

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RightArm Encoder Value", leftArmEncoder.getPosition());

    // only if we need for debugging
  //   leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
  //       SmartDashboard.getBoolean("Forward Soft Limit Enabled", true));
  //   leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
  //       SmartDashboard.getBoolean("Reverse Soft Limit Enabled", true));

  //   leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
  //       (float) SmartDashboard.getNumber("Forward Soft Limit", 15));
  //   leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
  //       (float) SmartDashboard.getNumber("Reverse Soft Limit", 0));
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
    ArmEncoder.setPositionConversionFactor(Setting.armSetting.armConversionFactor);
    ArmMotor.enableVoltageCompensation(Setting.armSetting.maxVoltage);
    armAngleController.setFeedbackDevice(ArmEncoder);
    ArmMotor.burnFlash();
    Timer.delay(1);
    // resetToAbsolute();//FIXME if we are adding a canCODER to the shaft of the arm
  }
  private void pidUp(){
    armAngleController.setP(Setting.armSetting.armPup);
    armAngleController.setI(Setting.armSetting.armIup);
    armAngleController.setD(Setting.armSetting.armDup);
    armAngleController.setFF(Setting.armSetting.armFFup);
  }

  private void pidDown(){
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

  public void setPosition(double targetPosition) {
    manageMotion(targetPosition);
    armAngleController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  public void setSmartMotionPosition(double targetPosition) {
    manageMotion(targetPosition);
    armAngleController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  //Getters
 public double getCurrentPosition() {
    return rightArmEncoder.getPosition();
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
    IdleMode mode = on ? IdleMode.kBrake: IdleMode.kCoast;

    leftArmMotor.setIdleMode(mode);
    rightArmMotor.setIdleMode(mode);
  }

  public void resetEncoder() {
    leftArmEncoder.setPosition(0);
    rightArmEncoder.setPosition(0);
  }

}
