// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Setting;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

public class extenderSubsystem extends PIDInterface {
  /** Creates a new extenderSubsystem. */
  private final CANSparkMax extenderMotor;
  private final RelativeEncoder extenderEncoder;

  private final SparkMaxPIDController extenderController;

  public extenderSubsystem() {
    extenderMotor = new CANSparkMax(Ports.Extender.extender, MotorType.kBrushless);
    extenderEncoder = extenderMotor.getEncoder();
    extenderMotor.setIdleMode(IdleMode.kBrake);

    extenderController = extenderMotor.getPIDController();
    configExtenderMotor(extenderMotor, extenderEncoder, extenderController, Ports.Extender.ExtenderMotorInvert);

    extenderMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    extenderMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    extenderMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 30);
    // 270 when neo was 27:1 ratio
    // extenderMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 270);//
    // 270 when neo was 27:1 ratio

    extenderMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    setSmartMotionParams();
    PID();

    // resetEncoder();

    // enableMotors(true);//TODO test later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Extender Encoder Value", extenderEncoder.getPosition());
  }

  // Methods for config for the motors used in this subsystems
  private void setSmartMotionParams() {
    extenderController.setSmartMotionMaxVelocity(Setting.wristSetting.SmartMotionParameters.maxVel,
        Setting.wristSetting.SmartMotionParameters.smartMotionSlot);
    extenderController.setSmartMotionMinOutputVelocity(Setting.wristSetting.SmartMotionParameters.minVel,
        Setting.wristSetting.SmartMotionParameters.smartMotionSlot);
    extenderController.setSmartMotionMaxAccel(Setting.wristSetting.SmartMotionParameters.maxAccel,
        Setting.wristSetting.SmartMotionParameters.smartMotionSlot);
    extenderController.setSmartMotionAllowedClosedLoopError(Setting.wristSetting.SmartMotionParameters.maxErr,
        Setting.wristSetting.SmartMotionParameters.smartMotionSlot);
  }

  private void configExtenderMotor(CANSparkMax extenderMotor, RelativeEncoder extenderEncoder,
      SparkMaxPIDController extenderController, boolean Invert) {
    extenderMotor.restoreFactoryDefaults();
    extenderMotor.clearFaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(extenderMotor, Usage.kPositionOnly);
    extenderMotor.setSmartCurrentLimit(Setting.extenderSetting.extenderContinousCurrentLimit);
    extenderMotor.setInverted(Invert);
    extenderMotor.setIdleMode(Setting.extenderSetting.extenderNeutralMode);
    extenderMotor.enableVoltageCompensation(Setting.extenderSetting.maxVoltage);
    extenderController.setFeedbackDevice(extenderEncoder);
    extenderMotor.burnFlash();
    Timer.delay(1);
    // resetToAbsolute();//FIXME if we are adding a canCODER to the shaft of the arm
  }

  private void PID() {
    extenderController.setP(Setting.extenderSetting.extenderP);
    extenderController.setI(Setting.extenderSetting.extenderI);
    extenderController.setD(Setting.extenderSetting.extenderD);
    extenderController.setFF(Setting.extenderSetting.extenderFF);
  }

  @Override
  public void setPosition(double targetPosition) {
    extenderController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  public void setSmartMotionPosition(double targetPosition) {
    extenderController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  // Getters
  @Override
  public double getCurrentPosition() {
    return extenderEncoder.getPosition();
  }

  public void setMotor(double speed) {
    extenderMotor.set(speed);
  }

  public double getExtenderVelocity() {
    return extenderMotor.getEncoder().getVelocity();
  }

  public double getExtenderCurrent() {
    return extenderMotor.getOutputCurrent();
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
    extenderMotor.setIdleMode(mode);
  }

  public void resetEncoder() {
    extenderEncoder.setPosition(0);
  }

}
