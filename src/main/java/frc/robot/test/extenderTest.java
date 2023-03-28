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

public class extenderTest extends SubsystemBase {
  /** Creates a new ExtendSubsystem. */
  private final CANSparkMax extenderMotor;
  private final RelativeEncoder extenderEncoder;
  private final SparkMaxPIDController extenderController;
  private double extenderEncoderValue;

  public extenderTest() {
    extenderMotor = new CANSparkMax(Ports.Extender.extender, MotorType.kBrushless);
    extenderEncoder = extenderMotor.getEncoder();

    extenderController = extenderMotor.getPIDController();

    configArmMotor(extenderMotor, extenderEncoder, extenderController, Ports.Extender.ExtenderMotorInvert);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extender Encoder Value", getEncoderMetersLeft(extenderEncoderValue));
  }

  private void configArmMotor(CANSparkMax extenderMotor, RelativeEncoder extenderEncoder,
      SparkMaxPIDController extenderController, boolean Invert) {
    extenderMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(extenderMotor, Usage.kPositionOnly);
    extenderMotor.setSmartCurrentLimit(Setting.extenderSetting.extenderContinousCurrentLimit);
    extenderMotor.setInverted(Invert);
    extenderMotor.setIdleMode(Setting.extenderSetting.extenderNeutralMode);
    // extenderEncoder.setPositionConversionFactor(Setting.extenderSetting.extenderConversionFactor);
    extenderController.setP(Setting.extenderSetting.extenderP);
    extenderController.setI(Setting.extenderSetting.extenderI);
    extenderController.setD(Setting.extenderSetting.extenderD);
    extenderController.setFF(Setting.extenderSetting.extenderFF);
    extenderMotor.enableVoltageCompensation(Setting.extenderSetting.maxVoltage);
    extenderMotor.burnFlash();
    Timer.delay(1);
    // resetToAbsolute();//FIXME if we are adding a canCODER to the shaft of the arm
  }

  public void setMotor(double speed) {
    extenderMotor.set(speed);
  }

  public double getEncoderMetersLeft(double position) {
    position = extenderEncoder.getPosition() * Setting.extenderSetting.kEncoderTick2Meter;
    return position;
  }
}
