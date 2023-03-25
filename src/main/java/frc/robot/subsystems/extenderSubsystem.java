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

public class extenderSubsystem extends SubsystemBase {
  /** Creates a new extenderSubsystem. */
private final CANSparkMax extenderMotor;
private final RelativeEncoder extenderEncoder;

private final SparkMaxPIDController extenderController;

public extenderSubsystem() {
  extenderMotor = new CANSparkMax(Ports.Extender.extender, MotorType.kBrushless);
  extenderEncoder = extenderMotor.getEncoder();

  extenderController = extenderMotor.getPIDController();
  configExtenderMotor(extenderMotor, extenderEncoder, extenderController, Ports.Extender.ExtenderMotorInvert);

  extenderMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
  extenderMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

  extenderMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);//TODO check the value for both forward and reverse
  extenderMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

  SmartDashboard.putNumber("extender Forward Soft Limit",
  extenderMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));

  SmartDashboard.putNumber("extender Reverse Soft Limit",
  extenderMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));
  
  resetEncoder();

  //enableMotors(true);//TODO test later
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  // This method will be called once per scheduler run
  // SmartDashboard.putNumber("extender Encoder Value", getEncoderMeters());
  SmartDashboard.putNumber("Extender Encoder Value", extenderEncoder.getPosition());

  //only if we need for debugging
  extenderMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 
  SmartDashboard.getBoolean("extender Forward Soft Limit Enabled", true));
  extenderMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 
  SmartDashboard.getBoolean("extender Reverse Soft Limit Enabled", true));

  extenderMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 
  (float)SmartDashboard.getNumber("extender Forward Soft Limit", 15));

  extenderMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
  (float)SmartDashboard.getNumber("extender Reverse Soft Limit", 0));
}
  //Methods for config for the motors used in this subsystems
  private void configExtenderMotor(CANSparkMax extenderMotor, RelativeEncoder extenderEncoder, SparkMaxPIDController extenderController, boolean Invert) {
    extenderMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(extenderMotor, Usage.kPositionOnly);
    extenderMotor.setSmartCurrentLimit(Setting.extenderSetting.extenderContinousCurrentLimit);
    extenderMotor.setInverted(Invert);
    extenderMotor.setIdleMode(Setting.extenderSetting.extenderNeutralMode);
    extenderEncoder.setPositionConversionFactor(Setting.extenderSetting.extenderConversionFactor);
    extenderController.setP(Setting.extenderSetting.extenderP);
    extenderController.setI(Setting.extenderSetting.extenderI);
    extenderController.setD(Setting.extenderSetting.extenderD);
    extenderController.setFF(Setting.extenderSetting.extenderFF);
    extenderMotor.enableVoltageCompensation(Setting.extenderSetting.maxVoltage);
    extenderMotor.burnFlash();
    Timer.delay(1);
    //resetToAbsolute();//FIXME if we are adding a canCODER to the shaft of the arm
    }
    public double getEncoderMeters() {
      double postion = extenderEncoder.getPosition() * Setting.extenderSetting.kEncoderTick2Meter;
      return postion;
    }
  //TODO try with the wrist that if its in code that its coast, and moves freely, then this method is not needed
    public void enableMotors(boolean on){
      IdleMode mode;
      if(on){
        mode = IdleMode.kBrake;
      }else{
        mode = IdleMode.kCoast;
      }
      extenderMotor.setIdleMode(mode);
    }
    public void setMotor(double speed) {
      extenderMotor.set(speed);
    }
    public void resetEncoder(){
      extenderEncoder.setPosition(0);
    }
  
}
