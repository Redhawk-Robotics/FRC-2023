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

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
private final CANSparkMax wristMotor;
private final RelativeEncoder wristEncoder;

private final SparkMaxPIDController wristAngleController;

private double wristEncoderValue;

  public WristSubsystem() {
    wristMotor = new CANSparkMax(Ports.Wrist.wrist, MotorType.kBrushless);
    wristEncoder = wristMotor.getEncoder();

    wristAngleController = wristMotor.getPIDController();

    configWristMotor(wristMotor, wristEncoder, wristAngleController, Ports.Wrist.wristMotorInvert);

    wristMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    wristMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    wristMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);//TODO check the value for both forward and reverse
    wristMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    SmartDashboard.putNumber("wrist Forward Soft Limit",
    wristMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));

    SmartDashboard.putNumber("wrist Reverse Soft Limit",
    wristMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));

    resetEncoder();
    //enableMotors(true);//TODO test later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("wrist Encoder Value", getEncoderMeters());
    SmartDashboard.putNumber("Wrist Encoder Value", wristEncoder.getPosition());

    //only if we need for debugging
    wristMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 
    SmartDashboard.getBoolean("wrist Forward Soft Limit Enabled", true));
    wristMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 
    SmartDashboard.getBoolean("wrist Reverse Soft Limit Enabled", true));

    wristMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 
    (float)SmartDashboard.getNumber("wrist Forward Soft Limit", 15));

    wristMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
    (float)SmartDashboard.getNumber("wrist Reverse Soft Limit", 0));
  }
    //Methods for config for the motors used in this subsystems
    private void configWristMotor(CANSparkMax wristMotor, RelativeEncoder wristEncoder, SparkMaxPIDController wristAngleController, boolean Invert) {
      wristMotor.restoreFactoryDefaults();
      CANSparkMaxUtil.setCANSparkMaxBusUsage(wristMotor, Usage.kPositionOnly);
      wristMotor.setSmartCurrentLimit(Setting.wristSetting.wristContinousCurrentLimit);
      wristMotor.setInverted(Invert);
      wristMotor.setIdleMode(Setting.wristSetting.wristNeutralMode);
      wristEncoder.setPositionConversionFactor(Setting.wristSetting.wristConversionFactor);
      wristAngleController.setP(Setting.wristSetting.wristP);
      wristAngleController.setI(Setting.wristSetting.wristI);
      wristAngleController.setD(Setting.wristSetting.wristD);
      wristAngleController.setFF(Setting.wristSetting.wristFF);
      wristMotor.enableVoltageCompensation(Setting.wristSetting.maxVoltage);
      wristMotor.burnFlash();
      Timer.delay(1);
      //resetToAbsolute();//FIXME if we are adding a canCODER to the shaft of the arm
      }
      public double getEncoderMeters() {
        double position = wristEncoder.getPosition() * Setting.wristSetting.kEncoderTick2Meter;
        return position;
      }

    //TODO try with the wrist that if its in code that its coast, and moves freely, then this method is not needed
      public void enableMotors(boolean on){
        IdleMode mode;
        if(on){
          mode = IdleMode.kBrake;
        }else{
          mode = IdleMode.kCoast;
        }
        wristMotor.setIdleMode(mode);
      }
    
      public void setMotor(double speed) {
        wristMotor.set(speed);
      }
      public void resetEncoder(){
        wristEncoder.setPosition(0);
      }
}
