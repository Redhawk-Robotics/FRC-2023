// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.Ports;
import frc.robot.constants.Setting;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  private final CANSparkMax leftNeo550, rightNeo550;
  private final RelativeEncoder leftEncoder, rightEncoder;
  
  private final SparkMaxPIDController clawSpeedPIDController; 

  private final DoubleSolenoid clawSolenoid;

  public ClawSubsystem() {
    leftNeo550 = new CANSparkMax(Ports.Claw.leftClaw, MotorType.kBrushless);
    rightNeo550 = new CANSparkMax(Ports.Claw.rightClaw, MotorType.kBrushless);

    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Setting.clawPneumatic.clawForwardChan, Setting.clawPneumatic.clawReverseChan);
    
    leftEncoder = leftNeo550.getEncoder();
    rightEncoder = rightNeo550.getEncoder();

    leftNeo550.follow(rightNeo550);
    
    clawSpeedPIDController = leftNeo550.getPIDController();
    
    configClawMotor(leftNeo550, leftEncoder, clawSpeedPIDController, Ports.Claw.leftClawMotorInvert);
    configClawMotor(rightNeo550, rightEncoder, clawSpeedPIDController, Ports.Claw.rightClawMotorInvert);

    //enableMotors(true);//TODO test later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void enableMotors(boolean on){
    IdleMode mode;
    if(on){
      mode = IdleMode.kBrake;
    }else{
      mode = IdleMode.kCoast;
    }
    leftNeo550.setIdleMode(mode);
    rightNeo550.setIdleMode(mode);
  }
  
  public void coneIntake() {
    clawSolenoid.set(Value.kForward);
    leftNeo550.set(.5);
    rightNeo550.set(.5);
  }

  public void outTake() {
    clawSolenoid.set(Value.kReverse);
    // leftClaw.set(-.25);
    // rightClaw.set(-.25);
  }

  public void cubeIntake() {
    clawSolenoid.set(Value.kReverse);
    // leftClaw.set(.75);
    // rightClaw.set(.75);
  }

  public void stopClaw() {
    clawSolenoid.set(Value.kOff);
    // leftClaw.set(0);
    // rightClaw.set(0);
  }


  public void configClawMotor(CANSparkMax clawMotor, RelativeEncoder clawEncoder, SparkMaxPIDController clawController, boolean invert) {
    clawMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(clawMotor, Usage.kAll);
    clawMotor.setSmartCurrentLimit(Setting.clawSetting.clawContinousCurrentLimit);
    clawMotor.setIdleMode(Setting.clawSetting.clawNeutralMode);
    clawMotor.setInverted(invert);
    clawEncoder.setVelocityConversionFactor(Setting.clawSetting.clawConversionVelocityFactor);
    clawEncoder.setPositionConversionFactor(Setting.clawSetting.clawConversionPositionFactor);
    clawController.setP(Setting.clawSetting.clawP);
    clawController.setI(Setting.clawSetting.clawI);
    clawController.setD(Setting.clawSetting.clawD);
    clawController.setFF(Setting.clawSetting.clawFF);
    clawMotor.enableVoltageCompensation(Setting.clawSetting.maxVoltage);
    clawMotor.burnFlash();
  }
}
