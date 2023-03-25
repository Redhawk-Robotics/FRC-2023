// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Setting;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

public class clawTest extends SubsystemBase {

  private final CANSparkMax leftNeo550, rightNeo550;
  private final RelativeEncoder leftEncoder, righEncoder;
  private final SparkMaxPIDController leftPIDController, rightPIDController; 
  private final DoubleSolenoid clawSolenoid;
  public final double neoPower = .5;

  /** Creates a new clawTest. */
  public clawTest() {
    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Setting.clawPneumatic.clawForwardChan, Setting.clawPneumatic.clawReverseChan);
    
    leftNeo550 = new CANSparkMax(Ports.Claw.leftClaw, MotorType.kBrushless);
    leftEncoder = leftNeo550.getEncoder();

    rightNeo550 = new CANSparkMax(Ports.Claw.rightClaw, MotorType.kBrushless);
    righEncoder = rightNeo550.getEncoder();

    leftPIDController = leftNeo550.getPIDController();
    rightPIDController = rightNeo550.getPIDController();

    configClawMotor(leftNeo550, leftEncoder, leftPIDController, true);
    configClawMotor(rightNeo550, righEncoder, rightPIDController, false);

    configClawMotor(leftNeo550, leftEncoder, leftPIDController, Ports.Claw.leftClawMotorInvert);
    configClawMotor(rightNeo550, righEncoder, rightPIDController, Ports.Claw.rightClawMotorInvert);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotors() {
    leftNeo550.set(neoPower);
    rightNeo550.set(neoPower);
  }

  public void stopMotors() {
    leftNeo550.set(0);
    rightNeo550.set(0);
  }

  public void closeClaw() {
    clawSolenoid.set(kForward);
  }

  public void openClaw() {
    clawSolenoid.set(kReverse);
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
