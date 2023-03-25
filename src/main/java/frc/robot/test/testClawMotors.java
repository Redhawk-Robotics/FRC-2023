// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;

public class testClawMotors extends SubsystemBase {
  public CANSparkMax leftClawMotor = new CANSparkMax(Ports.Claw.leftClaw, MotorType.kBrushless);
  public CANSparkMax rightClawMotor = new CANSparkMax(Ports.Claw.rightClaw, MotorType.kBrushless);

  /** Creates a new testClawMotors. */
  public testClawMotors() {
    leftClawMotor.setIdleMode(IdleMode.kBrake);
    rightClawMotor.setIdleMode(IdleMode.kBrake);
    rightClawMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double upPower, double downPower) {
    double power = upPower - downPower;

    this.leftClawMotor.set(power);
    this.rightClawMotor.set(power);
  }

  public void stop() {
    this.leftClawMotor.set(0);
    this.rightClawMotor.set(0);
  }
}
