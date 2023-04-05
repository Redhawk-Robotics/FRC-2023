// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.subsystems.modules.SparkMaxModules;

@Deprecated
public class armTest extends SubsystemBase {
  /** Creates a new armTest. */
  private final CANSparkMax armMotorLeft, armMotorRight;

  private final RelativeEncoder armEncoderLeft, armEncoderRight;

  private double armSpeed = 0.5;
  private double armSpeedReverse = -0.5;

  private double stop = 0;

  public armTest() {
    // ------------------------------------- ARM LEFT
    armMotorLeft = SparkMaxModules.leftArm;
    armEncoderLeft = SparkMaxModules.leftArmEncoder;

    // ------------------------------------- ARM RIGHT

    armMotorRight = SparkMaxModules.rightArm;
    armEncoderRight = SparkMaxModules.rightArmEncoder;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("1. TEST arm pivot left", armEncoderLeft.getPosition());

  }

  /*************/
  /*** ARM ***/
  /*************/

  public void upGoArm() {
    System.out.println("ARMUP");
    armMotorLeft.set(armSpeed);
    armMotorRight.set(armSpeed);
    System.out.println("LEFT: " + armMotorLeft.getAppliedOutput());
    System.out.println("RIGHT: " + armMotorRight.getAppliedOutput());
  }

  public void upGoArmController(double speed) {
    armMotorLeft.set(speed);
    armMotorRight.set(speed);
  }

  public double armEncoder() {
    return armEncoderLeft.getPosition();
  }

  public void downGoArm() {
    System.out.println("ARMDOWN");
    armMotorLeft.set(armSpeedReverse);
    armMotorRight.set(armSpeedReverse);
    System.out.println("LEFT: " + armMotorLeft.getAppliedOutput());
    System.out.println("RIGHT: " + armMotorRight.getAppliedOutput());
  }

  public void stopArm() {
    armMotorLeft.set(stop);
    armMotorRight.set(stop);
  }
}
