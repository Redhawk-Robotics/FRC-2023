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

public class extenderTest extends SubsystemBase {
  /** Creates a new extenderTest. */
  private final CANSparkMax extender;
  private final RelativeEncoder extenderEncoder;

  private double extenderSpeed = 1;
  private double extenderSpeedReverse = -1;

  private double stop = 0;

  public extenderTest() {
    // ------------------------------------- EXTENDER

    extender = SparkMaxModules.extender;
    extenderEncoder = SparkMaxModules.extenderEncoder;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("2. EXTENDER", extenderEncoder.getPosition());
  }

  /*************/
  /*** EXTENDER ***/
  /*************/
  public void extenderController(double speed) {
    extender.set(speed);
  }

  public void upExtender() {
    extender.set(extenderSpeed);
  }

  public void stopExtender() {
    extender.set(stop);
  }

  public void downExtender() {
    extender.set(extenderSpeedReverse);
  }

  public double extenderEncoder() {
    return extenderEncoder.getPosition();
  }

}
