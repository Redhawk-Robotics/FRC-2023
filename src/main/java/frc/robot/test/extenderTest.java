// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;

public class extenderTest extends SubsystemBase {
  /** Creates a new extenderTest. */
  private final CANSparkMax extender;
  private final RelativeEncoder extenderEncoder;

  private double extenderSpeed = .5;
  private double extenderSpeedReverse = -.5;

  private double stop = 0;

  public extenderTest() {
// ------------------------------------- EXTENDER

extender = new CANSparkMax(Ports.Extender.extender, MotorType.kBrushless); // 11
extenderEncoder = extender.getEncoder();
extender.restoreFactoryDefaults();

extender.setIdleMode(IdleMode.kBrake);
extender.setInverted(false);

    /****************/
    /*** EXTENDER ***/
    /***************/

    extender.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
    extender.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    extender.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -270);
    extender.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /*************/
  /*** EXTENDER ***/
  /*************/
  public void extenderController(double speed){
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

  public double extenderEncoder(){
    return extenderEncoder.getPosition();
  }

  
}
