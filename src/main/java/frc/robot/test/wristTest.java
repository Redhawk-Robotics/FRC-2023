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

public class wristTest extends SubsystemBase {
  /** Creates a new wristTest. */
  private final CANSparkMax wrist;
  private final RelativeEncoder wristEncoder;

  private double wristSpeed = 0.3;
  private double wristSpeedReverse = -0.2;

  private double stop = 0;

  public wristTest() {
    // ------------------------------------- WRIST

    wrist = new CANSparkMax(Ports.Wrist.wrist, MotorType.kBrushless); // 14
    wristEncoder = wrist.getEncoder();
    wrist.restoreFactoryDefaults();
    wrist.setIdleMode(IdleMode.kBrake);
    wrist.setInverted(false);

    wrist.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 5);
    wrist.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    wrist.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -30);
    wrist.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    // ------------------------------------

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("WRIST", wristEncoder.getPosition());
  }
   /*************/
  /*** WRIST ***/
  /*************/
  

  public void wristController(double speed){
    wrist.set(speed);
  }

  public void upGoWrist() {
    wrist.set(wristSpeed);
  }

  public void stopWrist() {
    wrist.set(stop);
  }

  public void downGoWrist() {
    wrist.set(wristSpeedReverse);
  }

  public double wristEncoder(){
    return wristEncoder.getPosition();
  } 

  // -------------------------------------

}
