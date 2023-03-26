// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.util.Set;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Setting;

public class testWhatever extends SubsystemBase {
  /** Creates a new testWhatever. */
  private final CANSparkMax armMotorLeft, armMotorRight;

  private final RelativeEncoder armEncoderLeft, armEncoderRight;
  private final CANSparkMax leftClaw, rightClaw;

  private final CANSparkMax wrist;
  private final RelativeEncoder wristEncoder;

  private final CANSparkMax extender;
  private final RelativeEncoder extenderEncoder;

  private final DoubleSolenoid clawOpen;

  private double stop = 0;

  private double extenderSpeed = 1;
  private double extenderSpeedReverse = -1;

  private double wristSpeed = 0.1;
  private double wristSpeedReverse = -0.1;

  private double armSpeed = 0.3;
  private double armSpeedReverse = -0.1;

  public testWhatever() {
    // ------------------------------------- ARM LEFT
    armMotorLeft = new CANSparkMax(Ports.Arm.leftArm, MotorType.kBrushless); // 9
    armEncoderLeft = armMotorLeft.getEncoder();
    armMotorLeft.setInverted(false);
    armMotorLeft.restoreFactoryDefaults();

    // ------------------------------------- ARM RIGHT

    armMotorRight = new CANSparkMax(Ports.Arm.rightArm, MotorType.kBrushless); // 10
    armEncoderRight = armMotorRight.getEncoder();
    armMotorRight.setInverted(true);
    armMotorRight.restoreFactoryDefaults();
    armMotorRight.follow(armMotorLeft, false); // <= follow

    // ------------------------------------- EXTENDER

    extender = new CANSparkMax(Ports.Extender.extender, MotorType.kBrushless); // 11
    extenderEncoder = extender.getEncoder();
    extender.setInverted(true);
    extender.restoreFactoryDefaults();

    // ------------------------------------- LEFT CLAW

    leftClaw = new CANSparkMax(Ports.Claw.leftClaw, MotorType.kBrushless); // 12
    leftClaw.setInverted(false);

    // RIGHT CLAW
    rightClaw = new CANSparkMax(Ports.Claw.rightClaw, MotorType.kBrushless); // 13
    rightClaw.setInverted(true);

    // ------------------------------------- WRIST

    wrist = new CANSparkMax(Ports.Wrist.wrist, MotorType.kBrushless); // 14
    wristEncoder = wrist.getEncoder();
    wrist.setInverted(false);
    wrist.restoreFactoryDefaults();

    // -------------------------------------

    clawOpen = new DoubleSolenoid(PneumaticsModuleType.REVPH, Setting.clawPneumatic.clawForwardChan,
        Setting.clawPneumatic.clawReverseChan);

    // ----------------------------------------------------------------------------------

    /************/
    /*** ARM ***/
    /***********/

    armMotorLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armMotorLeft.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 50);
    armMotorLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    armMotorLeft.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -10);

    // -------------------------------------

    /****************/
    /*** EXTENDER ***/
    /***************/

    extender.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 290);
    extender.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    extender.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    extender.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    /*************/
    /*** WRIST ***/
    /*************/

    wrist.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, -10);
    wrist.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    wrist.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 50);
    wrist.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    // -------------------------------------

    // FIXME
    // check if both right and left are going the correct directions FIXME
    // armMotorRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    // armMotorRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 50);
    // armMotorRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    // armMotorRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("testMotor Encoder Value",
    // getEncoderMeters(EncoderValue));

    SmartDashboard.putNumber("TEST arm pivot left", armEncoderLeft.getPosition());
    SmartDashboard.putNumber("TEST arm pivot right", armEncoderRight.getPosition());
    SmartDashboard.putNumber("TEST arm extender", extenderEncoder.getPosition());
    SmartDashboard.putNumber("TEST wrist pivot", wristEncoder.getPosition());
  }

  /*************/
  /*** ARM ***/
  /*************/

  public void upGoArm() {
    armMotorLeft.set(armSpeed);
    armMotorRight.set(armSpeed);
  }

  public void downGoArm() {
    armMotorLeft.set(armSpeedReverse);
    armMotorRight.set(armSpeedReverse);
  }

  public void stopArm() {
    armMotorLeft.set(stop);
    armMotorRight.set(stop);
  }

  // -------------------------------------

  /*************/
  /*** CLAW ***/
  /*************/

  public void coneIntake() {
    clawOpen.set(Value.kForward);
    leftClaw.set(.5);
    rightClaw.set(.5);
  }

  public void outTake() {
    clawOpen.set(Value.kReverse);
    // leftClaw.set(-.25);
    // rightClaw.set(-.25);
  }

  public void cubeIntake() {
    clawOpen.set(Value.kReverse);
    // leftClaw.set(.75);
    // rightClaw.set(.75);
  }

  public void stopClaw() {
    clawOpen.set(Value.kOff);
    // leftClaw.set(0);
    // rightClaw.set(0);
  }

  // -------------------------------------

  /*************/
  /*** WRIST ***/
  /*************/

  public void upGoWrist() {
    wrist.set(wristSpeed);
  }

  public void stopWrist() {
    wrist.set(stop);
  }

  public void downGoWrist() {
    wrist.set(wristSpeedReverse);
  }

  // -------------------------------------

  /*************/
  /*** EXTENDER ***/
  /*************/

  public void upExtender() {
    extender.set(extenderSpeed);
  }

  public void stopExtender() {
    extender.set(stop);
  }

  public void downExtender() {
    extender.set(extenderSpeedReverse);
  }

  /****************/
  /*** COMMANDS ***/
  /***************/

  public Command rasieShoulderCMD() {
    return new Command() {

      @Override
      public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;

      };

      @Override
      public void execute() {
        upGoArm();
        // armEncoderLeft

      }
    };
  }
}
// thing