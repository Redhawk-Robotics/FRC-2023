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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  private final CANSparkMax leftNeo550, rightNeo550;
  private final RelativeEncoder leftEncoder;

  private final SparkMaxPIDController clawSpeedPIDController;

  private final DoubleSolenoid clawSolenoid;

  public ClawSubsystem() {
    leftNeo550 = new CANSparkMax(Ports.Claw.leftClaw, MotorType.kBrushless);
    rightNeo550 = new CANSparkMax(Ports.Claw.rightClaw, MotorType.kBrushless);

    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Setting.clawPneumatic.clawForwardChan,
        Setting.clawPneumatic.clawReverseChan);

    leftEncoder = leftNeo550.getEncoder();

    leftNeo550.follow(rightNeo550, false);

    clawSpeedPIDController = leftNeo550.getPIDController();

    configClawMotor(leftNeo550, leftEncoder, clawSpeedPIDController, Ports.Claw.leftClawMotorInvert);

    // PID();
    // enableMotors(true);//TODO test later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableMotors(boolean on) {
    IdleMode mode;
    if (on) {
      mode = IdleMode.kBrake;
    } else {
      mode = IdleMode.kCoast;
    }
    leftNeo550.setIdleMode(mode);
    rightNeo550.setIdleMode(mode);
  }

  public void openClaw() {
    clawSolenoid.set(Value.kForward);

  }

  public void closeClaw() {
    clawSolenoid.set(Value.kReverse);

  }

  /*************/
  /*** CLAW ***/
  /*************/

  public void coneIntake() {
    clawSolenoid.set(Value.kReverse);
    SmartDashboard.putNumber("leftNeo current", leftNeo550.getOutputCurrent());
    SmartDashboard.putNumber("rightNeo current", rightNeo550.getOutputCurrent());

    // if (rightNeo550.getOutputCurrent() > 5 || leftNeo550.getOutputCurrent() > 5)
    // {
    // rightNeo550.set(0);
    // leftNeo550.set(0);
    // } else {
    rightNeo550.set(.75);
    leftNeo550.set(-.75);
    // }
  }

  public void outTake() {
    clawSolenoid.set(Value.kReverse);
    leftNeo550.set(1);
    rightNeo550.set(-1);
  }

  public void single() {
    clawSolenoid.set(Value.kForward);
    leftNeo550.set(.75);
    rightNeo550.set(-.75);
  }

  public void outTakeCube() {
    clawSolenoid.set(Value.kOff);
    leftNeo550.set(.25);
    rightNeo550.set(-.25);
  }

  public void outTake1() {
    clawSolenoid.set(Value.kForward);
    leftNeo550.set(.75);
    rightNeo550.set(-.75);
  }

  public void cubeIntake() {
    clawSolenoid.set(Value.kForward);
    // if (rightNeo550.getOutputCurrent() > 1.8 || leftNeo550.getOutputCurrent() >
    // 1.8) {
    // rightNeo550.set(0);
    // leftNeo550.set(0);
    // } else {
    rightNeo550.set(.75);
    leftNeo550.set(-.75);
    // }
  }

  public void stopClaw() {
    clawSolenoid.set(Value.kOff);
    leftNeo550.set(0);
    rightNeo550.set(0);
  }

  // -------------------------------------

  public void configClawMotor(CANSparkMax clawMotor, RelativeEncoder clawEncoder, SparkMaxPIDController clawController,
      boolean invert) {
    clawMotor.restoreFactoryDefaults();
    clawMotor.clearFaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(clawMotor, Usage.kAll);
    clawMotor.setSmartCurrentLimit(Setting.clawSetting.clawContinousCurrentLimit);
    clawMotor.setIdleMode(Setting.clawSetting.clawNeutralMode);
    clawMotor.setInverted(invert);
    clawEncoder.setVelocityConversionFactor(Setting.clawSetting.clawConversionVelocityFactor);
    clawEncoder.setPositionConversionFactor(Setting.clawSetting.clawConversionPositionFactor);
    clawMotor.enableVoltageCompensation(Setting.clawSetting.maxVoltage);
    clawController.setFeedbackDevice(clawEncoder);
    clawMotor.burnFlash();
  }

  private void PID() {
    clawSpeedPIDController.setP(Setting.clawSetting.clawP);
    clawSpeedPIDController.setI(Setting.clawSetting.clawI);
    clawSpeedPIDController.setD(Setting.clawSetting.clawD);
    clawSpeedPIDController.setFF(Setting.clawSetting.clawFF);
  }
}
