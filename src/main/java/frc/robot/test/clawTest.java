// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Setting;
import frc.robot.subsystems.modules.SparkMaxModules;
import frc.robot.constants.Ports;

@Deprecated
public class clawTest extends SubsystemBase {
  /** Creates a new clawTest. */
  private final CANSparkMax leftClaw, rightClaw;

  private final DoubleSolenoid clawOpen;

  public clawTest() {
    // ------------------------------------- CLAW

    leftClaw = SparkMaxModules.leftClaw;
    rightClaw = SparkMaxModules.rightClaw;

    clawOpen = new DoubleSolenoid(PneumaticsModuleType.REVPH, Setting.clawPneumatic.clawReverseChan,
        Setting.clawPneumatic.clawForwardChan);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*************/
  /*** CLAW ***/
  /*************/

  public void coneIntake() {
    clawOpen.set(Value.kReverse);
    if (rightClaw.getOutputCurrent() < 5 && leftClaw.getOutputCurrent() < 5) {
      leftClaw.set(-.4);
      rightClaw.set(.4);
    } else {
      leftClaw.set(0);
      rightClaw.set(0);
    }
  }

  public void outTake() {
    clawOpen.set(Value.kForward);
    leftClaw.set(.5);
    rightClaw.set(-.5);
  }

  public void cubeIntake() {
    clawOpen.set(Value.kForward);
    if (rightClaw.getOutputCurrent() < 5 && leftClaw.getOutputCurrent() < 5) {
      leftClaw.set(-.5);
      rightClaw.set(.5);
    } else {
      leftClaw.set(0);
      rightClaw.set(0);
    }
  }

  public void stopClaw() {
    // clawOpen.set(Value.kOff);
    leftClaw.set(0);
    rightClaw.set(0);
  }

  public void openClaw() {
    clawOpen.set(Value.kForward);
  }

  public void closeClaw() {
    clawOpen.set(Value.kReverse);
  }

  // -------------------------------------
}
