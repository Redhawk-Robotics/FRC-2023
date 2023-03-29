// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Setting;
import frc.robot.constants.Ports;


public class clawTest extends SubsystemBase {
  /** Creates a new clawTest. */
  private final CANSparkMax leftClaw, rightClaw;

  private final DoubleSolenoid clawOpen;

  public clawTest() {
// ------------------------------------- CLAW

leftClaw = new CANSparkMax(Ports.Claw.leftClaw, MotorType.kBrushless); // 12
leftClaw.setInverted(true);
// leftClaw.setIdleMode(IdleMode.kCoast);

// RIGHT CLAW
rightClaw = new CANSparkMax(Ports.Claw.rightClaw, MotorType.kBrushless); // 13
rightClaw.setInverted(true);
// rightClaw.setIdleMode(IdleMode.kCoast);

clawOpen = new DoubleSolenoid(PneumaticsModuleType.REVPH, Setting.clawPneumatic.clawForwardChan,
Setting.clawPneumatic.clawReverseChan);

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /*************/
  /*** CLAW ***/
  /*************/

  public void coneIntake() {
    clawOpen.set(Value.kForward);
    if(rightClaw.getOutputCurrent() < 5 && leftClaw.getOutputCurrent() < 5){
    rightClaw.set(.4);  
    leftClaw.set(-.4);
    }else{
      rightClaw.set(0);  
      leftClaw.set(0);
    }
  }

  public void outTake() {
    clawOpen.set(Value.kReverse);
    leftClaw.set(.25);
    rightClaw.set(-.25);
  }

  public void cubeIntake() {
    clawOpen.set(Value.kReverse);
    if(rightClaw.getOutputCurrent() < 5 && leftClaw.getOutputCurrent() < 5){
      rightClaw.set(.5);  
      leftClaw.set(-.5);
      }else{
        rightClaw.set(0);  
        leftClaw.set(0);
      }
  }

  public void stopClaw() {
    // clawOpen.set(Value.kOff);
    leftClaw.set(0);
    rightClaw.set(0);
  }

  // -------------------------------------
}
