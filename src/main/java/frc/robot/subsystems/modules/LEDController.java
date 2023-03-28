// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.modules;

import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
  /** Creates a new LEDController. */
  private LEDColor LED;

    // Motor that controlls the LEDs
    private final PWMSparkMax controller;

    // Stopwatch to check when to start overriding manual updates
    private final StopWatch lastUpdate;
    private double manualTime;

  private static LEDController instance;

    public static LEDController getInstance() {
        if (instance == null) {
            instance = new LEDController();
        }
        return instance;
    }
    
  public LEDController() {
    this.controller = new PWMSparkMax(0);//Make a constant
    this.lastUpdate = new StopWatch();
    setColor(LEDColor.OFF);

  }
  public void setColor(LEDColor color, double time) {
    LED = color;
    manualTime = time;
    // lastUpdate.reset();
  }
  public void setColor(LEDColor color) {
    setColor(color, 0);//fix time
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
