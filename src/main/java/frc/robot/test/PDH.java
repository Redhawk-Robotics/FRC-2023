// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;

@Deprecated
public class PDH extends SubsystemBase {
  /** Creates a new PDH. */
  // private final PowerDistribution powerDistributionHub = new
  // PowerDistribution(Ports.PDH.port, ModuleType.kRev);

  public PDH() {
    // powerDistributionHub.setSwitchableChannel(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Gets the voltage going into the PDP, in Volts and stores in SmartDashboard
    // double voltage = powerDistributionHub.getVoltage();

    // The PDP returns the voltage in increments of 0.05 Volts.
    // SmartDashboard.putNumber("PDH Voltage", voltage);

    // Retrieves the temperature of the PDP, in degrees Celsius.
    // double temperatureCelsius = powerDistributionHub.getTemperature();
    // SmartDashboard.putNumber("Temperature", temperatureCelsius);

    // Get the total current of all channels.
    // double totalCurrent = powerDistributionHub.getTotalCurrent();
    // SmartDashboard.putNumber("Total Current", totalCurrent);

    // Get the total power of all channels.
    // Power is the bus voltage multiplied by the current with the units Watts.
    // double totalPower = powerDistributionHub.getTotalPower();
    // SmartDashboard.putNumber("Total Power", totalPower);

    // Get the total energy of all channels.
    // Energy is the power summed over time with units Joules.
    // double totalEnergy = powerDistributionHub.getTotalEnergy();
    // SmartDashboard.putNumber("Total Energy", totalEnergy);

    // Get the current going through channel 7, in Amperes.
    // The PDP returns the current in increments of 0.125A.
    // At low currents the current readings tend to be less accurate.
    // double current7 = powerDistributionHub.getCurrent(7);
    // double current8 = powerDistributionHub.getCurrent(8);
    // double current9 = powerDistributionHub.getCurrent(9);
    // SmartDashboard.putNumber("Current Channel 7", current7);
  }
}
