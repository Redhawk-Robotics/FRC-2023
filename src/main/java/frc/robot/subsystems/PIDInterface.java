package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PIDInterface extends SubsystemBase {
    public void setPosition(double targetPosition) {
    }

    public double getCurrentPosition() {
        return 0;
    }

    public SparkMaxPIDController getController() {
        return null;
    }
}
