package frc.robot.subsystems.modules;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.constants.Ports;

public class PigeonModule {

    private final Pigeon2 m_Pigeon;
    private static PigeonModule singleton;

    private PigeonModule() {
        this.m_Pigeon = new Pigeon2(Ports.Gyro.drivetrainPigeonID);
    }

    public static PigeonModule getPigeonModule() {
        if (singleton == null) {
            singleton = new PigeonModule();
        }
        return singleton;
    }

    public double getPitch() {
        return this.m_Pigeon.getPitch();
    }

    public void setYaw(double yaw) {
        this.m_Pigeon.setYaw(yaw);
    }

    public double getYaw() {
        return this.m_Pigeon.getYaw();
    }

    public double getAnglePerpendicularToGroundDEG() {
        return this.m_Pigeon.getYaw();
    }

    public double getRoll() {
        return this.m_Pigeon.getRoll();
    }
}
