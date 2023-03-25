package frc.robot.subsystems.modules;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Setting;

public class CompressorModule {
    
    private final Compressor phCompressor;
    private static CompressorModule singleton;

    private CompressorModule() {
        this.phCompressor =  new Compressor(PneumaticsModuleType.REVPH);
    }

    public static CompressorModule getCompressorModule() {
        if (singleton == null) {
            singleton = new CompressorModule();
        }
        return singleton;
    }

    public void enableAnalog(double min, double max) {
        phCompressor.enableAnalog(min, max);
        SmartDashboard.putNumber("Pressure", phCompressor.getPressure());
    }

    public boolean isEnabled() {
        return phCompressor.isEnabled();
    }

    public double getPressure() {
        // phCompressor.makeCompressor().getPressure()
        return phCompressor.getPressure();
    }
    // public void enableDigital(){
    //     phCompressor.enableDigital();
    // }

    public void disableCompressor(){
        phCompressor.disable();
    }

    // public void setPressureAnalog(double pressure) {
    //     if (pressure == 220) {
    //         compressor.disable();
    //     } else if (pressure >= 60) {
    //         enableAnalog(Setting.compressor.absoluteMinPressure, Setting.compressor.absoluteMaxPressure);
    //     } else {
    //         enableAnalog(Setting.compressor.relativeMinPressure,Setting.compressor.absoluteMaxPressure);
    //     }
    // }
}
