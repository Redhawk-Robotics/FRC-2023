package frc.robot.constants;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public interface Setting {
    /* Swerve Voltage Compensation */
    public static final double voltageComp = 8.5;    

    //RobotCharacterizations
    public static final double drivetrainTrackWidthMeters = Units.inchesToMeters(28.5);
    public static final double drivetrainWheelBaseMeters = Units.inchesToMeters(28.5);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    
    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;//kCoast
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveMotorInvert = false;
    public static final boolean angleMotorInvert = true;

    /* Swerve Profiling Values */
    public static final double maxVelocityMetersPerSecond = 5880.0 / 60.0 /
    (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * 0.10033 * Math.PI;      

    public static final double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
    Math.hypot(drivetrainTrackWidthMeters / 2.0, drivetrainWheelBaseMeters / 2.0);
    
    /* Swerve Current Limiting */
    public static final int angleContinousCurentLimit = 20;//FIXME
    public static final int driveContinousCurrentLimit = 40;//FIXME

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;//FIXME //try 1.0
    public static final double angleKI = 0.0;//FIXME
    public static final double angleKD = 0.0;//FIXME //try 0.1
    public static final double angleKFF = 0.0;//FIXME 

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;//FIXME maybe 0.01
    public static final double driveKI = 0.0;//FIXME
    public static final double driveKD = 0.0;//FIXME
    public static final double driveKFF = 0.0;//FIXME

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.0;//FIXME
    public static final double driveKV = 0.0;//FIXME Volt-seconds per meter (max voltage divided by max speed) 12/MAX SPEED
    public static final double driveKA = 0.0;//FIXME Volt-seconds^2 per meter (max voltage divided by max accel) 12/max acceleration
    
    /** SDS MK4i l1 - 8.14 : 1 */       
    /** SDS MK4i l2 - 6.75 : 1 */
    /** SDS MK4i l3 - 6.12 : 1 */
    ///public static final double driveGearRatio = (50.0/14.0) * (17.0/27.0) * (45.0/15.0); // 6.75:1
    public static final double driveGearRatio = (6.75 / 1.0);

    /** SDS MK4i l1 - (150 / 7) : 1 */
    /** SDS MK4i l2 - (150 / 7) : 1 */
    /** SDS MK4i l3 - (150 / 7) : 1 */
    public static final double angleGearRatio = (150.0 / 7.0); //Same for all of the MK4i modules 

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
    (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /*------- CANcoder Config ------- */
    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;//Change to true if moving the module right isnt positive
    
    // public static final AbsoluteSensorRange CANCODER_ABSOLUTE_SENSOR_RANGE = AbsoluteSensorRange.Unsigned_0_to_360;
    // public static final SensorInitializationStrategy CANCODER_SENSOR_INIT_STRATEGY = SensorInitializationStrategy.BootToAbsolutePosition;
    // public static final SensorTimeBase CANCODER_SENSOR_TIME_BASE = SensorTimeBase.PerSecond;        

    /* Swerve Kinematics 
    * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(drivetrainTrackWidthMeters / 2.0, drivetrainWheelBaseMeters / 2.0),
        // Front right
        new Translation2d(drivetrainTrackWidthMeters / 2.0, -drivetrainWheelBaseMeters / 2.0),
        // Back left
        new Translation2d(-drivetrainTrackWidthMeters / 2.0, drivetrainWheelBaseMeters / 2.0),
        // Back right
        new Translation2d(-drivetrainTrackWidthMeters / 2.0, -drivetrainWheelBaseMeters / 2.0)
    );
    public static final class intakePneumatics {
        public static final int leftForwardChan = 0;
        public static final int leftReverseChan = 1;

        public static final int rightForwardChan = 2;
        public static final int rightReverseChan = 3;
    }
    public static final class compressor {
        public static final int absoluteMinPressure = 0;
        public static final int absoluteMaxPressure = 120;

        public static final int relativeMinPressure = 60;
    }
    public static final class clawPneumatic {
        public static final int clawForwardChan = 4;
        public static final int clawReverseChan = 5; 
    }
    public static final class clawSetting{
        public static final double clawP = 0;
        public static final double clawI = 0;
        public static final double clawD = 0;
        public static final double clawFF = 0;
        public static final double maxVoltage = 0;
        public static final double clawConversionPositionFactor = 0;//FIXME need to find the conversion Factor
        public static final double clawConversionVelocityFactor = clawConversionPositionFactor / 60;

        public static final int clawContinousCurrentLimit = 40; 

        public static final IdleMode clawNeutralMode = IdleMode.kCoast;

    }

    public static final class armSetting{
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;

        public static final double armP = 0;
        public static final double armI = 0;
        public static final double armD = 0;
        public static final double armFF = 0;
        public static final double maxVoltage = 0;
        public static final double armConversionFactor = 0;//FIXME need to find the conversion Factor
        
        public static final int armContinousCurrentLimit = 40; 

        public static final IdleMode armNeutralMode = IdleMode.kCoast;

   }
   public static final class extenderSetting{
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;

        public static final double extenderP = 0;
        public static final double extenderI = 0;
        public static final double extenderD = 0;
        public static final double extenderFF = 0;
        public static final double maxVoltage = 0;
        public static final double extenderConversionFactor = 0;//FIXME need to find the conversion Factor
        
        public static final int extenderContinousCurrentLimit = 40; 

        public static final IdleMode extenderNeutralMode = IdleMode.kBrake;
   }
   public static final class wristSetting{
    public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;

    public static final double wristP = 0;
    public static final double wristI = 0;
    public static final double wristD = 0;
    public static final double wristFF = 0;
    public static final double maxVoltage = 0;
    public static final double wristConversionFactor = 0;//FIXME need to find the conversion Factor
    
    public static final int wristContinousCurrentLimit = 40; 

    public static final IdleMode wristNeutralMode = IdleMode.kBrake;
}
   
    public static final class AutoConstants {
        //tune later
        
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1.05;
        public static final double kPYController = 1.05;
        public static final double kPThetaController = 1;

        public static final HashMap<String, Command> EventMap = new HashMap<>();

    }
}
