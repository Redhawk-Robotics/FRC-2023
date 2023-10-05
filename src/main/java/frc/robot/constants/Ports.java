// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @Filename:Ports.java
 * @Purpose:This file contains the different ports of motors, and sensors
 * @Version:1.0
 * @Author: TEAM 8739 RedhawkRobotics
 * @Date: 02/07/23
 */

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.SwerveModuleConstants;

public interface Ports {

    public static final double stickDeadband = 0.1;// may be 0.01

    public static final class Gamepad {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
        public static final int DEBUGGER = 2;
    }

    public static final class Gyro {
        public static final int drivetrainPigeonID = 39;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    }

    public static final class PDH {
        public static final int port = 1;
    }

    public static final class Compressor {
        public static final int port = 1;
    }

    public static final class Arm {
        public static final int leftArm = 9;
        public static final int rightArm = 10;

        // public static final boolean leftArmMotorInvert = false;
        public static final boolean rightArmMotorInvert = false;
    }

    public static final class Extender {
        public static final int extender = 11;

        public static final boolean ExtenderMotorInvert = true;
    }

    public static final class Claw {
        public static final int leftClaw = 12;
        public static final int rightClaw = 13;

        public static final boolean leftClawMotorInvert = true;
        public static final boolean rightClawMotorInvert = true;
    }

    public static final class Wrist {
        public static final int wrist = 14;

        public static final boolean wristMotorInvert = false;
    }

    // FRONT_LEFT_MODULE
    public static final class frontLeftModule0 {
        public static final int frontLeftModuleDriveMotor = 55; // FIXME Set front left module drive motor ID
        public static final int frontLeftModuleSteerMotor = 56; // FIXME Set front left module steer motor ID
        public static final int frontLeftModuleSteerEncoder = 11; // FIXME Set front left steer encoder ID
        public static final Rotation2d frontLeftModuleSteerOffSet = Rotation2d.fromDegrees(0); // FIXME
        // Measure
        // and set
        // front left
        // steer
        // offset
        // 297.861328125
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(frontLeftModuleDriveMotor,
                frontLeftModuleSteerMotor, frontLeftModuleSteerEncoder, frontLeftModuleSteerOffSet);
    }

    // FRONT_RIGHT_MODULE
    public static final class frontRightModule1 {
        public static final int frontRightModuleDriveMotor = 3; // FIXME Set front right drive motor ID
        public static final int frontRightModuleSteerMotor = 4; // FIXME Set front right steer motor ID
        public static final int frontRightModuleSteerEncoder = 22; // FIXME Set front right steer encoder ID
        public static final Rotation2d frontRightModuleSteerOffSet = Rotation2d
                .fromDegrees(0); // FIXME
        // Measure
        // and set
        // front
        // right
        // steer
        // offset
        // 247.060546875
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(frontRightModuleDriveMotor,
                frontRightModuleSteerMotor, frontRightModuleSteerEncoder, frontRightModuleSteerOffSet);
    }

    // BACK_LEFT_MODULE
    public static final class backLeftModule2 {
        public static final int backLeftModuleDriveMotor = 5; // FIXME Set back left drive motor ID
        public static final int backLeftModuleSteerMotor = 6; // FIXME Set back left steer motor ID
        public static final int backleftModuleSteerEncoder = 33; // FIXME Set back left steer encoder ID
        public static final Rotation2d backLeftModuleSteerOffSet = Rotation2d
                .fromDegrees(0); // FIXME
        // Measure
        // and set back
        // left steer
        // offset
        // 231.328125
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(backLeftModuleDriveMotor,
                backLeftModuleSteerMotor, backleftModuleSteerEncoder, backLeftModuleSteerOffSet);
    }

    // BACK_RIGHT_MODULE
    public static final class backRightModule3 {
        public static final int backRightModuleDriveMotor = 7; // FIXME Set back right drive motor ID
        public static final int backRightModuleSteerMotor = 8; // FIXME Set back right steer motor ID
        public static final int backRightModuleSteerEncoder = 44; // FIXME Set back right steer encoder ID
        public static final Rotation2d backRightModuleSteerOffSet = Rotation2d
                .fromDegrees(0); // FIXME
        // Measure and
        // set back
        // right steer
        // offset
        // 216.2109375
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(backRightModuleDriveMotor,
                backRightModuleSteerMotor, backRightModuleSteerEncoder, backRightModuleSteerOffSet);
    }
}
