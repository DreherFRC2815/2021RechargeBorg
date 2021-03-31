/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * Robot constants
 */
public class Constants {
    // the talons for driving and moving around and stuff
    public static final int[] driveTalonPorts = {0, 1, 2, 3};
    public static final int[] mainTalonPorts = {0, 3};  // talons which have the encoders
    public static final int[] bareTalonPorts = {1, 2};  // talons with no encoders

    // weird thingy on robot that moves balls from collector to tower
    public static final int leftHopperPort = 0;
    public static final int rightHopperPort = 2;

    // ports for the spark maxes for powering the two neos which shoot balls
    public static final int rightShooterPort = 5;
    public static final int leftShooterPort = 4;

    // collector/intake thingy that picks up balls
    public static final int intakePort = 6;

    // the spark for the redline which powers the tower
    public static final int towerPort = 1;
    
    public static final double wheelCircumference = 18.85;  // inches
    public static final double wheelCircumferenceMeters = 0.47877871986;  // meters
    public static final double wheelDiameterMeters = 0.1524;  // meters
    public static final double gearboxRatio = 10.91;

    public static final double cameraHeight = -1;
    public static final double cameraFovX = 49.7;
    public static final double cameraFovY = 59.6;

    // PathWeaver stuff
    public static final double ksVolts = 0.742;
    public static final double kvVoltSecondsPerMeter = 3.29;
    public static final double kaVoltSecondsSquaredPerMeter = 0.44;

    public static final double kPDriveVel = 0.08;

    public static final double kTrackwidthMeters = 0.5369566868741729;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 0.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = .25;

    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;

    // public static final double kMagEncoderDistancePerPulse = 0.47877871986 / 4096;
    public static final double kMagEncoderDistancePerPulse = (Math.PI * wheelDiameterMeters * 2.0) / 4096.0;
}
