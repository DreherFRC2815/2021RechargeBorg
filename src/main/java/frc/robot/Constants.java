/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Robot constants
 */
public class Constants {
    // the talons for driving and moving around and stuff
    public static int[] driveTalonPorts = {0, 1, 2, 3};
    public static int[] mainTalonPorts = {0, 3};  // talons which have the encoders
    public static int[] bareTalonPorts = {1, 2};  // talons with no encoders

    // weird thingy on robot that moves balls from collector to tower
    public static int leftHopperPort = 0;
    public static int rightHopperPort = 2;

    // ports for the spark maxes for powering the two neos which shoot balls
    public static int rightShooterPort = 5;
    public static int leftShooterPort = 4;

    // collector/intake thingy that picks up balls
    public static int intakePort = 6;

    // the spark for the redline which powers the tower
    public static int towerPort = 1;
    
    public static double wheelCircumference = 18.5;
    public static double gearboxRatio = 10.91;

    public static double cameraHeight = -1;
    public static double cameraFovX = 49.7;
    public static double cameraFovY = 59.6;

	/**
	 * Gains used in Positon Closed Loop, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static Gains kGains = new Gains(0.15, 0.0, 1.0, 0.0, 0, 1.0);
}
