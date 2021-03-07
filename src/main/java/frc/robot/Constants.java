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
    public static int[] mainTalonPorts = {0, 2};  // talons which have the encoders
    public static int[] bareTalonPorts = {1, 3};  // talons with no encoders

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
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;
	
	/* Choose so that Talon does not report sensor out of phase */
	public static boolean kSensorPhase = true;

	/**
	 * Choose based on what direction you want to be positive,
	 * this does not affect motor invert. 
	 */
	public static boolean kMotorInvert = false;

	/**
	 * Gains used in Positon Closed Loop, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static Gains kGains = new Gains(0.15, 0.0, 1.0, 0.0, 0, 1.0);
}
