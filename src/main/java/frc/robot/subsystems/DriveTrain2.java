// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MagEncoder;
import frc.robot.RobotContainer;

public class DriveTrain2 extends SubsystemBase {
  private final WPI_TalonSRX leftMainTalon = new WPI_TalonSRX(Constants.mainTalonPorts[0]);
  private final WPI_TalonSRX leftBareTalon = new WPI_TalonSRX(Constants.bareTalonPorts[0]);
  private final WPI_TalonSRX rightMainTalon = new WPI_TalonSRX(Constants.mainTalonPorts[1]);
  private final WPI_TalonSRX rightBareTalon = new WPI_TalonSRX(Constants.bareTalonPorts[1]);

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(
    leftMainTalon,
    leftBareTalon
  );

  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(
    rightMainTalon,
    rightBareTalon
  );

  private final MagEncoder leftEncoder = new MagEncoder(leftMainTalon);
  private final MagEncoder rightEncoder = new MagEncoder(rightMainTalon);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

  private final ADXRS450_Gyro gyro;

  private final DifferentialDriveOdometry differentialDriveOdometry;

  /** Creates a new DriveTrain2. */
  public DriveTrain2() {
    leftEncoder.setDistancePerPulse(Constants.kMagEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(Constants.kMagEncoderDistancePerPulse);

    resetEncoders();

    gyro = RobotContainer.getGyro();
    differentialDriveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    differentialDriveOdometry.update(gyro.getRotation2d(), leftEncoder.getDistance(),
                      rightEncoder.getDistance());
  }

  private void logTelemetry() {
    double leftEncoderValue = getLeftEncoder().getDistance();
    double rightEncoderValue = getRightEncoder().getDistance();

    SmartDashboard.putNumber("Left Encoder", leftEncoderValue);
    SmartDashboard.putNumber("Right Encoder", rightEncoderValue);
    SmartDashboard.putNumber("Encoder Diff", leftEncoderValue + rightEncoderValue);
    SmartDashboard.putNumber("Gyro Heading", getHeading());
    SmartDashboard.putNumber("Gyro Rate", getTurnRate());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return differentialDriveOdometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    differentialDriveOdometry.resetPosition(pose, gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
    logTelemetry();
  }

  /**
   * Drives the robot using curvature drive (essentially arcade drive) controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void teleopDrive(double fwd, double rot) {
    // double eVLeft = leftMainTalon.getSelectedSensorVelocity();
    // double eVRight = rightMainTalon.getSelectedSensorVelocity();

    // if (rot == 0) {
    //   rot = (eVLeft + eVRight) / 768 * Math.abs(fwd);
    //   SmartDashboard.putNumber("tError", rot);
    // }

    differentialDrive.curvatureDrive(fwd, rot, true);
    logTelemetry();
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
    differentialDrive.feed();
    logTelemetry();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public MagEncoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public MagEncoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }
}
