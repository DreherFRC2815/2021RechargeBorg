/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonSRX[] talons = new WPI_TalonSRX[4];
  private final Faults[] faultss = new Faults[2];

  private final DifferentialDrive botDrive;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    for (int i = 0; i < talons.length; i++) {
      talons[i] = new WPI_TalonSRX(Constants.driveTalonPorts[i]);
    }

    configNormal();

    botDrive = new DifferentialDrive(talons[Constants.mainTalonPorts[0]], talons[Constants.mainTalonPorts[1]]);
    botDrive.setRightSideInverted(true);
  }

  public void configNormal() {
    for (WPI_TalonSRX talon : talons) {
      talon.configFactoryDefault();
      talon.setSensorPhase(true);
      talon.setSelectedSensorPosition(0);
    }

    for (int i = 0; i < 2; i++) {
      talons[Constants.bareTalonPorts[i]].follow(talons[Constants.mainTalonPorts[i]]);
      talons[Constants.bareTalonPorts[i]].setInverted(InvertType.FollowMaster);
    }
  }

  public void configPositionalDrive(double maxP) {
    for (WPI_TalonSRX talon : talons) {
      talon.configFactoryDefault();
      talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
      talon.setSensorPhase(Constants.kSensorPhase);
      talon.setInverted(Constants.kMotorInvert);
      talon.configNominalOutputForward(0, Constants.kTimeoutMs);
      talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
      talon.configPeakOutputForward(maxP, Constants.kTimeoutMs);
      talon.configPeakOutputReverse(-maxP, Constants.kTimeoutMs);
      talon.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
      talon.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
      talon.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
      talon.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
      talon.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
  
      int absolutePosition = talon.getSensorCollection().getPulseWidthPosition() & 0xFFF;

      if (Constants.kSensorPhase) { absolutePosition *= -1; }
      if (Constants.kMotorInvert) { absolutePosition *= -1; }

      talon.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }
  }

  public void drive(double f, double t) {
    SmartDashboard.putNumber("forw", f);
    SmartDashboard.putNumber("turn", t);
    
    botDrive.curvatureDrive(f, t, true);

    // talons[Constants.mainTalonPorts[0]].getFaults(faultss[0]);
    // talons[Constants.mainTalonPorts[1]].getFaults(faultss[1]);
  }

  public void driveDistance(double inches) {
    double targetPositionRotations = (inches / Constants.wheelCircumference) * 4096;

    for (WPI_TalonSRX talon : talons) {
      talon.set(ControlMode.Position, targetPositionRotations);
    }
  }

  public double getAvgPower() {
    double power = 0;

    for (WPI_TalonSRX talon : talons) {
      power += talon.get();
    }

    return power / talons.length;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
