/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonSRX[] talons = new WPI_TalonSRX[4];

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
      talons[Constants.mainTalonPorts[i]].setInverted(false);
      
      talons[Constants.bareTalonPorts[i]].follow(talons[Constants.mainTalonPorts[i]]);
      talons[Constants.bareTalonPorts[i]].setInverted(InvertType.FollowMaster);
    }
  }

  public void configPositionDrive() {
    for (WPI_TalonSRX talon : talons) {
      talon.configFactoryDefault();
      talon.setSensorPhase(true);
      talon.setSelectedSensorPosition(0);

      talon.configNominalOutputForward(0.0);
      talon.configNominalOutputReverse(0.0);
      talon.configPeakOutputForward(1.0);
      talon.configPeakOutputReverse(1.0);

      talon.configAllowableClosedloopError(0, 0.0);

      talon.config_kF(0, 0.0);
      talon.config_kP(0, 0.15);
      talon.config_kI(0, 0.0);
      talon.config_kD(0, 1.0);
    }

    talons[Constants.mainTalonPorts[1]].setInverted(true);

    for (int i = 0; i < 2; i++) {
      talons[Constants.bareTalonPorts[i]].follow(talons[Constants.mainTalonPorts[i]]);
      talons[Constants.bareTalonPorts[i]].setInverted(InvertType.FollowMaster);
    }
  }

  public void driveTicks(double ticks) {
    talons[Constants.mainTalonPorts[0]].set(ControlMode.Position, ticks);
    talons[Constants.mainTalonPorts[1]].set(ControlMode.Position, ticks);
    // talons[Constants.mainTalonPorts[1]].set(ControlMode.MotionProfile, );

  }

  public void drive(double f, double t) {
    SmartDashboard.putNumber("forw", f);
    SmartDashboard.putNumber("turn", t);

    double eLeft = talons[Constants.mainTalonPorts[0]].getSelectedSensorPosition();
    double eRight = talons[Constants.mainTalonPorts[1]].getSelectedSensorPosition();

    SmartDashboard.putNumber("eLeft", eLeft);
    SmartDashboard.putNumber("eRight", eRight);

    double eVLeft = talons[Constants.mainTalonPorts[0]].getSelectedSensorVelocity();
    double eVRight = talons[Constants.mainTalonPorts[1]].getSelectedSensorVelocity();

    if (t == 0) {
      t = (eVLeft + eVRight) / 768 * -Math.abs(f);
      SmartDashboard.putNumber("tError", t);
    }
    
    botDrive.curvatureDrive(f, t, true);
  }

  public double getAvgPower() {
    double power = 0;

    for (WPI_TalonSRX talon : talons) {
      power += talon.get();
    }

    return power / talons.length;
  }

  public void resetEncoders() {
    talons[Constants.mainTalonPorts[0]].setSelectedSensorPosition(0);
    talons[Constants.mainTalonPorts[1]].setSelectedSensorPosition(0);
  }

  public double[] getEncoders() {
    return new double[] {
      talons[Constants.mainTalonPorts[0]].getSelectedSensorPosition(),
      talons[Constants.mainTalonPorts[1]].getSelectedSensorPosition()
    };
  }

  public double[] getEncodersVelocity() {
    return new double[] {
      talons[Constants.mainTalonPorts[0]].getSelectedSensorVelocity(),
      talons[Constants.mainTalonPorts[1]].getSelectedSensorVelocity()
    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
