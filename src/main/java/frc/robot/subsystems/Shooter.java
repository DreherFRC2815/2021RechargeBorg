// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leftSparkMax;
  private final CANSparkMax rightSparkMax;

  /** Creates a new Shooter. */
  public Shooter() {
    leftSparkMax = new CANSparkMax(Constants.leftShooterPort, MotorType.kBrushless);
    rightSparkMax = new CANSparkMax(Constants.rightShooterPort, MotorType.kBrushless);

    leftSparkMax.restoreFactoryDefaults();
    rightSparkMax.restoreFactoryDefaults();

    leftSparkMax.setIdleMode(IdleMode.kCoast);
    rightSparkMax.setIdleMode(IdleMode.kCoast);
  }

  public void set(double r) {
    double rpm = getRPM();
    double error = r / (rpm + .000000001);

    SmartDashboard.putNumber("Shooter Error", error);
    SmartDashboard.putNumber("Shooter RPM", rpm);

    leftSparkMax.set(error);
    rightSparkMax.set(-error);
  }

  public void setPower(double p) {
    SmartDashboard.putNumber("Input P", p);
    SmartDashboard.putNumber("Output RPM", 5700.0 * p);
    set(p * 5700.0);
  }

  public double getRPM() {
    return (leftSparkMax.getEncoder().getVelocity() - rightSparkMax.getEncoder().getVelocity()) / 2;
  }
  
  private PIDController leftPID = new PIDController(.0001, 0, .00001);
  private PIDController rightPID = new PIDController(.0001, 0, .00001);

  public void setRPM(double rpm){
    leftSparkMax.set(leftPID.calculate(leftSparkMax.getEncoder().getVelocity(), rpm));
    rightSparkMax.set(rightPID.calculate(rightSparkMax.getEncoder().getVelocity(), -rpm));
    
  }
  
  public double[] getCurrents() {
    double[] currents = {leftSparkMax.getOutputCurrent(), rightSparkMax.getOutputCurrent()};
    return currents;
  }

  public double[] getVoltages() {
    double[] voltages = {leftSparkMax.getBusVoltage(), rightSparkMax.getBusVoltage()};
    return voltages;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
