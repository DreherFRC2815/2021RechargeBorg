// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leftSparkMax;
  private final CANSparkMax rightSparkMax;

  /** Creates a new Shooter. */
  public Shooter() {
    leftSparkMax = new CANSparkMax(Constants.leftShooterPort, MotorType.kBrushless);
    rightSparkMax = new CANSparkMax(Constants.rightShooterPort, MotorType.kBrushless);

    leftSparkMax.setIdleMode(IdleMode.kCoast);
    rightSparkMax.setIdleMode(IdleMode.kCoast);
  }

  public void set(boolean on) {
    if (on) {
      leftSparkMax.set(1);
      rightSparkMax.set(-1);
    } else {
      leftSparkMax.set(0);
      rightSparkMax.set(0);
    }
  }

  public void set(double p) {
    leftSparkMax.set(p);
    rightSparkMax.set(-p);
  }

  public double getRPM() {
    return (leftSparkMax.getEncoder().getVelocity() - rightSparkMax.getEncoder().getVelocity()) / 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
