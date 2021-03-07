/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  private final Spark leftMotor;
  private final Spark rightMotor;

  /**
   * Creates a new Hopper.
   */
  public Hopper() {
    leftMotor = new Spark(Constants.leftHopperPort);
    rightMotor = new Spark(Constants.rightHopperPort);
  }

  public void set(double p) {
    leftMotor.set(p);
    rightMotor.set(p);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
