/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain2;

public class Drive extends CommandBase {
  private final DriveTrain2 driveTrain;
  
  private final DoubleSupplier forwardsAxis;
  private final DoubleSupplier turnAxis;

  /**
   * Creates a new Drive.
   */
  public Drive(DriveTrain2 d, DoubleSupplier f, DoubleSupplier t) {
    driveTrain = d;
    forwardsAxis = f;
    turnAxis = t;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardsValue = -forwardsAxis.getAsDouble();
    double turnValue = turnAxis.getAsDouble() * .8;

    if (Math.abs(forwardsValue) < .05) {
      forwardsValue = 0.0;
    }

    if (Math.abs(turnValue) < .05) {
      turnValue = 0.0;
    }

    driveTrain.curvatureDrive(forwardsValue, turnValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
