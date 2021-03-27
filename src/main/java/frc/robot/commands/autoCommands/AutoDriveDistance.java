// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveDistance extends CommandBase {
  private final DriveTrain driveTrain;

  private final double encoderTicks;
  private boolean done = false;

  /**
   * Creates a new AutoDriveDistance.
   * (drivetrain, wheel circumference (inches), gearbox ratio, distance inches, power)
   */
  public AutoDriveDistance(DriveTrain d, double i) {
    driveTrain = d;
    encoderTicks = (i / Constants.wheelCircumference) * 4096;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] encoderValues = driveTrain.getEncoders();
    double encoderAvg = (encoderValues[0] - encoderValues[1]) / 2.0;

    SmartDashboard.putNumber("encoderAvg", encoderAvg);

    if (encoderTicks > encoderAvg) {
      driveTrain.drive(.3, 0);
    } else {
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0);
    driveTrain.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
