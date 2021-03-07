// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveDistance extends CommandBase {
  private final DriveTrain driveTrain;

  private final Timer timer = new Timer();

  private final double inches;
  private final double maxPower;

  /**
   * Creates a new AutoDriveDistance.
   * (drivetrain, wheel circumference (inches), gearbox ratio, distance inches, power)
   */
  public AutoDriveDistance(DriveTrain d, double i, double p) {
    driveTrain = d;
    inches = i;
    maxPower = p;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    
    driveTrain.configPositionalDrive(maxPower);
    driveTrain.driveDistance(inches);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.configNormal();
    driveTrain.drive(0, 0);

    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveTrain.getAvgPower() == 0 && timer.get() > .5);
  }
}
