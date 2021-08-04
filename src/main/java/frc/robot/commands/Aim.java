// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class Aim extends CommandBase {
  private BooleanSupplier aim;
  private DriveTrain driveTrain;
  private NetworkTableEntry tx;
  private double x;
  private NetworkTableEntry tv;
  private boolean v;
    public Aim (DriveTrain d) {
      driveTrain = d;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    tv = table.getEntry("tv");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //read values periodically
    v = tv.getBoolean(false);
    x = tx.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putBoolean("Target?", v);

    if (v) {
        driveTrain.drive(0, x);
    }
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
