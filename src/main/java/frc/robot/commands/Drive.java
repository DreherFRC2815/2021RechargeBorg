/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {
  private final DriveTrain driveTrain;
  
  private final DoubleSupplier forwardsAxis;
  private final DoubleSupplier turnAxis;
  private final BooleanSupplier aim;
  private NetworkTableEntry tx;
  private NetworkTableEntry tv;
  private double x;
  private double v;
  private int abc = 1;
  private double turn;
  /**
   * Creates a new Drive.
   */
  public Drive(DriveTrain d, DoubleSupplier f, DoubleSupplier t, BooleanSupplier a) {
    driveTrain = d;
    forwardsAxis = f;
    turnAxis = t;
    aim = a;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    tv = table.getEntry("tv");
  }

  @Override
  public void execute() {
        //read values periodically
        v = tv.getDouble(0.0);
        x = tx.getDouble(0.0);
        
    
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("Target?", v);
        SmartDashboard.putNumber("ran ", abc);
        boolean a = aim.getAsBoolean();
        SmartDashboard.putBoolean("Aim?", a);

        if (a && v == 1.0) {
          turn = Math.pow(x / 28.9, 1.5);
            // driveTrain.drive(0, Math.pow(x / 28.9, 1.5));
        }else {
          turn = 0;
        }
        abc++;
        
    double forwardsValue = -forwardsAxis.getAsDouble();
    double turnValue = turnAxis.getAsDouble() * .8;

    if (Math.abs(forwardsValue) < .05) {
      forwardsValue = 0.0;
    }

    if (Math.abs(turnValue) < .05) {
      turnValue = 0.0;
    }

    double forwardsNegation = 1.0;
    double turnNegation = 1.0;

    if (forwardsValue < 0.0) {
      forwardsNegation = -1.0;
    }

    if (turnValue < 0.0) {
      turnNegation = -1.0;
    }

    driveTrain.drive(Math.pow(Math.abs(forwardsValue), 1.5) * forwardsNegation, Math.pow(Math.abs(turnValue), 1.5) * turnNegation + turn);
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
