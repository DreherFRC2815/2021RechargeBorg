// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class AutoSetShooter extends CommandBase {
  private final Shooter shooter;

  private final double power;

  private boolean done = false;

  /** Creates a new AutoShooter. */
  public AutoSetShooter(Shooter y, double p) {
    shooter = y;
    power = p;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.set(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooter.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooter.getRPM() < power) {
      done = false;
    } else {
      done = true;
    }
    SmartDashboard.putBoolean("Shooter on?", done);
    return done;
  }
}
