  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootShooter extends CommandBase {
  private final Shooter shooter;

  private final BooleanSupplier shoot;
  private final IntSupplier POV;

  private double power = 1;
  private boolean shooting = false;
  private boolean didSet = false;

  /** Creates a new ShootShooter. */
  public ShootShooter(Shooter s, BooleanSupplier b, IntSupplier p) {
    shooter = s;
    shoot = b;
    POV = p;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter.set(shoot.getAsBoolean());
    // double povAngle = xbox.getPOV();

    // if (povAngle == -1) {
    //   didSet = false;
    // } else if (!didSet) {
    //   if (povAngle == 0 && power < 1) {
    //     power += .05;
    //   } else if (povAngle == 180 && power > .05) {
    //     power -= .05;
    //   }

    //   didSet = true;
    // }
    int povAngle = POV.getAsInt();
    if (povAngle == -1) {
      didSet = false;
    } else if (!didSet) {
      if (povAngle == 0 && power < 1) {
        power += .025;
      } else if (povAngle == 180 && power > .025) {
        power -= .025;
      }
        didSet = true;
    }
    if (shoot.getAsBoolean()) {
      shooting = !shooting;
    }

    shooter.set(shooting ? power : 0);

    SmartDashboard.putNumber("Shooter RPM", shooter.getRPM());
    SmartDashboard.putNumber("Power", power);
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
