/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class MoveHopper extends CommandBase {
  private final Hopper hopper;
  
  private final BooleanSupplier runButton;
  private final BooleanSupplier backButton;

  /**
   * Creates a new MoveHopper.
   */
  public MoveHopper(Hopper h, BooleanSupplier b, BooleanSupplier r) {
    hopper = h;
    runButton = b;
    backButton = r;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (runButton.getAsBoolean()) {
      hopper.set(.5);
    } else if (backButton.getAsBoolean()) {
      hopper.set(-.5);
    } else {
      hopper.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
