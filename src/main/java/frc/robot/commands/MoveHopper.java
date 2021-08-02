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
  private final BooleanSupplier runButton2;
  private final BooleanSupplier backButton;
  private final BooleanSupplier backButton2;
  /**
   * Creates a new MoveHopper.
   */
  public MoveHopper(Hopper h, BooleanSupplier b, BooleanSupplier r, BooleanSupplier b2, BooleanSupplier r2) {
    hopper = h;
    runButton = b;
    runButton2 = b2;
    backButton = r;
    backButton2 = r2;

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
    boolean run = runButton.getAsBoolean() || runButton2.getAsBoolean();
    if (run) {
      hopper.set(.5);
    } else if (backButton.getAsBoolean() || backButton2.getAsBoolean()) {
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
