/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class MoveTower extends CommandBase {
  private final Tower tower;

  private final BooleanSupplier upButton;
  private final BooleanSupplier upButton2;
  private final BooleanSupplier downButton;

  /**
   * Creates a new MoveTower.
   */
  public MoveTower(Tower t, BooleanSupplier u, BooleanSupplier u2, BooleanSupplier d) {
    tower = t;
    upButton = u;
    upButton2 = u2;
    downButton = d;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (upButton.getAsBoolean() || upButton2.getAsBoolean()) {
      tower.set(-.5);
    } else if (downButton.getAsBoolean()) {
      tower.set(.5);
    } else {
      tower.set(0);
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
