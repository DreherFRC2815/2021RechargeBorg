package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
    private final Climber climber;
    private final BooleanSupplier lift;
    private final BooleanSupplier engage;
    public Climb(Climber c, BooleanSupplier l, BooleanSupplier e) {
        climber = c;
        engage = e;
        lift = l;
        addRequirements(climber);

        
    }
// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.init();
  }
    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean toggle = engage.getAsBoolean();
    if (toggle) {
        climber.toggle();
    }
    boolean raise = lift.getAsBoolean();
    if (raise) {
      climber.switchPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
	public void toggle() {
    climber.toggle();
	}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

