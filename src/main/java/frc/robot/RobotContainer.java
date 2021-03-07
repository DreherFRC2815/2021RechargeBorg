/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.Drive;
import frc.robot.commands.MoveHopper;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveTower;
import frc.robot.commands.ShootShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

/**
 * Le robot container
 */
public class RobotContainer {
    // Important things that aren't subsystems
    private final XboxController mano = new XboxController(0);
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    // Subsystems
    private final DriveTrain driveTrain = new DriveTrain();
    private final Tower verticalIndexingSystem = new Tower();
    private final Hopper hopper = new Hopper();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();

    // Commands
    private final Drive drive = new Drive(
      driveTrain,
      () -> mano.getY(GenericHID.Hand.kLeft),
      () -> mano.getX(GenericHID.Hand.kRight)
    );
    private final MoveTower moveTower = new MoveTower(
      verticalIndexingSystem,
      () -> mano.getBumper(GenericHID.Hand.kLeft),
      () -> mano.getBumper(GenericHID.Hand.kRight)
    );
    private final MoveHopper moveHopper = new MoveHopper(hopper, () -> mano.getXButton());
    private final ShootShooter shootShooter = new ShootShooter(shooter, () -> mano.getYButton(), mano);
    private final MoveIntake moveIntake = new MoveIntake(intake, () -> mano.getAButton());

    public RobotContainer() {
      driveTrain.setDefaultCommand(drive);
      verticalIndexingSystem.setDefaultCommand(moveTower);
      hopper.setDefaultCommand(moveHopper);
      shooter.setDefaultCommand(shootShooter);
      intake.setDefaultCommand(moveIntake);
    
      configureButtonBindings();
    }

    private void configureButtonBindings() {

    }
  
    public Command getAutonomousCommand() {
      // An ExampleCommand will run in autonomous
      return null;
    }
}
