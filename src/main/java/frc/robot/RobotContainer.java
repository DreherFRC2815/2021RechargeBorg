/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Aim;
import frc.robot.commands.Climb;
import frc.robot.commands.Drive;
import frc.robot.commands.MoveHopper;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveTower;
import frc.robot.commands.ShootShooter;
import frc.robot.commands.autoCommandGroups.TestAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

/**
 * Le robot container
 */
public class RobotContainer {
  // Controllers
  private XboxController mano;
  private XboxController mano2;
  private Joystick stick;
  // private final XboxController mano2 =
  // DriverStation.getInstance().getJoystickName(1).isEmpty() ? mano : new
  // XboxController(1);

  // Important stuff that isn't a controller, command, or subsytem
  private final String driveMode = "normal";
  public final static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Tower tower = new Tower();
  private final Hopper hopper = new Hopper();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();

  // Commands
  private Drive drive;
  private MoveTower moveTower;
  private MoveHopper moveHopper;
  private ShootShooter shootShooter;
  private MoveIntake moveIntake;
  private Climb climb;
  private Aim aim;
  private Trigger trigger;
  private Button button;

  public RobotContainer() {
    setup();
  }

  public void setup() {
    // button = new Button(() -> stick.getRawButton(8));
    // trigger = new Trigger(() -> stick.getRawButton(9));
    mano = new XboxController(0);
    stick = new Joystick(3);
    if (DriverStation.getInstance().getJoystickName(1).isEmpty()) {
      mano2 = mano;
    } else {
      mano2 = new XboxController(1);
    }

    if (driveMode.equals("trigger")) {
      drive = new Drive(driveTrain,
          () -> (mano.getTriggerAxis(GenericHID.Hand.kRight) - mano.getTriggerAxis(GenericHID.Hand.kLeft)),
          () -> mano.getX(GenericHID.Hand.kRight), () -> stick.getRawButton(9));
    } else {
      drive = new Drive(driveTrain, () -> mano.getY(GenericHID.Hand.kLeft), () -> mano.getX(GenericHID.Hand.kRight), () -> stick.getRawButton(9));
    }

    moveTower = new MoveTower(tower, () -> mano2.getBumper(GenericHID.Hand.kRight), () -> mano2.getXButton(),
        () -> mano2.getBumper(GenericHID.Hand.kLeft), () -> stick.getRawButton(1), () -> mano2.getAButton(), () -> stick.getRawButton(6));

    moveHopper = new MoveHopper(hopper, () -> mano2.getXButton(), () -> mano2.getAButton(), () -> stick.getRawButton(1), () -> stick.getRawButton(6));
    shootShooter = new ShootShooter(shooter, () -> mano2.getYButtonPressed(), () -> mano2.getPOV(), () -> stick.getRawButtonPressed(2));

    moveIntake = new MoveIntake(intake,
          () -> (mano.getTriggerAxis(GenericHID.Hand.kRight) - mano.getTriggerAxis(GenericHID.Hand.kLeft)),
           () -> mano.getBButton(), () -> stick.getRawButtonPressed(5), () -> stick.getRawButton(3),
            () -> stick.getRawButton(4));

    climb = new Climb(climber, () -> stick.getRawButtonPressed(11), () -> stick.getRawButtonPressed(12));

    // aim = new Aim(driveTrain, () -> stick.getRawButton(9));

    // button.whenPressed(new Aim(driveTrain));
    
    driveTrain.setDefaultCommand(drive);
    tower.setDefaultCommand(moveTower);
    hopper.setDefaultCommand(moveHopper);
    shooter.setDefaultCommand(shootShooter);
    intake.setDefaultCommand(moveIntake);
    climber.setDefaultCommand(climb);

    driveTrain.resetEncoders();
  }

  public static ADXRS450_Gyro getGyro() {
    return gyro;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SequentialCommandGroup getAutonomousCommand() {
    return new TestAuto(driveTrain, shooter, hopper, tower);
  }
}