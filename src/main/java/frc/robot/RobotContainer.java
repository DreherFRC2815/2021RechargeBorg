/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive;
import frc.robot.commands.MoveHopper;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveTower;
import frc.robot.commands.ShootShooter;
import frc.robot.commands.autoCommandGroups.TestAuto;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain2;
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
  // private final XboxController mano2 =
  // DriverStation.getInstance().getJoystickName(1).isEmpty() ? mano : new
  // XboxController(1);

  // Important stuff that isn't a controller, command, or subsytem
  private final String driveMode = "normal";
  public final static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  RamseteController controller1 = new RamseteController();
  RamseteController controller2 = new RamseteController(2.1, 0.8);

  // Subsystems
  // private final DriveTrain driveTrain = new DriveTrain();
  private final DriveTrain2 driveTrain2 = new DriveTrain2();
  private final Tower tower = new Tower();
  private final Hopper hopper = new Hopper();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();

  // Commands
  private Drive drive;
  private MoveTower moveTower;
  private MoveHopper moveHopper;
  private ShootShooter shootShooter;
  private MoveIntake moveIntake;

  public RobotContainer() {
    setup();
  }

  public void setup() {
    mano = new XboxController(0);

    if (DriverStation.getInstance().getJoystickName(1).isEmpty()) {
      mano2 = mano;
    } else {
      mano2 = new XboxController(1);
    }

    if (driveMode.equals("trigger")) {
      drive = new Drive(driveTrain2,
          () -> (mano.getTriggerAxis(GenericHID.Hand.kRight) - mano.getTriggerAxis(GenericHID.Hand.kLeft)),
          () -> mano.getX(GenericHID.Hand.kRight));
    } else {
      drive = new Drive(driveTrain2, () -> mano.getY(GenericHID.Hand.kLeft), () -> mano.getX(GenericHID.Hand.kRight));
    }

    moveTower = new MoveTower(tower, () -> mano2.getBumper(GenericHID.Hand.kRight), () -> mano2.getXButton(),
        () -> mano2.getBumper(GenericHID.Hand.kLeft));

    moveHopper = new MoveHopper(hopper, () -> mano2.getXButton(), () -> mano2.getAButton());
    shootShooter = new ShootShooter(shooter, () -> mano2.getYButtonPressed(), () -> mano2.getPOV());

    if (mano2.equals(mano)) {
      moveIntake = new MoveIntake(intake,
          () -> (mano.getTriggerAxis(GenericHID.Hand.kRight) - mano.getTriggerAxis(GenericHID.Hand.kLeft)));
    } else {
      moveIntake = new MoveIntake(intake, () -> mano2.getY(GenericHID.Hand.kLeft));
    }

    driveTrain2.setDefaultCommand(drive);
    tower.setDefaultCommand(moveTower);
    hopper.setDefaultCommand(moveHopper);
    shooter.setDefaultCommand(shootShooter);
    intake.setDefaultCommand(moveIntake);

    driveTrain2.resetEncoders();
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

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed // unsure if it's
            // actually that as there's a MaxVelocityConstraint
            .setKinematics(Constants.kDriveKinematics)
            // apply max velocity constraint
            // .addConstraint(new MaxVelocityConstraint(.2))
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    // s curve
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    // Pass config
    config);

    // should go 1 meter forward
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
    //     List.of(), new Pose2d(0, .5, Rotation2d.fromDegrees(90)), config);

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, driveTrain2::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics, driveTrain2::getWheelSpeeds, new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveTrain2::tankDriveVolts, driveTrain2);

    // Reset everything
    driveTrain2.resetOdometry(exampleTrajectory.getInitialPose());
    driveTrain2.resetEncoders();
    driveTrain2.zeroHeading();

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain2.tankDriveVolts(0, 0));
  }
}