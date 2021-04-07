/**
* This is a very simple robot program that can be used to send telemetry to
* the data_logger script to characterize your drivetrain. If you wish to use
* your actual robot code, you only need to implement the simple logic in the
* autonomousPeriodic function and change the NetworkTables update rate
*/

package dc;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// WPI_Talon* imports are needed in case a user has a Pigeon on a Talon
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

import com.playingwithfusion.CANVenom;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.EncoderType;
import com.revrobotics.AlternateEncoderType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

class Encoder {
//  private final double wheelCircumference = 0.4699; // meters
//  private final double gearboxRatio = 10.91;

  private final WPI_TalonSRX talon;
  private boolean reverse = false;
  private double dpp;

  public Encoder(WPI_TalonSRX t) {
    talon = t;
  }

  public void setReverseDirection(boolean r) {
    reverse = r;
  }

  public void setDistancePerPulse(double d) {
    dpp = d;
  }

  public double getRawPosition() {
    return talon.getSelectedSensorPosition() * (reverse ? -1 : 1);
  }

  public double getRawVelocity() {
    return talon.getSelectedSensorVelocity() * (reverse ? -1 : 1);
  }

  public double getPosition() {
    return getRawPosition()  * dpp;
  }

  public double getVelocity() {
    return (getRawVelocity() * dpp) * 10.0;
  }
}

public class Robot extends TimedRobot {

  static private double ENCODER_EDGES_PER_REV = 4096;
  static private int PIDIDX = 0;
  static private int ENCODER_EPR = 4096;
  static private double GEARING = 10.91;
  
//  private double encoderConstant = (1 / (Math.PI * 0.1524)) * (1 / (ENCODER_EDGES_PER_REV));
  // private double encoderConstant = (Math.PI * 0.1524) / 4096.0;
  // private double encoderConstant = (1.0 / 4096.0) * 0.1524 * Math.PI;
  // private double encoderConstant = 4096.0/GEARING;
  private double encoderConstant = 0.47877871986 / 4096;
  // private double encoderConstant = 1.0 / ((1.0 / 0.47878) * (1024.0 * 4.0 * 1.0));


  XboxController xbox;
  DifferentialDrive drive;


  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  String data = "";
  
  int counter = 0;
  double startTime = 0;
  double priorAutospeed = 0;

  double[] numberArray = new double[10];
  ArrayList<Double> entries = new ArrayList<Double>();
  public Robot() {
    super(.005);
    LiveWindow.disableAllTelemetry();
  }

  public enum Sides {
    LEFT,
    RIGHT,
    FOLLOWER
  }

  // methods to create and setup motors (reduce redundancy)
  public WPI_TalonSRX setupWPI_TalonSRX(int port, Sides side, boolean inverted) {
    // create new motor and set neutral modes (if needed)
    WPI_TalonSRX motor = new WPI_TalonSRX(port);
    // setup talon
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(inverted);
    motor.setSelectedSensorPosition(0);
    
    // setup encoder if motor isn't a follower
    if (side != Sides.FOLLOWER) {
    
      Encoder encoder;



    switch (side) {
      // setup encoder and data collecting methods

      case RIGHT:
        // set right side methods = encoder methods

        encoder = new Encoder(motor);
        encoder.setReverseDirection(false);

        encoder.setDistancePerPulse((double) encoderConstant);
        rightEncoderPosition = encoder::getPosition;
        rightEncoderRate = encoder::getVelocity;

        break;
      case LEFT:
        encoder = new Encoder(motor);
        encoder.setReverseDirection(true);
        encoder.setDistancePerPulse(encoderConstant);
        leftEncoderPosition = encoder::getPosition;
        leftEncoderRate = encoder::getVelocity;


        break;
      default:
        // probably do nothing
        break;

      }
    
    }
    

    return motor;

  }

  @Override
  public void robotInit() {
    if (!isReal()) SmartDashboard.putData(new SimEnabler());

    xbox = new XboxController(0);
    
    // create left motor
    WPI_TalonSRX leftMotor = setupWPI_TalonSRX(0, Sides.LEFT, false);

    WPI_TalonSRX leftFollowerID1 = setupWPI_TalonSRX(1, Sides.FOLLOWER, false);
    leftFollowerID1.follow(leftMotor);

    WPI_TalonSRX rightMotor = setupWPI_TalonSRX(3, Sides.RIGHT, false);
    WPI_TalonSRX rightFollowerID3 = setupWPI_TalonSRX(2, Sides.FOLLOWER, false);
    rightFollowerID3.follow(rightMotor);
    drive = new DifferentialDrive(leftMotor, rightMotor);
    drive.setDeadband(0);

    //
    // Configure gyro
    //

    // Note that the angle from the NavX and all implementors of WPILib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    Gyro gyro = new ADXRS450_Gyro();
    gyroAngleRadians = () -> -1 * Math.toRadians(gyro.getAngle());

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  @Override
  public void disabledInit() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    System.out.println("Robot disabled");
    drive.tankDrive(0, 0);
    // data processing step
    data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    entries.clear();
    System.out.println("Robot disabled");
    System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
    data = "";
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    // feedback for users, but not used by the control program
    SmartDashboard.putNumber("l_encoder_pos", leftEncoderPosition.get());
    SmartDashboard.putNumber("l_encoder_rate", leftEncoderRate.get());
    SmartDashboard.putNumber("r_encoder_pos", rightEncoderPosition.get());
    SmartDashboard.putNumber("r_encoder_rate", rightEncoderRate.get());
  }

  @Override
  public void teleopInit() {
    System.out.println("Robot in operator control mode");
  }

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(-xbox.getRawAxis(1), xbox.getRawAxis(4));
  }

  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");
    startTime = Timer.getFPGATimestamp();
    counter = 0;
  }

  /**
  * If you wish to just use your own robot program to use with the data logging
  * program, you only need to copy/paste the logic below into your code and
  * ensure it gets called periodically in autonomous mode
  * 
  * Additionally, you need to set NetworkTables update rate to 10ms using the
  * setUpdateRate call.
  */
  @Override
  public void autonomousPeriodic() {

    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double leftPosition = leftEncoderPosition.get();
    double leftRate = leftEncoderRate.get();

    double rightPosition = rightEncoderPosition.get();
    double rightRate = rightEncoderRate.get();

    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    drive.tankDrive(
      (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed,
      false
    );

    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = gyroAngleRadians.get();

    // Add data to a string that is uploaded to NT
    for (double num : numberArray) {
      entries.add(num);
    }
    counter++;
  }
}
