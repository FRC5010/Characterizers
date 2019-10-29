/**
* This is a very simple robot program that can be used to send telemetry to
* the data_logger script to characterize your drivetrain. If you wish to use
* your actual robot code, you only need to implement the simple logic in the
* autonomousPeriodic function and change the NetworkTables update rate
*/

package dc;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  static private double WHEEL_DIAMETER = 0.5155;
  static private double ENCODER_PULSE_PER_REV = 120;
  boolean useDC = true;

  Joystick stick1;
  Joystick stick2;

  DifferentialDrive drive;
  PowerDistributionPanel pdp;
  public static AHRS gyro = new AHRS(Port.kUSB1);
  
  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

  double priorAutospeed = 0;
  Number[] numberArray = new Number[9];
  WPI_TalonSRX leftMotor1, rightMotor1;

  @Override
  public void robotInit() {
    if (!isReal()) SmartDashboard.putData(new SimEnabler());

    stick1 = new Joystick(0);
    stick2 = new Joystick(1);
    pdp = new PowerDistributionPanel();

    leftMotor1 = new WPI_TalonSRX(2);

    rightMotor1 = new WPI_TalonSRX(3);
    rightMotor1.setInverted(true);

    SpeedController[] leftMotors = new SpeedController[1];
    leftMotors[0] = new WPI_TalonSRX(5);

    SpeedController[] rightMotors = new SpeedController[1];
    rightMotors[0] = new WPI_TalonSRX(6);
    rightMotors[0].setInverted(true);

    //
    // Configure drivetrain movement
    //

    SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftMotor1, leftMotors);

    SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightMotor1, rightMotors);

    drive = new DifferentialDrive(leftGroup, rightGroup);
    drive.setDeadband(0);


    //
    // Configure encoder related functions -- getDistance and getrate should return
    // units and units/s
    //

    double encoderConstant = (1 / ENCODER_PULSE_PER_REV) * WHEEL_DIAMETER * Math.PI;

    Encoder leftEncoder = new Encoder(0, 1, false, EncodingType.k1X);
    leftEncoder.setDistancePerPulse(encoderConstant);
    leftEncoderPosition = leftEncoder::getDistance;
    leftEncoderRate = leftEncoder::getRate;

    Encoder rightEncoder = new Encoder(2, 3, true, EncodingType.k1X);
    rightEncoder.setReverseDirection(true);
    rightEncoder.setDistancePerPulse(encoderConstant);
    rightEncoderPosition = rightEncoder::getDistance;
    rightEncoderRate = rightEncoder::getRate;

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  @Override
  public void disabledInit() {
    System.out.println("Robot disabled");
    drive.tankDrive(0, 0);
  }

  @Override
    public void disabledPeriodic() {
  }
  boolean gyroReset = false;

  @Override
  public void robotPeriodic() {
    if (!gyroReset) {
      if (!gyro.isCalibrating()) {
        gyroReset = true;
        gyro.reset();
      }
    }
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    SmartDashboard.putNumber("Gyro Rotation", gyro.getRate());
    SmartDashboard.putBoolean("Gyro IsCal", gyro.isCalibrating());
    SmartDashboard.putBoolean("Gyro IsConn", gyro.isConnected());    // feedback for users, but not used by the control program
    SmartDashboard.putNumber("l_encoder_pos", leftEncoderPosition.get());
    SmartDashboard.putNumber("l_encoder_rate", leftEncoderRate.get());
    SmartDashboard.putNumber("r_encoder_pos", rightEncoderPosition.get());
    SmartDashboard.putNumber("r_encoder_rate", rightEncoderRate.get());
  }

  @Override
  public void teleopInit() {
    System.out.println("Robot in operator control mode");
  }

  int count = 0;
  double current, accCur = 0;
  double power, accPow = 0;

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(-stick1.getY(), stick2.getX());
    if (stick1.getY() != 0) {
      count++;
      accCur += pdp.getTotalCurrent();
      current = accCur / count;
      SmartDashboard.putNumber("PDP current", current);
      accPow += pdp.getTotalPower();
      power = accPow / count;
      SmartDashboard.putNumber("PDP power", power);
      }
    }

  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");
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

    double leftMotorVolts = leftMotor1.getMotorOutputVoltage();
		double rightMotorVolts = rightMotor1.getMotorOutputVoltage();

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    double angle = gyro.getAngle();
    double driftCorrection = angle * .01;

    // command motors to do things
    drive.tankDrive(autospeed + (useDC ? driftCorrection : 0.0), autospeed - (useDC ? driftCorrection : 0.0), false);

    // send telemetry data array back to NT
    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;

    telemetryEntry.setNumberArray(numberArray);
  }
}