/**
 * This is a very simple robot program that can be used to send telemetry to
 * the data_logger script to characterize your drivetrain. If you wish to use
 * your actual robot code, you only need to implement the simple logic in the
 * autonomousPeriodic function and change the NetworkTables update rate
 *
 * This program assumes that you are using TalonSRX motor controllers and that
 * the drivetrain encoders are attached to the TalonSRX
 */

package dc;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  static private double WHEEL_DIAMETER = 0.5;
  static private double GEARING = 1;
  static private int PIDIDX = 0;

  Joystick stick1;
  Joystick stick2;
  DifferentialDrive drive;

  CANSparkMax leftMaster;
  CANSparkMax rightMaster;
  PowerDistributionPanel pdp;

  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;

  NetworkTableEntry autoSpeedEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

  double priorAutospeed = 0;
  Number[] numberArray = new Number[9];

  @Override
  public void robotInit() {
    if (!isReal()) SmartDashboard.putData(new SimEnabler());

    stick1 = new Joystick(0);
    stick2 = new Joystick(1);

    leftMaster = new CANSparkMax(1, MotorType.kBrushless);
    leftMaster.setIdleMode(IdleMode.kBrake);

    rightMaster = new CANSparkMax(3, MotorType.kBrushless);
    rightMaster.setIdleMode(IdleMode.kBrake);

    CANSparkMax leftSlave0 = new CANSparkMax(2, MotorType.kBrushless);
    leftSlave0.follow(leftMaster);
    leftSlave0.setIdleMode(IdleMode.kBrake);

    CANSparkMax rightSlave0 = new CANSparkMax(4, MotorType.kBrushless);
    rightSlave0.follow(rightMaster);
    rightSlave0.setIdleMode(IdleMode.kBrake);

    //
    // Configure drivetrain movement
    //

    drive = new DifferentialDrive(leftMaster, rightMaster);

    drive.setDeadband(0);

    //
    // Configure encoder related functions -- getDistance and getrate should
    // return units and units/s
    //

    double encoderConstant =
        (1 / GEARING) * WHEEL_DIAMETER * Math.PI;

    leftEncoderPosition = ()
        -> leftMaster.getEncoder().getPosition() * encoderConstant;
    leftEncoderRate = ()
        -> leftMaster.getEncoder().getVelocity() * encoderConstant / 60.;

    rightEncoderPosition = ()
        -> rightMaster.getEncoder().getPosition() * encoderConstant;
    rightEncoderRate = ()
        -> rightMaster.getEncoder().getVelocity() * encoderConstant / 60.;

    // Reset encoders
    leftMaster.getEncoder().setPosition(0);
    rightMaster.getEncoder().setPosition(0);

    pdp = new PowerDistributionPanel();

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
  public void disabledPeriodic() {}

  int count = 0;
  double current, accCur = 0;
  double power, accPow = 0;

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
    
    double leftMotorVolts = leftMaster.getBusVoltage() * leftMaster.getAppliedOutput();
    double rightMotorVolts = rightMaster.getBusVoltage() * rightMaster.getAppliedOutput();

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    drive.tankDrive(autospeed, autospeed, false);

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
