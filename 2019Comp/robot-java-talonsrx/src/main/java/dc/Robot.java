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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	static private double WHEEL_DIAMETER = .5;
	static private double ENCODER_PULSE_PER_REV = 120;
	static private int PIDIDX = 0;

	Joystick stick;
	DifferentialDrive drive;

	WPI_TalonSRX leftFrontMotor;
	WPI_TalonSRX rightFrontMotor;
	Encoder leftEncoder;
	Encoder rightEncoder;
	
	Supplier<Double> leftEncoderPosition;
	Supplier<Double> leftEncoderRate;
	Supplier<Double> rightEncoderPosition;
	Supplier<Double> rightEncoderRate;

	NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
	NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

	double priorAutospeed = 0;
	Number[] numberArray = new Number[9];

	@Override
	public void robotInit() {

		stick = new Joystick(0);

		leftFrontMotor = new WPI_TalonSRX(2);
		leftFrontMotor.setInverted(false);
		leftFrontMotor.setSensorPhase(false);
		leftFrontMotor.setNeutralMode(NeutralMode.Brake);

		rightFrontMotor = new WPI_TalonSRX(3);
		rightFrontMotor.setInverted(false);
		rightFrontMotor.setSensorPhase(true);
		rightFrontMotor.setNeutralMode(NeutralMode.Brake);

		// left rear follows front
		WPI_TalonSRX leftRearMotor = new WPI_TalonSRX(5);
		leftRearMotor.setInverted(false);
		leftRearMotor.setSensorPhase(false);
		leftRearMotor.follow(leftFrontMotor);
		leftRearMotor.setNeutralMode(NeutralMode.Brake);

		// right rear follows front
		WPI_TalonSRX rightRearMotor = new WPI_TalonSRX(6);
		rightRearMotor.setInverted(false);
		rightRearMotor.setSensorPhase(true);
		rightRearMotor.follow(rightRearMotor);
		rightRearMotor.setNeutralMode(NeutralMode.Brake);

		rightEncoder = new Encoder(0, 1, true,EncodingType.k1X);
		leftEncoder = new Encoder(2, 3, false, EncodingType.k1X);
			//
		// Configure drivetrain movement
		//

		SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftFrontMotor, leftRearMotor);
		SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightFrontMotor, rightRearMotor);

		drive = new DifferentialDrive(leftGroup, rightGroup);
		drive.setDeadband(.1);


		//
		// Configure encoder related functions -- getDistance and getrate should return
		// ft and ft/s
		//

		double encoderConstant = (1 / ENCODER_PULSE_PER_REV) * WHEEL_DIAMETER * Math.PI;
		leftEncoder.setDistancePerPulse(encoderConstant);
		rightEncoder.setDistancePerPulse(encoderConstant);

		leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDIDX, 10);
		// leftEncoderPosition = () -> leftFrontMotor.getSelectedSensorPosition(PIDIDX) * encoderConstant;
		// leftEncoderRate = () -> leftFrontMotor.getSelectedSensorVelocity(PIDIDX) * encoderConstant * 10;
		leftEncoderPosition = () -> leftEncoder.getDistance();
		leftEncoderRate = () -> leftEncoder.getRate();

		rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDIDX, 10);
		// rightEncoderPosition = () -> rightFrontMotor.getSelectedSensorPosition(PIDIDX) * encoderConstant;
		// rightEncoderRate = () -> rightFrontMotor.getSelectedSensorVelocity(PIDIDX) * encoderConstant * 10;
		rightEncoderPosition = () -> rightEncoder.getDistance();
		rightEncoderRate = () -> rightEncoder.getRate();

		// Reset encoders
		// leftFrontMotor.setSelectedSensorPosition(0);
		// rightFrontMotor.setSelectedSensorPosition(0);
		leftEncoder.reset();
		rightEncoder.reset();

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
		leftEncoder.reset();
		rightEncoder.reset();
		System.out.println("Robot in operator control mode");
	}

	@Override
	public void teleopPeriodic() {
		drive.arcadeDrive(-stick.getRawAxis(1), stick.getRawAxis(4));
	}

	@Override
	public void autonomousInit() {
		leftEncoder.reset();
		rightEncoder.reset();
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

		double leftMotorVolts = leftFrontMotor.getMotorOutputVoltage();
		double rightMotorVolts = rightFrontMotor.getMotorOutputVoltage();

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
