package org.usfirst.frc.team4541.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team4541.robot.Constants;
import org.usfirst.frc.team4541.robot.Robot;
import org.usfirst.frc.team4541.robot.RobotMap;
import org.usfirst.frc.team4541.robot.commands.TankDriveWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;



/**
 * The DriveTrain subsystem incorporates the sensors and actuators attached to
 * the robots chassis. These include four drive motors, a left and right encoder
 * and a gyro.
 */
public class DriveTrain extends Subsystem {
	public WPI_TalonSRX leftMotor1  = new WPI_TalonSRX(RobotMap.leftDriveMotor1);
	public WPI_TalonSRX leftMotor2  = new WPI_TalonSRX(RobotMap.leftDriveMotor2);
	
	public WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RobotMap.rightDriveMotor1);
	public WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RobotMap.rightDriveMotor2);
	private DoubleSolenoid sol;
	
	private DifferentialDrive drive = new DifferentialDrive(leftMotor1, rightMotor1);

	public DriveTrain() {
		super();
		drive.setSafetyEnabled(false);
		this.configTalons();
		leftMotor2.follow(leftMotor1);
		rightMotor2.follow(rightMotor1);
		
		sol = new DoubleSolenoid(RobotMap.PCM, 0, 1);
	}

	/**
	 * Initialize normal driving as a default command
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new TankDriveWithJoystick());
	}
	
	/**
	 * Set the current state of the gear shifting solenoid
	 * 
	 * @param state: the desired state of the solenoid //TODO: is high gear true or false?
	 */
    public void setSolenoidOpen(boolean state){
    	if (state) {
    		sol.set(DoubleSolenoid.Value.kReverse);
    	} else {
    		sol.set(DoubleSolenoid.Value.kForward);
    	}
    }
    
    /**
     * Get the current state of the gear shifting solenoid
     * 
     * @return if the DriveTrain is in high or low gear //TODO: is high gear true or false?
     */
    public boolean isSolenoidOpen() {
    	return sol.get() == DoubleSolenoid.Value.kReverse;
    }
    
	/**
	 * Tank style driving for the DriveTrain.
	 * 
	 * @param forward: forward velocity in range [-1,1]
	 * @param rotate: rotational velocity in range [-1,1]
	 */
	public void drive(double forward, double rotate) {
		drive.curvatureDrive(forward, rotate, true);
	}
	
	/**
	 * Drive the robot based on the current joystick state
	 * 
	 * @param joy: current state of the robot
	 */
	public void drive(Joystick joy) {
		this.drive(this.addDeadZone(-joy.getRawAxis(1)), this.addDeadZone(joy.getRawAxis(4)));
	}
	
	/**
	 * Add dead zones to the value given by the joystick so that a slight bump does not
	 * move the robot.  Also square inputs for better velocity control
	 * 
	 * @param: input the raw value given by a joystick axis
	 */
	public double addDeadZone(double input) {
		if (Math.abs(input) <= .05)
			input = 0;
		else if (input < 0)
			input = -Math.pow(input, 2);
		else
			input = Math.pow(input, 2);
		return input;
	}
	
	/**
	 * configure the talons for use in the DriveTrain
	 */
	public void configTalons() {
		leftMotor1.setInverted(true);
		leftMotor2.setInverted(true);
		rightMotor1.setInverted(true);
		rightMotor2.setInverted(true);
		
		rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		rightMotor1.setSensorPhase(true); // keep encoder and motor in phase 
		
		// set the peak, nominal outputs
		rightMotor1.configNominalOutputForward(0, Constants.kTimeoutMs);
		rightMotor1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		rightMotor1.configPeakOutputForward(1, Constants.kTimeoutMs);
		rightMotor1.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		// set closed loop gains in slot0  //TODO: Tune
		rightMotor1.config_kF(Constants.kPIDLoopIdx, 0.7, Constants.kTimeoutMs);
		rightMotor1.config_kP(Constants.kPIDLoopIdx, 0.6, Constants.kTimeoutMs);
		rightMotor1.config_kI(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		rightMotor1.config_kD(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);

		leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		leftMotor1.setSensorPhase(false); // keep encoder and motor in phase
		
		// set the peak, nominal outputs
		leftMotor1.configNominalOutputForward(0, Constants.kTimeoutMs);
		leftMotor1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		leftMotor1.configPeakOutputForward(1, Constants.kTimeoutMs);
		leftMotor1.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		// set closed loop gains in slot0  //TODO: Tune
		leftMotor1.config_kF(Constants.kPIDLoopIdx, 0.7, Constants.kTimeoutMs);
		leftMotor1.config_kP(Constants.kPIDLoopIdx, 0.6, Constants.kTimeoutMs);
		leftMotor1.config_kI(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		leftMotor1.config_kD(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
	}
	
	/**
	 * Takes a velocity in inches per second and makes it the setpoint of the left motor
	 * 
	 * @param vel: the new velocity setpoint (in/sec)
	 */
	public void setLeftVel(double vel) {
		// convert velocity from inches/sec to pulses/100ms
		double targetVel = vel * Constants.kSensorUnitsPerInch / 10;
		if (targetVel > 0) {
			targetVel += 50; 
		} else if (targetVel < 0){
			targetVel -= 50;
		}
		
		// set the current setpoint of the talon's PIDF Controller to the desired velocity
		leftMotor1.set(ControlMode.Velocity, targetVel);
	}
	
	/**
	 * Takes a velocity in inches per second and makes it the setpoint of the right motor
	 * 
	 * @param vel: the new velocity setpoint (in/sec)
	 */
	public void setRightVel(double vel) {
		// convert velocity from inches/sec to pulses/100ms
		double targetVel = vel * Constants.kSensorUnitsPerInch / 10;
		if (targetVel > 0) {
			targetVel += 50; 
		} else if (targetVel < 0){
			targetVel -= 50;
		}
		
		// set the current setpoint of the talon's PIDF Controller to the desired velocity
		rightMotor1.set(ControlMode.Velocity, targetVel);
	}

	/**
	 * Get the velocity of the robot
	 * 
	 * @return velocity of the robot
	 */
	public double getVel() {
		return(this.leftMotor1.getSelectedSensorVelocity(0) + this.rightMotor1.getSelectedSensorVelocity(0) / 2.0);
	}
	
	/**
	 * Set the current position to zero on the wheel encoders
	 */
	public void zeroEncoders() {
		this.leftMotor1.setSelectedSensorPosition(0, 0, 0);
		this.rightMotor1.setSelectedSensorPosition(0, 0, 0);
	}
	
	/**
	 * Get the position of the left motors
	 * 
	 * @return: position in inches
	 */
	public double getLeftPos() {
		return this.leftMotor1.getSelectedSensorPosition(0) / Constants.kSensorUnitsPerInch;
	}
	
	/**
	 * Get the velocity of the left motors
	 * 
	 * @return velocity in inches/sec
	 */
	public double getLeftVel() {
		return this.leftMotor1.getSelectedSensorVelocity(0) / Constants.kSensorUnitsPerInch * 10;
	}
	
	/**
	 * Get the position of the right motors
	 * 
	 * @return: position in inches
	 */
	public double getRightPos() {
		return this.rightMotor1.getSelectedSensorPosition(0) / Constants.kSensorUnitsPerInch;
	}
	
	/**
	 * Get the velocity of the right motors
	 * 
	 * @return velocity in inches/sec
	 */
	public double getRightVel() {
		return this.rightMotor1.getSelectedSensorVelocity(0) / Constants.kSensorUnitsPerInch * 10;
	}
}
