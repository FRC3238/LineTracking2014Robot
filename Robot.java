package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.util.MathUtils;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.Vector;

public class Robot extends IterativeRobot
{
	Joystick joystick;
	Chassis chassis;
	AnalogChannel ultrasonicSensor, gyroSensor, infaredSensor;
	DigitalInput frontPhotoswitch, rearPhotoswitch;
	DriverStationLCD driverStationLCD;
	DriverStation driverStation;

	double m_autoPConstant;
	double m_autoIConstant;
	double m_teleopPConstant;
	double m_teleopIConstant;
	double m_spinThreshold;
	double m_xPower;
	double m_yPower;

	Robot()
	{
		// Talon Ports
		final int LEFTFRONTTALONPORT = 6;
		final int LEFTREARTALONPORT = 8;
		final int RIGHTFRONTTALONPORT = 7;
		final int RIGHTREARTALONPORT = 3;
		// Digital Inputs
		final int FRONTPHOTOSWITCHPORT = 13;
		final int REARPHOTOSWITCHPORT = 14;
		// Analog Inputs
		final int ULTRASONICSENSORPORT = 1;
		final int GYROSENSORPORT = 6;
		final int INFAREDSENSORPORT = 5;
		// Driver Station Inputs
		final int JOYSTICKPORT = 1;

		chassis = new Chassis(LEFTFRONTTALONPORT, LEFTREARTALONPORT, 
				RIGHTFRONTTALONPORT, RIGHTREARTALONPORT);
		joystick = new Joystick(JOYSTICKPORT);
		ultrasonicSensor = new AnalogChannel(ULTRASONICSENSORPORT);
		gyroSensor = new AnalogChannel(GYROSENSORPORT);
		frontPhotoswitch = new DigitalInput(FRONTPHOTOSWITCHPORT);
		rearPhotoswitch = new DigitalInput(REARPHOTOSWITCHPORT);
		infaredSensor = new AnalogChannel(INFAREDSENSORPORT);
		driverStationLCD = DriverStationLCD.getInstance();
		driverStation = DriverStation.getInstance();
	}

	public void robotInit()
	{
	}

	public void disabledInit()
	{
	}

	public void autonomousInit()
	{
		Vector settingsFile = FileReader.getFileContents("settings.txt");
		m_autoPConstant
				= Double.parseDouble((String) settingsFile.elementAt(0));
		m_autoIConstant
				= Double.parseDouble((String) settingsFile.elementAt(1));
		m_spinThreshold
				= Double.parseDouble((String) settingsFile.elementAt(4));
		m_xPower
				= Double.parseDouble((String) settingsFile.elementAt(5));
		m_yPower
				= Double.parseDouble((String) settingsFile.elementAt(6));
		GyroDrive.reinit();
		LineTrack.initializeConstants(m_xPower, m_yPower, m_autoPConstant,
				m_autoIConstant);
	}

	public void teleopInit()
	{
		Vector settingsFile = FileReader.getFileContents("settings.txt");
		m_teleopPConstant
				= Double.parseDouble((String) settingsFile.elementAt(2));
		m_teleopIConstant
				= Double.parseDouble((String) settingsFile.elementAt(3));
		m_spinThreshold
				= Double.parseDouble((String) settingsFile.elementAt(4));
		GyroDrive.reinit();
	}

	public void testInit()
	{
	}

	public void disabledPeriodic()
	{
		System.out.println(0.261091633
				* MathUtils.pow(infaredSensor.getVoltage(), -1.190686871));
	}

	public void autonomousPeriodic()
	{
		int gyroValue = gyroSensor.getValue() - 476;
		LineTrack.lineTrack(frontPhotoswitch, rearPhotoswitch, chassis,
				m_spinThreshold, gyroValue);
		chassis.idle();
	}

	public void teleopPeriodic()
	{
		driverStationLCD.clear();
		int gyroValue = gyroSensor.getValue() - 476;
		driverStationLCD.println(DriverStationLCD.Line.kUser1, 1,
				Integer.toString(gyroValue));
		driverStationLCD.updateLCD();
		double x = joystick.getRawAxis(1);
		double y = joystick.getRawAxis(2);
		double twist = joystick.getRawAxis(3);
		if(driverStation.getDigitalIn(2))
		{
			double adjustedRotationValue
					= GyroDrive.getAdjustedRotationValue(x, y, twist,
							m_teleopPConstant, m_teleopIConstant,
							m_spinThreshold, gyroValue);
			chassis.setJoystickData(x, y, adjustedRotationValue);
		}
		else
		{
			chassis.setJoystickData(x, y, twist);
		}
		chassis.idle();
	}

	public void testPeriodic()
	{
		System.out.println("Front Photoswitch: " + frontPhotoswitch.get());
		System.out.println("Rear Photoswitch: " + rearPhotoswitch.get());
		chassis.setJoystickData(0, 0, 0);
		chassis.idle();
	}
}
