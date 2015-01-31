package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DigitalInput;

/**
* Makes mecanum chassis robots track a "line" based on two areas of different
* reflectivity using Allen-Bradly 42EF-DIMNAK-A2 DIFFUSE Photswitches, but it
* could be repurposed for any digital switch
*/
public class LineTrack
{
    static double m_xPower;
    static double m_yPower;
    static double m_pConstant;
    static double m_iConstant;
	static int m_stateID;

    /**
    * Inputs the values of the PI constants and the movement power constants
    * 
    * @param xPower The power for the chassis's movement in the x direction
    * @param yPower The power for the chassis's movement in the y direction
    * @param pConstant The P constant for the PI loop
    * @param iConstant The I constant for the PI loop
    */
    static void initializeConstants(double xPower, double yPower, 
        double pConstant, double iConstant)
    {
        m_xPower = xPower;
        m_yPower = yPower;
        m_pConstant = pConstant;
        m_iConstant = iConstant;
		m_stateID = 0;
    }

    /**
    * Controls the chassis to track the imaginary line between the areas of 
    * different reflectivity
    * 
    * @param frontPhotoswitch The input from the photoswitch closest to the back
    * end of the chassis
    * @param rearPhotoswitch The input from the photoswitch furthest from the
    * back of the chassis
    * @param chassis A chassis object for controlling the drive motors
    * @param gyroValue The current gyro sensor value
    */
	static void lineTrack(DigitalInput frontPhotoswitch, 
			DigitalInput rearPhotoswitch, Chassis chassis, int gyroValue)
	{
        /* If both sensors are over the reflective plaform, the robot moves
         forward */
		if(frontPhotoswitch.get() && rearPhotoswitch.get())
		{
			if(m_stateID != 1)
			{
				GyroDrive.reinit();
			}
            double adjustedRotationValue
				= GyroDrive.getAdjustedRotationValue(0, m_yPower, 0, 
						m_pConstant, m_iConstant, gyroValue);
			chassis.setJoystickData(0, m_yPower, adjustedRotationValue);
			m_stateID = 1;
			System.out.println(m_stateID);
		}
        // If both sensors are over the carpet, the robot moves backward
		else if(!frontPhotoswitch.get() && !rearPhotoswitch.get())
		{
			if(m_stateID != 2)
			{
				GyroDrive.reinit();
			}
            double adjustedRotationValue
				= GyroDrive.getAdjustedRotationValue(0, -m_yPower, 0, 
						m_pConstant, m_iConstant, gyroValue);
			chassis.setJoystickData(0, -m_yPower, adjustedRotationValue);
			m_stateID = 2;
			System.out.println(m_stateID);
		}
        // The robot moves sideways
		else
		{
			if(m_stateID != 3)
			{
				GyroDrive.reinit();
			}
            double adjustedRotationValue
				= GyroDrive.getAdjustedRotationValue(m_xPower, 
                0, 0, m_pConstant, m_iConstant, 
                gyroValue);
			chassis.setJoystickData(m_xPower, 0, 
                adjustedRotationValue);
			m_stateID = 3;
			System.out.println(m_stateID);
		}
	}
}

