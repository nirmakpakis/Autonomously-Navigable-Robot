package navigation;

import ca.mcgill.ecse211.project.DesignProjectMain;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import odometer.Odometer;

public class Navigate {

	//Initialize odometer
	private static Odometer odometer;
	
	//Left light sensor
	private static SampleProvider sensorVal;
	static float[] sensorValData;
	
	//right light sensor
	private static SampleProvider sensorVal2;
	static float[] sensorValData2;
	
	//motors
	static EV3LargeRegulatedMotor leftMotor;
	static EV3LargeRegulatedMotor rightMotor;
	
	//Initiate variables and constants
	private static final double WHEEL_RAD = DesignProjectMain.WHEEL_RAD;
	private static final double TRACK = DesignProjectMain.TRACK; 	
	private static final int FORWARD_SPEED = 150;	
	private static final double sensorOffset = -5.5;
	private static boolean keepGoing;
	
	//variables and constants used for line detection
	private static double lineDetection = DesignProjectMain.lineDetection;
	private static int firstTacho = 0;
	private static int secondTacho = 0;
	private static int tachoDiff = 0;
	private static double distanceDiff = 0;
	private static int leftOrRightDetectedFirst = 0;
	private static double deltaT = 0;
	
	//variables used for the travelTo method
	static double xDistance = 0;
	static double yDistance = 0;
	double distanceToTravel = 0;
	static double currentXposition = 0;
	static double currentYposition = 0;
	
	//variables and constants used for travelling
	static double currentCoordinates [] = new double[3];
	static double angleBeta;
	static double currentAngle;
	static double finalAngle;
	static double initialAngle = 0;
	private static double leftDetection;
	private static double rightDetection;
	
	public Navigate(Odometer odo,
						SampleProvider sensorVal, float[] sensorValData,
							SampleProvider sensorVal2, float[] sensorValData2) {
		odometer = odo;
		leftMotor = odometer.leftMotor;
		rightMotor = odometer.rightMotor;
		
		Navigate.sensorVal = sensorVal;
		Navigate.sensorValData = sensorValData;
		
		Navigate.sensorVal2 = sensorVal2;
		Navigate.sensorValData2 = sensorValData2;

	}
	
	public static void moveWithCorrection(int direction) {
		
		setSpeed(100);
		motorsForward();
		
		boolean rightDetected = false, leftDetected = false;
		while (true) {
			
			sensorVal.fetchSample(sensorValData, 0);
			leftDetection = sensorValData[0];
			sensorVal2.fetchSample(sensorValData2, 0);
			rightDetection = sensorValData2[0];
			
			if(leftDetection<lineDetection && !leftDetected && !rightDetected) {
				firstTacho = odometer.leftMotor.getTachoCount();
				Sound.beep();
				leftDetected = true;
				leftOrRightDetectedFirst = -1;
			}
			
			sensorVal.fetchSample(sensorValData, 0);
			leftDetection = sensorValData[0];
			sensorVal2.fetchSample(sensorValData2, 0);
			rightDetection = sensorValData2[0];
			
			if(rightDetection<lineDetection && !leftDetected && !rightDetected) {
				firstTacho = odometer.leftMotor.getTachoCount();
				Sound.beep();
				rightDetected = true;
				leftOrRightDetectedFirst = 1;
			}
			
			sensorVal.fetchSample(sensorValData, 0);
			leftDetection = sensorValData[0];
			sensorVal2.fetchSample(sensorValData2, 0);
			rightDetection = sensorValData2[0];
			
			if(leftDetection<lineDetection && !leftDetected && rightDetected) {
				secondTacho = odometer.leftMotor.getTachoCount();
				Sound.beep();
				leftDetected = true;
			}
			
			sensorVal.fetchSample(sensorValData, 0);
			leftDetection = sensorValData[0];
			sensorVal2.fetchSample(sensorValData2, 0);
			rightDetection = sensorValData2[0];
			
			if(rightDetection<lineDetection && leftDetected && !rightDetected) {
				secondTacho = odometer.leftMotor.getTachoCount();
				Sound.beep();
				rightDetected = true;
			}
			
			if(rightDetected && leftDetected) {
				stopMotors();
				Sound.beepSequence();
				break;
			}
			
		}
		
		tachoDiff = secondTacho - firstTacho;
		distanceDiff = (Math.PI*WHEEL_RAD*(tachoDiff)/180);
		deltaT = leftOrRightDetectedFirst*distanceDiff /TRACK;
		
		odometer.leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, deltaT*180/Math.PI), true);
		odometer.rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, deltaT*180/Math.PI), false);
		System.out.println("DONE");
		
	}
	
	/**
	 * This method stops the motions of the wheels' motors
	 */
	private static void stopMotors() {
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
	}
	
	/**
	 * This method changes the rotation speed of the wheels' motors
	 * 
	 * @param wheelSpeed: Speed of rotation of the wheels
	 */
	private static void setSpeed(int wheelSpeed) {
		
		rightMotor.setSpeed(wheelSpeed+2);
		leftMotor.setSpeed(wheelSpeed);
		
	}
	
	/**
	 * This method starts the forward motion of the wheels' motors
	 */
	private static void motorsForward() {
		
		leftMotor.forward();
		rightMotor.forward();
		
	}
	
	/**
	 * This method converts a distance to the total rotation that each
	 * wheel needs to perform
	 * 
	 * @param radius: wheel radius
	 * @param distance: distance to travel
	 * @return: integer
	 */
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * This method convert an angle to the total rotation that
	 * each wheel needs to perform
	 * 
	 * @param radius: wheel radius
	 * @param width: wheel base of the robot
	 * @param angle: the next robot orientation
	 * @return: convertDistance()
	 */
	private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}
