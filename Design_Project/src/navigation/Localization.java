package navigation;

import ca.mcgill.ecse211.project.DesignProjectMain;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import odometer.Odometer;

/**
 * "Localization" class is responsible of correctiong the position 
 *and orientation of the robot to be (0,0) and 0-degree starting from the y-axis
 *
 * @author Maxime Cardinal , Irmak Pakis
 * @dateCreated Monday 29th October 2018
 * @version 1.2
 *
 */
public class Localization {
	
	//Ultrasonic sensor and odometer
	private static SampleProvider us;
	private static float[] usData;
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
	private static final int ROTATE_SPEED = 285; 	
	private static final int FORWARD_SPEED = 150;	
	private static final int CORRECTION_SPEED = 70;	
	private static final int fullRotation = 360;
	private static final int halfRotation = 180;
	private static final int DISTANCE_RANGE = 40;	
	private static final double sensorOffset = -5.7;
	private static boolean keepGoing;
	
	//variables and constants used for line detection
	private static double boardLight = 0.0;
	private static double lineDetection = DesignProjectMain.lineDetection;
	private static final double lineDetectionOffset = 0.30;
	
	//variables and constants used for travelling
	static double currentCoordinates [] = new double[3];
	static double angleBeta;
	static double currentAngle;
	static double finalAngle;
	static double initialAngle = 0;
	
	//variables and constants used for usLocalization
	private static double firstFallingEdge;
	private static double secondFallingEdge;
	private static double FallingEdge;
	private static double leftDetection;
	private static double rightDetection;
	private static double previousReading [] = new double[2];
	
	/**
	 * Constructor of the class
	 * 
	 * @param odometer: Odometer
	 * @param us: SampleProvider
	 * @param usData: Ultrasonic sensor data
	 * @param sensorVal: Sample Provider
	 * @param sensorValData: Left light sensor data
	 * @param sensorVal2: Sample Provider
	 * @param sensorValData2: Right light sensor data
	 */
	public Localization(Odometer odo, SampleProvider us, float[] usData,
						SampleProvider sensorVal, float[] sensorValData,
							SampleProvider sensorVal2, float[] sensorValData2) {
		
		odometer = odo;
		leftMotor = odometer.leftMotor;
		rightMotor = odometer.rightMotor;
		
		Localization.us = us;
	    Localization.usData = usData;
	    
	    Localization.sensorVal = sensorVal;
	    Localization.sensorValData = sensorValData;
	    
	    Localization.sensorVal2 = sensorVal2;
	    Localization.sensorValData2 = sensorValData2;
	    
	}
	
	/**
	 * This method detects the board light that will be used for 
	 * the differentiator in line detection
	 */
	public static void getBoardLight() {
		
		for(int i = 0; i<5; i++) {
			sensorVal.fetchSample(sensorValData, 0);
			boardLight += sensorValData[0];
		}
		boardLight = boardLight/5;
		lineDetection = boardLight - lineDetectionOffset;
		DesignProjectMain.lineDetection = lineDetection;
		
	}
	
	/**
	 * This method corrects the initial angle of the robot to 0-degree
	 * It uses the falling edge algorithm 
	 */
	public static void usLocalization() {
		
		setSpeed(ROTATE_SPEED);
		
		//if the robot is facing a wall, turn around befor computing
		int distance;
		us.fetchSample(usData, 0); 				// acquire data
        distance = (int) (usData[0] * 100.0); 	// extract from buffer, cast to int
        
        if(distance<DISTANCE_RANGE) {
        	
    		Sound.beepSequence();
    		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, halfRotation), true);
    		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, halfRotation), false);
    	    
        }
		
        keepGoing = true;
        reinitializePreviousReadings();
        
        //Turn right until a falling edge is detected
        leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, fullRotation), true);
        rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, fullRotation), true);
	    
	    while(keepGoing){
	    	
	    	us.fetchSample(usData, 0); 							// acquire data
	        distance = (int) (usData[0] * 100.0); 				// extract from buffer, cast to int   
	        firstFallingEdge = fallingEdgeDetection(distance);	// get the angle detected by us sensor
	        
	    }
	    
	    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, -90), true);
	    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, -90), false);
	    
	    keepGoing = true;
	    reinitializePreviousReadings();
		
		//Turn left until a falling edge is detected
	    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, -360), true);
	    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, -360), true);
	    
	    while(keepGoing){
	    	
	    	us.fetchSample(usData, 0); 							// acquire data
	        distance = (int) (usData[0] * 100.0); 				// extract from buffer, cast to int   
	        secondFallingEdge = fallingEdgeDetection(distance);	// get the angle detected by us sensor
	        
	    }
	    
	    //Angle algorithm
	    if(firstFallingEdge<secondFallingEdge) {
	    	angleBeta =45- (firstFallingEdge+secondFallingEdge)/2;
	    }
	    else if(firstFallingEdge>secondFallingEdge) {
	    	angleBeta =225-(firstFallingEdge+secondFallingEdge)/2;
	    }
	    currentAngle = currentCoordinates[2];
	    finalAngle = 180 - (currentAngle + angleBeta);
		turnTo(finalAngle);
		
	}
	
	/**
	 * This method corrects the initial position of the robot
	 * to the point (0,0) in the grid
	 */
	public static void lsLocalization() {
		
		//correct y-axis
		setSpeed(CORRECTION_SPEED);
		angleCorrection();
		
		setSpeed(FORWARD_SPEED);
		sensorOffsetCorrection();
	    
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), false);
	    
		angleCorrection();
		
		sleep(100);
		sensorOffsetCorrection();
		
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), false);
	    	    
	}
	
	/**
	 * This method is responsible for adjusting the angle of the robot
	 * toward the next position it needs to travel
	 * 
	 * @param theta: Angle, in radian, to turn the robot to
	 */
	static void turnTo(double theta) {
		
		setSpeed(ROTATE_SPEED);

		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
	}
	
	/**
	 * This method converts a distance to the total rotation that each
	 * wheel needs to perform
	 * 
	 * @param radius: Wheel radius
	 * @param distance: Distance to travel
	 * @return: integer
	 */
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * This method convert an angle to the total rotation that
	 * each wheel needs to perform
	 * 
	 * @param radius: Wheel radius
	 * @param width: Wheel base of the robot
	 * @param angle: Whe next robot orientation
	 * @return: convertDistance()
	 */
	private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/**
	 * This method changes the rotation speed of the wheels' motors
	 * 
	 * @param wheelSpeed: Speed of rotation of the wheels
	 */
	private static void setSpeed(int wheelSpeed) {
		stopMotors();
		leftMotor.setSpeed(wheelSpeed);
		rightMotor.setSpeed(wheelSpeed);
		sleep(100);
		
	}
	
	/**
	 * This method reinitializes the previous readings of the
	 * ultrasonic sensor
	 */
	private static void reinitializePreviousReadings() {
		
		previousReading[0] = 0;
		previousReading[1] = 0;
		
	}
	
	/**
	 * This method updates the previous readings of the
	 * ultrasonic sensor
	 * 
	 * @param distance: Distance detected by ultrasonic sensor
	 */
	private static void updatePreviousReadings(int distance) {
		
		previousReading[1] = previousReading[0];
		previousReading[0] = distance;
		
	}
	
	/**
	 * This method starts the forward motion of the wheels' motors
	 */
	private static void motorsForward() {
		
		leftMotor.forward();
		rightMotor.forward();
		
	}
	
	/**
	 * This method stops the motions of the wheels' motors
	 */
	private static void stopMotors() {
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
	}
	
	/**
	 * This method corrects the sensor offset to the wheelcenter
	 * in the y-axis
	 */
	private static void sensorOffsetCorrection() {
		
		leftMotor.rotate(convertDistance(WHEEL_RAD, sensorOffset), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, sensorOffset), false);
		
	}
	
	/**
	 * This method corrects the robot angle after the localization
	 */
private static void angleCorrection() {
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);
		
		setSpeed(70);
		
		sleep(100);
		
		double angleProtection = odometer.getXYT()[2];
		boolean rightDetected = false, leftDetected = false;

		rightMotor.forward();
		leftMotor.forward();

		while (true) {
			sensorVal.fetchSample(sensorValData, 0);
			sensorVal2.fetchSample(sensorValData2, 0);
			double rightS = sensorValData2[0];
			double leftS = sensorValData[0];
			if (rightS < 0.25 && !rightDetected) {
				rightMotor.stop(!leftDetected);
				rightDetected = true;
	//			Sound.beep();
			}
			if (leftS < 0.25 && !leftDetected) {
				leftMotor.stop(!rightDetected);
				leftDetected = true;
//				Sound.beep();
			}
			if (leftDetected && rightDetected)
				break;
			
//   		if(lineMissedVerification(leftDetected, rightDetected, odometer.getXYT()[2], angleProtection)) {
//   			break;
//				
//			}
		}
	}
	
	private static boolean lineMissedVerification(boolean leftDetected, boolean rightDetected, double angle, double angleProtection) {
		
		if(angle>352 && angleProtection<7)
			angleProtection = angleProtection + 360;
		if(angle<7 && angleProtection>352)
			angle = angle + 360;
		
		if(Math.abs(angleProtection-angle)>55) {
			Sound.beepSequence();
			System.out.println("SHIET");
			
			if(leftDetected) {
				rightMotor.stop();
				rightMotor.backward();
				while(!rightDetected) {
					sensorVal2.fetchSample(sensorValData2, 0);
					rightDetection = sensorValData2[0];
					if(rightDetection<lineDetection) {
						rightMotor.stop(false);
						Sound.beep();
						break;
					}
				}
				return true;
			}
			
			if(rightDetected) {
				leftMotor.stop();
				leftMotor.backward();
				while(!leftDetected) {
					sensorVal.fetchSample(sensorValData, 0);
					leftDetection = sensorValData[0];
					if(leftDetection<lineDetection) {
						leftMotor.stop(false);
						Sound.beep();
						break;
					}
				}
				return true;
			}
			
		}
		return false;
	}
	
	/**
	 * This method detects when a falling edge occured
	 * 
	 * @param distance: distance detected by the ultrasonic sensor
	 * @return FallingEdge: Angle detected at the falling edge
	 */
	private static double fallingEdgeDetection(int distance) {
		
		FallingEdge = 0;	
		if(distance<DISTANCE_RANGE){
        	
        	if (distance<previousReading[0] && distance<previousReading[1]) {
        		
				Sound.beep();
				stopMotors();
				currentCoordinates = odometer.getXYT();
				FallingEdge = currentCoordinates[2];
				keepGoing = false;
				
			} 	
        	else {
        		updatePreviousReadings(distance);
        	}	
        }	
		return FallingEdge;	
	}	
	
	static void sleep(int a) {
		try {
			Thread.sleep(a);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
