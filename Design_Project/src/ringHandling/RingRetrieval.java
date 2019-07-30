package ringHandling;

import ca.mcgill.ecse211.project.DesignProjectMain;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import odometer.Odometer;

/**
 * "RingRetrieval" class is responsible of retrieving the a ring from the tree, identifying the color of that ring
 * and unloading the retrieved ring 
 *
 * @author Maxime Cardinal 
 * @dateCreated Thursday 8th November 2018
 * @version 1.5
 * @editHistory Maxime Cardinal - Creation of the class and reachRingSet() and ringGrab() methods (08/11/2018) (1.0)
 * 				Maxime Cardinal - Addition of the color identification (10/11/2018) (1.1)
 * 				Maxime Cardinal - Improvment of JavaDoc and comments (11/11/2018) (1.2)
 * 				Maxime Cardinal - Modification of the ring retrieval speed (13/11/2018) (1.3)
 * 				Maxime cardinal - Modification of the class architecture (thread -> class) (19/11/2018) (1.4)
 * 				Maxime Cardinal - Addition of the ringUnloading() method (19/11/2018) (1.5)
 * 
 */
public class RingRetrieval{
	
	//Odometer and motors declaration
	private static Odometer odometer;
	//motors
	static EV3LargeRegulatedMotor leftMotor;
	static EV3LargeRegulatedMotor rightMotor;
	static EV3LargeRegulatedMotor leftClawMotor;
	static EV3LargeRegulatedMotor rightClawMotor;
	
	//Initiate variables and constants
	private static final double WHEEL_RAD = DesignProjectMain.WHEEL_RAD;
	private static final double retrievalDistance = 15.5;
	private static final int clawControl = 90;
	private static final int ROTATE_SPEED = 100;
	
	//initiate all mean values (normalized)
	static double blueRingRedMean = 0.2139;
	static double blueRingGreenMean = 0.6483;
	static double blueRingBlueMean = 0.7307;
	
	static double greenRingRedMean = 0.4814;
	static double greenRingGreenMean = 0.8662;
	static double greenRingBlueMean = 0.1340;
	
	static double yellowRingRedMean = 0.8624;
	static double yellowRingGreenMean = 0.4922;
	static double yellowRingBlueMean = 0.1176;
	
	static double orangeRingRedMean = 0.9626;
	static double orangeRingGreenMean = 0.2564;
	static double orangeRingBlueMean = 0.0879;
	
	//initiate samples 
	static double redSample = 0.0;
	static double greenSample = 0.0;
	static double blueSample = 0.0;
	
	//initiate normalized samples
	static double normalizedRedSample;
	static double normalizedGreenSample;
	static double normalizedBlueSample;	
	
	//initiate colors detection
	static double blueRingDetector;
	static double greenRingDetector;
	static double yellowRingDetector;
	static double orangeRingDetector;
	static double detectionRange = 0.15;
	
	//initiate the sensor used for color identification
	private static Port lsPort = LocalEV3.get().getPort("S3");
	private static EV3ColorSensor lightSensor;
	
	//Constructor of the class
	public RingRetrieval(Odometer odo, EV3LargeRegulatedMotor leftClawMotor1, EV3LargeRegulatedMotor rightClawMotor1) {
		
		odometer = odo;
		leftMotor = odometer.leftMotor;
		rightMotor = odometer.rightMotor;
		
		leftClawMotor = leftClawMotor1;
		rightClawMotor = rightClawMotor1;
		
	}
	
	/**
	 * This method opens the robot's claws and
	 *  moves the robot forward up to the ring tree
	 */
	public static void reachRingSet() {
		
		
		
		//open the arms of the robot
		leftClawMotor.rotate(-clawControl,true);
		rightClawMotor.rotate(-clawControl,false);
		
		setSpeed(ROTATE_SPEED);
		
		//move forward to the retrievement position
		leftMotor.rotate(convertDistance(WHEEL_RAD, retrievalDistance), true);
	    rightMotor.rotate(convertDistance(WHEEL_RAD, retrievalDistance), false);
		
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
	 * This method controls the claws to grab a ring
	 */
	public static void ringGrab() {
		sleep(100);
		
		setClawSpeed(ROTATE_SPEED);
		//close the arms of the robot
		leftClawMotor.rotate(clawControl,true);
		rightClawMotor.rotate(clawControl,false);
		
		setSpeed(100);
		
		//move backward to its initial position
		leftMotor.rotate(-convertDistance(WHEEL_RAD, retrievalDistance), true);
	    rightMotor.rotate(-convertDistance(WHEEL_RAD, retrievalDistance), false);
		
	}
	
	/**
	 * this method is responsible for unloading the retrieved ring
	 * at the origin
	 */
	public static void ringUnloading() {
		sleep(100);
		
		leftClawMotor.rotate(-clawControl,true);
		rightClawMotor.rotate(-clawControl,false);
		
		sleep(100);
		
		leftMotor.rotate(convertDistance(WHEEL_RAD,-3),true);
		rightMotor.rotate(convertDistance(WHEEL_RAD,-3),false);
	}
	
	/**
	 * This method changes the rotation speed of the wheels' motors
	 * 
	 * @param wheelSpeed: Rotation speed of the wheels
	 */
	private static void setClawSpeed(int wheelSpeed) {
		stopMotors();
		
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);
		
		leftClawMotor.setSpeed(wheelSpeed);
		rightClawMotor.setSpeed(wheelSpeed);
		
		sleep(100);
	}
	
	/**
	 * This method changes the rotation speed of the wheels' motors
	 * 
	 * @param wheelSpeed: Speed of rotation of the wheels
	 */
	private static void setSpeed(int wheelSpeed) {
		stopMotors();
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);
		
		leftMotor.setSpeed(wheelSpeed);
		rightMotor.setSpeed(wheelSpeed);
		
		sleep(100);
		
	}
	
	/**
	 * This method is responsible for identifying the color
	 * of the ring retrieved 
	 */
	public static void colorIdentification() {
		
		//initiate light sensor for detection
		lightSensor = new EV3ColorSensor(lsPort);
		lightSensor.setCurrentMode("RGB");
		SampleProvider colorSensor = lightSensor.getRGBMode();
		float[] rgb=new float[colorSensor.sampleSize()];
		
		boolean keepGoing = true;
		
		int counter = 0; //if there is no ring detected, it will be used to break the while and move on to the next tasks
		
		while(keepGoing && counter<10) {
			
			colorSensor.fetchSample(rgb, 0); //get samplpe from sensor
			
			redSample = rgb[0];		
			greenSample =  rgb[1];
			blueSample =  rgb[2];		
			
			//denominator calculation of the normalization
			double normalizationDivider = Math.sqrt(redSample*redSample+greenSample*greenSample+blueSample*blueSample);
			
			//normalize the samples
			normalizedRedSample = redSample / normalizationDivider;
			normalizedGreenSample = greenSample / normalizationDivider;
			normalizedBlueSample = blueSample / normalizationDivider;
			
			blueRingDetector = Math.sqrt(Math.pow((normalizedRedSample-blueRingRedMean), 2)
											+Math.pow((normalizedGreenSample-blueRingGreenMean), 2)
												+Math.pow((normalizedBlueSample-blueRingBlueMean), 2));
			
			greenRingDetector = Math.sqrt(Math.pow((normalizedRedSample-greenRingRedMean), 2)
											+Math.pow((normalizedGreenSample-greenRingGreenMean), 2)
												+Math.pow((normalizedBlueSample-greenRingBlueMean), 2));
			
			yellowRingDetector = Math.sqrt(Math.pow((normalizedRedSample-yellowRingRedMean), 2)
											+Math.pow((normalizedGreenSample-yellowRingGreenMean), 2)
												+Math.pow((normalizedBlueSample-yellowRingBlueMean), 2));
			
			orangeRingDetector = Math.sqrt(Math.pow((normalizedRedSample-orangeRingRedMean), 2)
											+Math.pow((normalizedGreenSample-orangeRingGreenMean), 2)
												+Math.pow((normalizedBlueSample-orangeRingBlueMean), 2));
			
			//check which ring has been detected
			if(blueRingDetector<detectionRange) {
				//blue ring is detected
		        Sound.beep();
		        keepGoing = false;
			}
			else if(greenRingDetector<detectionRange) {
				//green ring is detected
		        Sound.beep();
		        Sound.beep();
		        keepGoing = false;
			}
			else if(yellowRingDetector<detectionRange) {
				//yellow ring is detected
		        Sound.beep();
		        Sound.beep();
		        Sound.beep();
		        keepGoing = false;
			}
			else if(orangeRingDetector<detectionRange) {
				//orange ring is detected
		        Sound.beep();
		        Sound.beep();
		        Sound.beep();
		        Sound.beep();
		        keepGoing = false;
			}
			else {
				//no ring is detected
				counter++;
			}
		}
		if(counter == 10) {
			System.out.println("No ring detected");
		}
	}
	
	static void stopMotors() {
		leftMotor.stop();
		rightMotor.stop();
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
