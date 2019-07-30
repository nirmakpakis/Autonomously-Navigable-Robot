package navigation;

import ca.mcgill.ecse211.project.DesignProjectMain;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import odometer.Odometer;

/**
 * "TravelToTree" class is responsible of initiating the game plan 
 * and travelling the robot to the ring tree
 *
 * @author Maxime Cardinal, Irmak Pakis
 * @dateCreated 3rd November 2018
 * @version 1.3
 *
 */
public class TravelToTree {

	//Odometer initialization 
	private static Odometer odometer;
	
	//Left light sensor
	private static SampleProvider sensorVal;
	static float[] sensorValData;
	
	//Right light sensor
	private static SampleProvider sensorVal2;
	static float[] sensorValData2;
	
	//motors
	static EV3LargeRegulatedMotor leftMotor;
	static EV3LargeRegulatedMotor rightMotor;
	
	//Bridge orientation
	private final static int horizontal = 1;
	private final static int vertical = 2;
	private static int bridgeOrientation;
	private final static int oneByOneBridge = 2; //if the bridge is 1 by 1, the distance to cross it is 2 tiles
	private final static int twoByTwoBridge = 3; //if the bridge is 1 by 1, the distance to cross it is 3 tiles
	private static int bridgeSize;
	
	//variables used for the travelTo method
	static double xDistance = 0;
	static double yDistance = 0;
	static double angleBeta = 0;
	static double distanceToTravel = 0;
	static double finalAngle = 0;
	
	static double currentCoordinates [] = new double[3];
	static double currentXposition = 0;
	static double currentYposition = 0;
	static double currentAngle = 0;
	static double angleProtection = 0;
	static final double angleDifference = 35;
	private static double leftDetection;
	private static double rightDetection;
	
	//Constants
	public static final double WHEEL_RAD = DesignProjectMain.WHEEL_RAD;
  	public static final double TRACK = DesignProjectMain.TRACK; 
  	private static final int FORWARD_SPEED = 250;
    private static final int ROTATE_SPEED = 100;
  	private static final double TILE_SIZE = 30.48;
	private static final double HALF_TILE = 0.5;
	private static final double TILE = 1.0;
	private static final double sensorOffset = -5.5;
	private static final double correctionOffset = -5.0;
	private static final double lineDetection = 0.24;
	
	/**
	 * Game parameters' name definition:
	 * 	LL = Lower-Left
	 * 	UR = Upper-Righ
	 * 	BR = Bridge
	 * 	T = Tree
	 *  CO = Corner
	 */	
	//Game-plan parameters
	private static int StartingCorner; 
	//bridge parameters
	private static int BR_LL_x;
	private static int BR_LL_y;
	private static int BR_UR_x;
	private static int BR_UR_y;
	//ring tree parameters
	private static int T_x;
	private static int T_y;
	
	/**
	 * Constructor of the class
	 * 
	 * @param odometer: Odometer
	 * @param sensorVal: Sample Provider
	 * @param sensorValData: Left light sensor data
	 * @param sensorVal2: Sample Provider
	 * @param sensorValData2: Right light sensor data
	 */
	public TravelToTree(Odometer odometer_,SampleProvider sensorVal_, float[] sensorValData_,
					SampleProvider sensorVal2_, float[] sensorValData2_, int startingCorner, int CO_LL_x, int CO_LL_y, int CO_UR_x, int CO_UR_y,
							int BR_LL_x_, int BR_LL_y_, int BR_UR_x_, int BR_UR_y_, int T_x_, int T_y_) {

		odometer = odometer_;
		leftMotor = odometer.leftMotor;
		rightMotor = odometer.rightMotor;
		
		sensorVal = sensorVal_;
		sensorValData = sensorValData_;
		sensorVal2 = sensorVal2_;
		sensorValData2 = sensorValData2_;
		
		//Game-plan parameters
		StartingCorner = startingCorner; 
		//bridge parameters
		BR_LL_x = BR_LL_x_;
		BR_LL_y = BR_LL_y_;
		BR_UR_x = BR_UR_x_;
		BR_UR_y = BR_UR_y_;
		//ring tree parameters
		T_x = T_x_;
		T_y = T_y_;
	
	}
	
	/**
	 * This method travels the robot from its starting point to 
	 * the tile in front of the bride
	 */
	public static void travelToBridge() {
		
		//if the bridge is horizontally oriented
		if(bridgeOrientation() == horizontal) {
			
			if(StartingCorner == 0) {
				//correct the initial position after ultrasonic localization
				odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
				//travel to the bridge and correct its position
				travelTo((BR_LL_x-TILE), BR_LL_y);
				odometerCorrection((BR_LL_x-TILE), BR_LL_y);
				//reach the entrance of the bridge
				travelTo((BR_LL_x-HALF_TILE), (BR_LL_y+HALF_TILE));
			}
			else if(StartingCorner == 1) {
				//correct the initial position after ultrasonic localization
				odometer.setXYT(7*TILE_SIZE, TILE_SIZE, 270);
				//travel to the bridge and correct its position
				travelTo((BR_UR_x+TILE), (BR_UR_y-TILE));
				odometerCorrection((BR_UR_x+TILE), (BR_UR_y-TILE));
				//reach the entrance of the bridge
				travelTo((BR_UR_x+HALF_TILE), (BR_UR_y-HALF_TILE));
			}
			else if(StartingCorner == 2) {
				//correct the initial position after ultrasonic localization
				odometer.setXYT(7*TILE_SIZE, 7*TILE_SIZE, 180);
				//travel to the bridge and correct its position
				travelTo((BR_UR_x+TILE), BR_UR_y);
				odometerCorrection((BR_UR_x+TILE), BR_UR_y);
				//reach the entrance of the bridge
				travelTo((BR_UR_x+HALF_TILE), (BR_UR_y-HALF_TILE));
			}
			else if(StartingCorner == 3) {
				//correct the initial position after ultrasonic localization
				odometer.setXYT(TILE_SIZE, 7*TILE_SIZE, 90);
				//travel to the bridge and correct its position
				travelTo((BR_LL_x-TILE), (BR_LL_y+TILE));
				odometerCorrection((BR_UR_x-TILE), (BR_UR_y+TILE));
				//reach the entrance of the bridge
				travelTo((BR_LL_x-HALF_TILE), (BR_LL_y+HALF_TILE));
			}
		}
		//if the bridge is vertically oriented
		else if(bridgeOrientation() == vertical) {
			
			if(StartingCorner == 0) {
				//correct the initial position after ultrasonic localization
				odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
				//travel to the bridge and correct its position
				travelTo(BR_LL_x,(BR_LL_y-TILE));
				odometerCorrection(BR_LL_x,(BR_LL_y-TILE));
				//reach the entrance of the bridge
				travelTo((BR_LL_x+HALF_TILE), (BR_LL_y-HALF_TILE));	
			}
			else if(StartingCorner == 1) {
				//correct the initial position after ultrasonic localization
				odometer.setXYT(7*TILE_SIZE, TILE_SIZE, 270);
				//travel to the bridge and correct its position
				travelTo((BR_LL_x+TILE),(BR_LL_y-TILE));
				odometerCorrection((BR_LL_x+TILE),(BR_LL_y-TILE));
				//reach the entrance of the bridge
				travelTo((BR_LL_x+HALF_TILE), (BR_LL_y-HALF_TILE));
			}
			else if(StartingCorner == 2) {
				//correct the initial position after ultrasonic localization
				odometer.setXYT(7*TILE_SIZE, 7*TILE_SIZE, 180);
				//travel to the bridge and correct its position
				travelTo(BR_UR_x,(BR_UR_y+TILE));
				odometerCorrection(BR_UR_x,(BR_UR_y+TILE));
				//reach the entrance of the bridge
				travelTo((BR_UR_x-HALF_TILE), (BR_UR_y+HALF_TILE));
			}
			else if(StartingCorner == 3) {
				//correct initial positon after ultrasonic localization
				odometer.setXYT(TILE_SIZE, 7*TILE_SIZE, 90);
				//travel to the bridge and correct its position
				travelTo((BR_UR_x-TILE),(BR_UR_y+TILE));
				odometerCorrection((BR_UR_x-TILE),(BR_UR_y+TILE));
				//reach the entrance of the bridge
				travelTo((BR_UR_x-HALF_TILE), (BR_UR_y+HALF_TILE));
			}
		}	
	}
	
	/**
	 * This method returns the orientation of the bridge
	 * 
	 * @return: Returns the bridge orientation
	 */
	private static int bridgeOrientation() {
		
		//checks the distance in x between lower-left corner and upper-right corner
		if(Math.abs(BR_LL_x - BR_UR_x) == 2) {
			bridgeOrientation = horizontal;
			bridgeSize = twoByTwoBridge;
			return bridgeOrientation;
		}
		//checks the distance in y between lower-left corner and upper-right corner
		else if(Math.abs(BR_LL_y - BR_UR_y) == 2) {
			bridgeOrientation = vertical;
			bridgeSize = twoByTwoBridge;
			return bridgeOrientation;
		}
		//bridge is one by one
		else {
			bridgeOrientation = oneByOneBridge();
			bridgeSize = oneByOneBridge;
		}
		return bridgeOrientation;
				
	}
	
	/**
	 * This method is returns the orientation of a one by one bridge
	 * 
	 * @return: Orientation of the one by one bridge 
	 */
	private static int oneByOneBridge() {
		
		if(StartingCorner == 0 || StartingCorner == 3) {		
			if(DesignProjectMain.CO_UR_x == BR_LL_x) {
				bridgeOrientation = horizontal;
			}
			else {
				bridgeOrientation = vertical;
			}
		}
		else if(StartingCorner == 1 || StartingCorner == 2) {
			if(DesignProjectMain.CO_LL_x == BR_UR_x) {
				bridgeOrientation = horizontal;
			}
			else {
				bridgeOrientation = vertical;
			}
		}
		return bridgeOrientation;
	}
	
	/**
	 * This method travels the robot through the bridge
	 */
	public static void travelThroughBridge() {
		setSpeed(100);
		//if the bridge is horizontally oriented, travel in x
		if(bridgeOrientation() == horizontal) {
			
			if(StartingCorner == 0) {//good
				travelTo((BR_LL_x-HALF_TILE+bridgeSize), (BR_LL_y+HALF_TILE));
				travelTo((BR_LL_x+bridgeSize), (BR_LL_y+TILE));
				odometerCorrection((BR_LL_x+bridgeSize), (BR_LL_y+TILE));
			}
			else if(StartingCorner == 1) {
				travelTo((BR_UR_x+HALF_TILE-bridgeSize), (BR_UR_y-HALF_TILE));
				travelTo((BR_UR_x-bridgeSize), (BR_UR_y-TILE));
				odometerCorrection((BR_UR_x-bridgeSize), (BR_UR_y-TILE));
			}
			else if(StartingCorner == 2) {
				travelTo((BR_UR_x+HALF_TILE-bridgeSize), (BR_UR_y-HALF_TILE));
				travelTo((BR_UR_x-bridgeSize), (BR_UR_y-TILE));
				odometerCorrection((BR_UR_x-bridgeSize), (BR_UR_y-TILE));
			}
			else if(StartingCorner == 3) {
				travelTo((BR_LL_x-HALF_TILE+bridgeSize), (BR_LL_y+HALF_TILE));
				travelTo((BR_LL_x+bridgeSize), (BR_LL_y+TILE));
				odometerCorrection((BR_LL_x+bridgeSize), (BR_LL_y+TILE));
			}
		}
		//if the bridge is vertically oriented, travel in y
		else if(bridgeOrientation() == vertical) {
			
			if(StartingCorner == 0) {
				travelTo((BR_LL_x+HALF_TILE), (BR_LL_y-HALF_TILE+bridgeSize));	
				travelTo((BR_LL_x+TILE), (BR_LL_y+bridgeSize));
				odometerCorrection((BR_LL_x+TILE), (BR_LL_y+bridgeSize));
			}
			else if(StartingCorner == 1) {
				travelTo((BR_LL_x+HALF_TILE), (BR_LL_y-HALF_TILE+bridgeSize));
				travelTo((BR_LL_x+TILE), (BR_LL_y+bridgeSize));
				odometerCorrection((BR_LL_x+TILE), (BR_LL_y+bridgeSize));
			}
			else if(StartingCorner == 2) {
				travelTo((BR_UR_x-HALF_TILE), (BR_UR_y+HALF_TILE-bridgeSize));
				travelTo((BR_UR_x), (BR_UR_y+TILE-bridgeSize));
				odometerCorrection((BR_UR_x), (BR_UR_y+TILE-bridgeSize));
			}
			else if(StartingCorner == 3) {
				travelTo((BR_UR_x-HALF_TILE), (BR_UR_y+HALF_TILE-bridgeSize));
				travelTo((BR_UR_x), (BR_UR_y+TILE-bridgeSize));
				odometerCorrection((BR_UR_x), (BR_UR_y+TILE-bridgeSize));
			}
		}	
	}



	
	/**
	 * This method travels the robot to the ring set and
	 * changes the robot's orientation to face the ring set 
	 */
	public static void travelToRingSet() {
		
		currentCoordinates = odometer.getXYT();
		
		if(T_x < currentCoordinates[0]/TILE_SIZE && T_y < currentCoordinates[1]/TILE_SIZE) {
			travelTo(T_x+1,T_y+1);
			odometerCorrection(T_x+1,T_y+1);
			travelTo(T_x+1,T_y);
			changeOrientation(T_x,T_y);
		}else if(T_x < currentCoordinates[0]/TILE_SIZE && T_y > currentCoordinates[1]/TILE_SIZE) {
			travelTo(T_x+1,T_y-1);
			odometerCorrection(T_x+1,T_y-1);
			travelTo(T_x+1,T_y);
			changeOrientation(T_x,T_y);
		}else if(T_x > currentCoordinates[0]/TILE_SIZE && T_y > currentCoordinates[1]/TILE_SIZE) {
			travelTo(T_x-1,T_y-1);
			odometerCorrection(T_x-1,T_y-1);
			travelTo(T_x-1,T_y);
			changeOrientation(T_x,T_y);
		}
		else if(T_x > currentCoordinates[0]/TILE_SIZE && T_y < currentCoordinates[1]/TILE_SIZE ) {
			travelTo(T_x-1,T_y+1);
			odometerCorrection(T_x-1,T_y+1);
			travelTo(T_x-1,T_y);
			changeOrientation(T_x,T_y);
		}
		
	}
	
	/**
	 * This method changes the orientation of the robot so that it faces the 
	 * ring tree for the retrievement
	 * 
	 * @param x: x location of the ring tree on the grid
	 * @param y: y location of the ring tree on the grid
	 */
	private static void changeOrientation(double x, double y){
		
		currentCoordinates = odometer.getXYT();
		currentXposition = currentCoordinates[0];
		currentYposition = currentCoordinates[1];
		currentAngle = currentCoordinates[2];
		
		xDistance = x*TILE_SIZE - currentXposition;//distance in x to travel
		yDistance = y*TILE_SIZE - currentYposition;//distance in y to travel
		
		angleBeta = Math.atan2(xDistance, yDistance); //calculate the angle where the next point is	
		finalAngle = angleBeta - currentAngle*Math.PI/180;
		
		if(finalAngle<-Math.PI) {
			finalAngle = finalAngle + 2*Math.PI;
		}
		else if(finalAngle>Math.PI){
			finalAngle = finalAngle - 2*Math.PI;
		}
		turnTo(finalAngle);	
	    
	}


	/**
	 * This method is responsible of the travel of the robot from a 
	 * point to another
	 * 
	 * @param x: corner position on the x-axis
	 * @param y: corner position on the y-axis
	 * @throws InterruptedException 
	 */
	private static void travelTo(double x, double y){
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);
		
		currentCoordinates = odometer.getXYT();
		currentXposition = currentCoordinates[0];
		currentYposition = currentCoordinates[1];
		currentAngle = currentCoordinates[2];
		
		xDistance = x*TILE_SIZE - currentXposition;//distance in x to travel
		yDistance = y*TILE_SIZE - currentYposition;//distance in y to travel
		
		angleBeta = Math.atan2(xDistance, yDistance); //calculate the angle where the next point is	
		finalAngle = angleBeta - currentAngle*Math.PI/180;
		
		if(finalAngle<-Math.PI) {
			finalAngle = finalAngle + 2*Math.PI;
		}
		else if(finalAngle>Math.PI){
			finalAngle = finalAngle - 2*Math.PI;
		}
		turnTo(finalAngle);	
		
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
		
		distanceToTravel = Math.hypot(xDistance,yDistance);//calculate the hypotenuse
		
		setSpeed(FORWARD_SPEED);
		
		leftMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel),true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), false);
	    
	}
	
	/**
	 * This method is responsible for adjusting the angle of the robot
	 * toward the next position it needs to travel
	 * 
	 * @param theta: angle, in radian, to turn the robot to
	 */
	private static void turnTo(double theta) {
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		sleep(100);
		
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta*180/Math.PI), true);
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta*180/Math.PI), false);
	
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
	
	/**
	 * This method changes the rotation speed of the wheels' motors
	 * 
	 * @param wheelSpeed: Speed of rotation of the wheels
	 */
	private static void setSpeed(int wheelSpeed) {
		
		leftMotor.setSpeed(wheelSpeed);
		rightMotor.setSpeed(wheelSpeed);
		
	}
	
	/**
	 * This method corrects the sensor offset to the wheelcenter
	 * in the y-axis
	 */
	private static void sensorOffsetCorrection() {
		leftMotor.stop();
		rightMotor.stop();
		
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);
		
		setSpeed(70);
		
		sleep(100);
		
		leftMotor.rotate(convertDistance(WHEEL_RAD, sensorOffset), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, sensorOffset), false);
		
	}
	
	/**
	 * This method corrects the x, y and theta of the robot 
	 * according to an intersection of the grid
	 * 
	 * @param x: x corrected position of the robot
	 * @param y: y corrected position of the robot
	 */
	private static void odometerCorrection(double x, double y) {
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		setSpeed(70);
		sleep(100);
		
		changeOrientation(x,y+1);	//change the orientation of the robot to be 0 degree
		
		leftMotor.rotate(convertDistance(WHEEL_RAD, correctionOffset), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, correctionOffset), false);
		
		angleCorrection();
	
		sensorOffsetCorrection();
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		setSpeed(ROTATE_SPEED);
		sleep(100);
		
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), false);
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		setSpeed(ROTATE_SPEED);
		sleep(100);
		
		leftMotor.rotate(convertDistance(WHEEL_RAD, correctionOffset), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, correctionOffset), false);

		angleCorrection();
	
		sensorOffsetCorrection();
		odometer.setXYT(x*TILE_SIZE, y*TILE_SIZE, 90);
		sleep(100);
		
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
		
		angleProtection = odometer.getXYT()[2];
		boolean rightDetected = false, leftDetected = false;

		rightMotor.forward();
		leftMotor.forward();

		while (true) {
			sensorVal.fetchSample(sensorValData, 0);
			sensorVal2.fetchSample(sensorValData2, 0);
			double rightS = sensorValData2[0];
			double leftS = sensorValData[0];
			if (rightS < 0.35 && !rightDetected) {
				rightMotor.stop(!leftDetected);
				rightDetected = true;
//				Sound.beep();
			}
			if (leftS < 0.35 && !leftDetected) {
				leftMotor.stop(!rightDetected);
				leftDetected = true;
//				Sound.beep();
			}
			if (leftDetected && rightDetected)
				break;
			
			if(lineMissedVerification(leftDetected, rightDetected, odometer.getXYT()[2], angleProtection)) {
				break;
				
			}
		}
	}
	
	/**
	 * This method checks if the robot missed a line while correcting itself 
	 * and re-correct the robot's position
	 * 
	 * @param leftDetected : boolean
	 * @param rightDetected : boolean
	 * @return boolean: returns true if a line was missed
	 */
	private static boolean lineMissedVerification(boolean leftDetected, boolean rightDetected, double angle, double angleProtection) {
		
		if(angle>352 && angleProtection<7)
			angleProtection = angleProtection + 360;
		if(angle<7 && angleProtection>352)
			angle = angle + 360;
		
		if(Math.abs(angleProtection-angle)>25) {
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
	
	
	static void sleep(int a) {
	try {
		Thread.sleep(a);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	}
}
