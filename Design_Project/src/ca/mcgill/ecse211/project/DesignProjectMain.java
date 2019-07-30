package ca.mcgill.ecse211.project;


import navigation.Localization;
import navigation.TravelToInitialPosition;
import navigation.TravelToTree;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import odometer.Odometer;
import odometer.OdometerExceptions;
import ringHandling.RingRetrieval;

/**
 * "DesignProjectMain" class is responsible of initiating all sensors
 *and managing(calling/starting) methods and threads
 * 
 * @author Maxime Cardinal, Irmak Pakis
 * @dateCreated Monday 29th October 2018
 * @version 1.3
 *
 */
public class DesignProjectMain {
	
	//EV3 motors used to travel
	static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));	
	
	//EV3 motor used to move the claws
	static final EV3LargeRegulatedMotor leftClawMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	static final EV3LargeRegulatedMotor rightClawMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
		
	//Ultrasonic sensor used for the light localization
	private static final Port usPort = LocalEV3.get().getPort("S1");	
    static SensorModes usSensor = new EV3UltrasonicSensor(usPort); 		
    static SampleProvider usDistance = usSensor.getMode("Distance"); 	
    static float[] usData = new float[usDistance.sampleSize()];		    
    
    //Left light sensor used for the correction (identified as light sensor 1)
  	private static final Port lsPort = LocalEV3.get().getPort("S2");
  	static EV3ColorSensor LightSensor = new EV3ColorSensor(lsPort);
  	static SampleProvider sensorVal = LightSensor.getRedMode();
  	static float[] sensorValData = new float[LightSensor.sampleSize()];
  	
  	//Right light sensor used for the correction (identified as light sensor 2)
  	private static final Port lsPort2 = LocalEV3.get().getPort("S4");
  	static EV3ColorSensor LightSensor2 = new EV3ColorSensor(lsPort2);
  	static SampleProvider sensorVal2 = LightSensor2.getRedMode();
  	static float[] sensorValData2 = new float[LightSensor2.sampleSize()];
  	
  	//Initiate variables and constants
  	private static final TextLCD textlcd = LocalEV3.get().getTextLCD();
  	public static final double WHEEL_RAD = 2.03;
  	public static final double TRACK = 13.3; 
  	public static double lineDetection = 0.0;
  	public static int buttonChoice;
  	private static final int ACCELERATION = 1500;

  	/**
	 * Game parameters' name definition:
	 * 	LL = Lower-Left
	 * 	UR = Upper-Righ
	 * 	BR = Bridge
	 * 	T = Tree
	 *  CO = Corner
	 */	
	//Game-plan parameters
	//  public static int StartingCorner; 
	//Corner parameters
	public static int CO_LL_x;
	public static int CO_LL_y;
	public static int CO_UR_x;
	public static int CO_UR_y; 
	//Island parameters
	public static int Island_LL_x;
	public static int Island_LL_y;
	public static int Island_UR_x;
	public static int Island_UR_y;
	//Bridge parameters
    public static int StartingCorner =2;
    public static int TN_LL_x =3;
    public static int TN_LL_y =3;
    public static int TN_UR_x =5;
    public static int TN_UR_y =4;
    public static int T_x =1;
    public static int T_y =7;

  	/**
  	 * Main function of the program, wait for the wifi inputs and then calls the required
  	 * methods and threads
  	 * 
  	 * @param args
  	 */
    public static void main(String[] args) throws OdometerExceptions {
    	
    	//Start the odometer thread
    	Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    	Thread odometerThread = new Thread(odometer);
        odometerThread.start();
     
        //wait until the user starts the program
	    do {
	        // clear the display
	        textlcd.clear();
	        
	        // ask to start the program
	        textlcd.drawString("Welcome team 17!", 0, 0);
	        textlcd.drawString("				    ", 0, 1);
	        textlcd.drawString("  -- MADMAX  -- ", 0, 2);
	        textlcd.drawString(" 				", 0, 3);
	        textlcd.drawString("    		     	", 0, 4);
	        textlcd.drawString("    <-run->	 	", 0, 5);
	        
	        buttonChoice = Button.waitForAnyPress(); // Record choice 
	        
	    }while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT); 
	    
	    if(buttonChoice == Button.ID_LEFT || buttonChoice == Button.ID_RIGHT) {
	    	
	    	textlcd.clear();
	    	
	    	//starts the wifi polling
	    	
	    	
	    	//wait until the data is received from the Wifi class
	 
	    	
	    	odometer.leftMotor.setAcceleration(ACCELERATION);
	    	odometer.rightMotor.setAcceleration(ACCELERATION);
	    	
	    	//Start the localization
	        @SuppressWarnings("unused")
			Localization localization = new Localization(odometer, usDistance, usData, sensorVal, sensorValData, sensorVal2, sensorValData2);
	        Localization.getBoardLight();
	        Localization.usLocalization();
	        Localization.lsLocalization();
	        
	        Sound.beep();
			Sound.beep();
			Sound.beep();
			
			//Start the travelling
			@SuppressWarnings("unused")
			TravelToTree travelToTree = new TravelToTree(odometer, sensorVal, sensorValData, sensorVal2, sensorValData2, StartingCorner,
																CO_LL_x,CO_LL_y, CO_UR_x,CO_UR_y, TN_LL_x,TN_LL_y, TN_UR_x,TN_UR_y, T_x,T_y);
			TravelToTree.travelToBridge();
			TravelToTree.travelThroughBridge();
			TravelToTree.travelToRingSet();
			
			Sound.beep();
			Sound.beep();
			Sound.beep();
			
			//Start ring retrieval
	        @SuppressWarnings("unused")
			RingRetrieval ringRetrieval = new RingRetrieval(odometer, leftClawMotor, rightClawMotor);
	        RingRetrieval.reachRingSet();
	        RingRetrieval.ringGrab();
	        RingRetrieval.colorIdentification();
	        
	        //Must add the travel back to origin part
	        TravelToInitialPosition max = new TravelToInitialPosition(odometer, sensorVal, sensorValData, sensorVal2, sensorValData2, StartingCorner,
					CO_LL_x,CO_LL_y, CO_UR_x,CO_UR_y, TN_LL_x,TN_LL_y, TN_UR_x,TN_UR_y, T_x,T_y);
	        max.MaxGoesBack();
	        
	       
	        //Start ring unloading
	        RingRetrieval.ringUnloading();
	        
	        Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();
	        
	        //Stop the program when pressing the exit button
	        while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			System.exit(0);
			
	    }
    	
    }
 
}
