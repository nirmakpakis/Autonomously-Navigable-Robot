package odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * "Odometer" class is responsible for keeping track of the robot's
 *position and orientation according to the wheels displacement
 * 
 * @author Maxime Cardinal , Irmak Pakis
 * @dateCreated Monday 29th October 2018
 * @version 1.1
 * @editHistory Maxime Cardinal - Creation (29/10/2018) (1.0)
 * 				Irmak Pakis - Correction of the odometer problem (12/11/2018) (1.1)
 *
 */
public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  public EV3LargeRegulatedMotor leftMotor;
  public EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;

  private double distanceLeft;
  private double distanceRight;
	
  public double leftMotorLastTachoCount;
  public double rightMotorLastTachoCount;
	
  private double deltaD;
  private double deltaT;
	
  private double Theta;
	
  private double dX;
  private double dY;
 
  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();

      //Calculate new robot position based on tachometer counts
      distanceLeft = (Math.PI*WHEEL_RAD*(leftMotorTachoCount-leftMotorLastTachoCount)/180); // compute wheel displacements
      distanceRight = (Math.PI*WHEEL_RAD*(rightMotorTachoCount-rightMotorLastTachoCount)/180);  
		
      leftMotorLastTachoCount=leftMotorTachoCount; 
      rightMotorLastTachoCount=rightMotorTachoCount;
      
      deltaD =  (0.5*(distanceLeft+distanceRight)); // compute vehicle displacement
      deltaT =  ((distanceLeft-distanceRight)/TRACK); 
      
      Theta = odo.getXYT()[2]*Math.PI/180;
      
      Theta += deltaT; 
	
      dX =  (deltaD * Math.sin(Theta)); // change in X
      dY =  (deltaD * Math.cos(Theta)); // Change in Y
		
      double thetaInD = (((deltaT*180/Math.PI)));//rad to deg
      odo.update(dX, dY, thetaInD);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }
}
