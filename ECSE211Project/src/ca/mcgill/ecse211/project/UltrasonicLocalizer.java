package ca.mcgill.ecse211.project;

/** 
 * US localization class
 * @author Antonios Valkanas, Borui Tao
 * @version 1.0
 *  
 */

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class uses the ultrasonic sensor as well as falling / rising edge routine to know what is the 0 degrees direction.
 */
public class UltrasonicLocalizer{
	/**
	 * The choice for either rising (0) or falling (1) edge routine.
	 */
	private int choice;
	/**
	 * The pointer to the Navigation class.
	 */
	private Navigation na;
	/**
	 * The pointer to the Odometer class.
	 */
	private Odometer odometer;
	/**
	 * The pointer to the left motor.
	 */
	private EV3LargeRegulatedMotor leftMotor;
	/**
	 * The pointer to the right motor.
	 */
	private EV3LargeRegulatedMotor rightMotor;

	/**
	 * The value got by the front light sensor an updated by the LightSensor class.
	 */
	public static float lightValue;
	/**
	 * The distance between the robot and the wall.
	 */
	public int distance;

	/**
	 * The angle where the robot detect the wall for the first time.
	 */
	private double firstAngle;

	private double secondAngle;		// The angle where the robot detect the wall for the second time
	/**
	 * The angle that the robot is away from the zero angle.
	 */
	private double correctedTheta;

	/**
	 * Used to prevent the robot from detecting a point 2 times.
	 */
	private boolean firstPointDetected;
	/**
	 * Used to prevent the robot from detecting a point 2 times.
	 */
	private boolean secondPointDetected;
	/**
	 * The current theta angle.
	 */
	private double locationTheta;


	/**
	 * The constructor for this class sets the navigation and odometer objects.
	 * @param navigation	pointer to the navigation class
	 * @param odometer		pointer to the Odometer
	 */
	public UltrasonicLocalizer(Navigation navigation, Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.na = navigation;
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		this.distance = 0;
		this.correctedTheta = 0;
		this.firstPointDetected = false;
		this.secondPointDetected = false;

		this.choice = 0;
	}

	/** 
	 * This method updates the distance.
	 * 
	 * @param  distance   The distance from the object detected by the Ultrasonic Sensor
	 */
	public void processUSData(int distance) {
		this.distance = distance;
	}

	/** 
	 * This method executes the ultrasonic localization.
	 */
	public void doUltrasonicLocalization(){
		na.makeTurn(360, false, true);

		while (distance == 1){

		}

		choice = (distance > (CaptureFlag.DISTANCE_THRESHOLD + CaptureFlag.NOISE_MARGIN+2)) ? 1 : 0;

		if (choice == 0) {
			risingEdge();
			System.out.println("we are using risingEdge!");
		}
		else {
			fallingEdge();
			System.out.println("we are using fallingEdge!");
		}
	}
	
	/**
	 * This method localizes the robot in the falling edge mode
	 * The ultrasonic sensor in front of the robot keeps detecting the distance between the wall and the robot
	 * When the ultrasonic sensor detects the falling edge, the robot records the turning angle, reverses the turning 
	 * direction and keeps turning until it finds the next falling edge. 
	 * 
	 * Because the noise margin exists, we record two turning angles.The first one is when the distance is greater than the sum of
	 * distance threshold and the noise margin (d+k); The second one is when it is less than the difference between them (d-k).
	 * Every time the falling edge is detected when the robot gets the two readings and the turning angle is the average of these two 
	 * readings.
	 *
	 */
	private void fallingEdge(){
		double theta1 = 0;
		double theta2 = 0;
		// Detect the first angle where the robot sees a wall from the distance. 
		// wait until the distance is greater than the threshold + noise margin + 1
		while(distance > (CaptureFlag.DISTANCE_THRESHOLD + CaptureFlag.NOISE_MARGIN+1)){
		}
		if(!firstPointDetected){
			theta1 = odometer.getTheta();
			// The second theta is detected if the distance is smaller than the threshold - noise margin - 1
			if (distance <= (CaptureFlag.DISTANCE_THRESHOLD - CaptureFlag.NOISE_MARGIN - 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			// the first angle is the average of the two thetas
			firstAngle = (theta1 + theta2) / 2;
			firstPointDetected = true;
		}
		//stop the motors and make it turn the opposite direction
		na.motorStop();
		na.makeTurn(-360, false, true);

		try {
			Thread.sleep(1500);
		} catch (Exception e) {
		}

		// Detect the second angle where the robot sees a wall from the distance. 
		// wait until the distance is greater than the threshold + noise margin + 1
		while(distance > (CaptureFlag.DISTANCE_THRESHOLD + CaptureFlag.NOISE_MARGIN+1)){
		}
		if (!secondPointDetected){
			theta1 = odometer.getTheta();

			// The second theta is detected if the distance is smaller than the threshold - noise margin - 1
			if (distance <= (CaptureFlag.DISTANCE_THRESHOLD - CaptureFlag.NOISE_MARGIN - 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			secondPointDetected = true;
			// the second angle is the average of the two thetas
			secondAngle = (theta1 + theta2) / 2;
		}
		na.motorStop();
		correctAngleAndLocation();
	}

	/**
	 * This method localizes the robot in the rising edge mode. 
	 * 
	 * Similar to the falling edge, we record two turning angles and record their average value to be the turning angle. 
	 * The first one is when the distance is less than the difference of distance threshold and the noise margin (d-k); 
	 * The second one is when it is greater than the sum between them (d+k).
	 * Every time the rising edge is detected when the robot gets the two readings and the turning angle is the average of these two 
	 * readings.
	 *
	 */
	private void risingEdge(){
		double theta1 = 0;
		double theta2 = 0;

		// Detect the first angle where the robot sees a wall from the distance. 
		// wait until the distance is smaller than the threshold - noise margin - 1
		while(distance < (CaptureFlag.DISTANCE_THRESHOLD - CaptureFlag.NOISE_MARGIN-1)){
		}
		if (!firstPointDetected){
			theta1 = odometer.getTheta();
			// The second theta is detected if the distance is greater than the threshold + noise margin + 1
			if (distance >= (CaptureFlag.DISTANCE_THRESHOLD + CaptureFlag.NOISE_MARGIN + 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			// the first angle is the average of the two thetas
			firstAngle = (theta1 + theta2) / 2;
			firstPointDetected = true;
		}
		//stop the motors and make it turn the opposite direction
		na.motorStop();
		na.makeTurn(-360, false, true);

		try {
			Thread.sleep(1500);
		} catch (Exception e) {
		}

		// Detect the second angle where the robot sees a wall from the distance. 
		// wait until the distance is smaller than the threshold - noise margin - 1
		while(distance < (CaptureFlag.DISTANCE_THRESHOLD - CaptureFlag.NOISE_MARGIN-1)){
		}
		if(!secondPointDetected){
			theta1 = odometer.getTheta();
			// wait until the distance is greater than the threshold + noise margin + 1
			if (distance >= (CaptureFlag.DISTANCE_THRESHOLD + CaptureFlag.NOISE_MARGIN + 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			// the second angle is the average of the two thetas
			secondAngle = (theta1 + theta2) / 2;
			secondPointDetected = true;
		}
		na.motorStop();
		correctAngleAndLocation();
	}

	/**
	 * This method gets the two turning angles either from the falling edge mode or the rising edge mode. 
	 * First it measures the approximate location of the robot. This measurements are applied in the main class
	 * to navigate the robot to the point (0,0).
	 * Then it calculates the corrected angle according to the two turning angles and the chosen mode
	 * Finally, it set the odometer Theta to be the calculated value and turn the robot to the 0 degree direction. 
	 *
	 */
	public void correctAngleAndLocation(){		  	
		firstAngle = ((firstAngle % 360) + 360) % 360;
		secondAngle = ((secondAngle % 360) + 360) % 360;

		if (choice == 0) {
			correctedTheta = 45 - ((firstAngle + secondAngle) / 2);
		} else {
			correctedTheta = 225 - ((firstAngle + secondAngle) / 2);
		}

		// Get the current odometer reading theta, add its value to the calculated theta correction. Then set the result
		// to the odometer. 
		double t = odometer.getTheta();
		correctedTheta += t;
		correctedTheta = ((correctedTheta % 360) + 360) % 360;
		odometer.setTheta(correctedTheta);	
		na.makeTurn(-correctedTheta, true, false);
	}

	/**
	 * This method returns the value of the distance.
	 * The if statement is there to put a maximum in the distance value in case the US sensor
	 * does not detect anything. This is done to help us collect and graph distance data
	 * since if the sensor does not detect anything it will return a value greater than 200,000 which is obviously incorrect.
	 * The maximum of the distance value is not used for anything else so this is not "hard-coding" to control the robot movement.
	 * @return 	distance value
	 */
	public int readUSDistance() {
		if (this.distance > 300)
			this.distance = 300;
		return this.distance;
	}
}