package ca.mcgill.ecse211.lab4;

/**
 * @author Antonios Valkanas, Borui Tao
 * @version 1.0
 */

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LocalizationLab {

	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final Port usPort = LocalEV3.get().getPort("S1");

	private static UltrasonicLocalizer ultraLoc;
	private static LightLocalizer lightLoc;

	//static values
	public static final int FORWARD_SPEED = 70;		  // the speed at which the robot is traveling straight
	public static final int ROTATE_SPEED = 70;		  // the speed at which the robot is rotating
	public static final double RADIUS = 2.12; 		  // radius of the wheel
	public static final double TRACK = 11.87; 		  // distance between wheels
	public static final int DISTANCE_THRESHOLD = 30;  // the distance from which the ultrasonic sensor detects the wall
	public static final int NOISE_MARGIN = 1;	  	  // the noice margin
	public static final double SQUARE_LENGTH = 30.48; // the length of the square tile
	public static void main(String[] args) {
		int buttonChoice;

		// clear the display
		@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[1];		// usData is the buffer in which data are returned
		UltrasonicPoller usPoller = null;									// the selected controller on each cycle


		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		Navigation na = new Navigation(odometer,leftMotor, rightMotor);

		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in the rising edge mode or falling edge mode
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString(" Rising|Falling ", 0, 2);
			t.drawString(" edge  | edge   ", 0, 3);
			t.drawString("       |  		", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) ultraLoc = new UltrasonicLocalizer(0, na, odometer);
		else ultraLoc = new UltrasonicLocalizer(1, na, odometer);

		odometer.start();
		OdometryDisplay od = new OdometryDisplay(odometer, t,ultraLoc);
		od.start();
		usPoller = new UltrasonicPoller(usDistance, usData, ultraLoc);

		usPoller.start();
		ultraLoc.start();

		Button.waitForAnyPress();

		try { 
			Thread.sleep(1000);
		} catch (Exception e) {
		}

		lightLoc = new LightLocalizer(odometer, na);
		double x = ultraLoc.getLocX();
		double y = ultraLoc.getLocY();

		/**
		 * Set the x and y to be measured location and make the robot travel to the point (0,0)
		 * Now the robot should roughly be at the point (0,0). Then we start the light localization 
		 * which navigates the robot to the exact (0,0) point.
		 * 
		 */
		odometer.setX(x-SQUARE_LENGTH);
		odometer.setY(y-SQUARE_LENGTH);
		na.travelTo(0, 0);

		//Wait until the robot stops moving, start the lightLocalization thread
		while(na.isNavigating()){
		}
		lightLoc.start();

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
