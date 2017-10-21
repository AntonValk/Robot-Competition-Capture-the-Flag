package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ZiplineLab {

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
	public static final int[][] CORNERS = new int[][] {
		{1,1,0},
		{7,1,270},
		{7,7,180},
		{1,7,90}
	}; // the coordinates of corners

	
	public static void main(String[] args) {
		int buttonChoice;
		int cornerCounter = 0;
		int bestCorner = 0;
		int xoCounter = 0;
		int yoCounter = 0;
		int xcCounter = 0;
		int ycCounter = 0;
		

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
			t.drawString("< Please select  >", 0, 0);
			t.drawString("corner number in  ", 0, 1);
			t.drawString("the range [0,3]:  ", 0, 2);
			t.drawString(cornerCounter + "                 ", 0, 3);
			
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_LEFT){
				cornerCounter--;
				if (cornerCounter < 0) cornerCounter = 3;
			}
			if(buttonChoice == Button.ID_RIGHT){
				cornerCounter++;
				if (cornerCounter > 3) cornerCounter = 0;
			}
		} while (buttonChoice != Button.ID_ENTER);
		
		do {
			// clear the display
			t.clear();
			
			// ask the user whether the motors should drive in the rising edge mode or falling edge mode
			t.drawString("< Please select  >", 0, 0);
			t.drawString("Xo and Yo in the  ", 0, 1);
			t.drawString("range [0,8]:      ", 0, 2);
			t.drawString(" (" + xoCounter + " " + yoCounter + ") ", 0, 3);
			
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_LEFT){
				xoCounter++;
				if (xoCounter > 3) xoCounter = 0;
			}
			if(buttonChoice == Button.ID_RIGHT){
				yoCounter++;
				if (yoCounter > 3) yoCounter = 0;
			}
		} while (buttonChoice != Button.ID_ENTER);
		
		double minDis = Integer.MAX_VALUE;
		bestCorner = 4;
		for (int i = 0; i < CORNERS.length; i++){
			double dis = Math.pow(CORNERS[i][0] - xoCounter, 2) + Math.pow(CORNERS[i][1] - yoCounter, 2);
			if (dis < minDis) {
				minDis = dis;
				bestCorner = i;
			}
		}
		do {
			// clear the display
			t.clear();
			
			// ask the user whether the motors should drive in the rising edge mode or falling edge mode
			t.drawString("< Please select  >", 0, 0);
			t.drawString("Xc and Yc in the  ", 0, 1);
			t.drawString("range [0,8]:      ", 0, 2);
			t.drawString(" (" + xcCounter + " " + ycCounter + ") ", 0, 3);
			
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_LEFT){
				xcCounter++;
				if (xcCounter > 3) xcCounter = 0;
			}
			if(buttonChoice == Button.ID_RIGHT){
				ycCounter++;
				if (ycCounter > 3) ycCounter = 0;
			}
		} while (buttonChoice != Button.ID_ENTER);
		
		ultraLoc = new UltrasonicLocalizer(0, na, odometer);
		
		odometer.start();
		OdometryDisplay od = new OdometryDisplay(odometer, t,ultraLoc);
		od.start();
		usPoller = new UltrasonicPoller(usDistance, usData, ultraLoc);

		usPoller.start();
		ultraLoc.start();

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
		
		switch (cornerCounter) {
		case 0:
			odometer.setX(CORNERS[0][0]);
			odometer.setY(CORNERS[0][1]);
			odometer.setTheta(CORNERS[0][2]);
			break;
		case 1:
			odometer.setX(CORNERS[1][0]);
			odometer.setY(CORNERS[1][1]);
			odometer.setTheta(CORNERS[0][2]);
			break;
		case 2:
			odometer.setX(CORNERS[2][0]);
			odometer.setY(CORNERS[2][1]);
			odometer.setTheta(CORNERS[0][2]);
			break;
		case 3:
			odometer.setX(CORNERS[3][0]);
			odometer.setY(CORNERS[3][1]);
			odometer.setTheta(CORNERS[0][2]);
			break;
		}
		
		Button.waitForAnyPress();
		int curCorner = cornerCounter;
		while (bestCorner-curCorner > 0) {
			na.travelTo(CORNERS[curCorner+1][0], CORNERS[curCorner+1][1]);
			curCorner++;
		}
		while (bestCorner-curCorner < 0) {
			na.travelTo(CORNERS[curCorner-1][0], CORNERS[curCorner-1][1]);
			curCorner--;
		}
		
		na.travelTo(xcCounter, ycCounter);
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
	
}
