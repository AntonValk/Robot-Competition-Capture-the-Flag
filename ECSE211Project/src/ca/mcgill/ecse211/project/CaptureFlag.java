package ca.mcgill.ecse211.project;

/** Main class
 * 	@author Borui Tao
 * 	@version 1.0
 * 
 */

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class is the main one and interacts with the other classes to perform the localization, navigation
 * and zipline traversal.
 */
public class CaptureFlag {


	//Set these as appropriate for your team and current situation
	private static final String SERVER_IP = "192.168.2.3";
	private static final int TEAM_NUMBER = 8;

	//Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;


	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final EV3LargeRegulatedMotor ziplineMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	private static final Port usPort = LocalEV3.get().getPort("S1");

	private static UltrasonicLocalizer ultraLoc;
	private static LightLocalizer lightLoc;
	public static float lightValue;

	/**
	 * The speed at which the robot is traveling straight.
	 */
	public static final int FORWARD_SPEED = 140;
	/**
	 * The distance between the Ultrasonic sensor and the back light sensor
	 */
	public static final double ROBOT_LENGTH = 10.2;
	/**
	 * The speed at which the robot is rotating.
	 */
	public static final int ROTATE_SPEED = 110;
	/**
	 * The radius of the wheel.
	 */
	public static final double RADIUS = 2.12;
	/**
	 * The distance between the wheels.
	 */
	public static final double TRACK = 11.3;
	/**
	 * The distance from which the ultrasonic sensor detects the wall.
	 */
	public static final int DISTANCE_THRESHOLD = 50;
	/**
	 * The noise margin.
	 */
	public static final int NOISE_MARGIN = 1;
	/**
	 * The length of the square tile.
	 */
	public static final double SQUARE_LENGTH = 30.48;
	/**
	 * The speed at which the robot is traveling when mounting the zipline.
	 */
	public static final int ZIPLINE_SPEED = 300;
	/**
	 * The length of the zipline.
	 */
	public static final double ZIPLENGTH =70;
	/**
	 * The coordinates for the different corners.
	 */
	public static final int[][] CORNERS = new int[][] {
		{1,1,0},
		{7,1,270},
		{7,7,180},
		{1,7,90}
	};

	/**
	 * Using the helper classes, the main method takes care of the localization, navigation and river traversal.
	 * 
	 */
	@SuppressWarnings("rawtypes")
	public static void main(String[] args) {

//		int redTeam = 0, greenTeam = 0;
//		int redCorner = 3, greenCorner = 1;
//		int og = 1, or = 1;
//		int red_ll_x = 0, red_ll_y = 0;
//		int red_ur_x = 0, red_ur_y = 0;
//		int green_ll_x = 0, green_ll_y = 0;
//		int green_ur_x = 0, green_ur_y = 0;
//		int zc_r_x = 4, zc_r_y = 9;
//		int zo_r_x = 3, zo_r_y = 10;
//		int zc_g_x = 8, zc_g_y = 3;
//		int zo_g_x = 9, zo_g_y = 2;
//		int sh_ll_x = 8, sh_ll_y = 9;
//		int sh_ur_x = 11, sh_ur_y = 10;
//		int sv_ll_x = 10, sv_ll_y = 5;
//		int sv_ur_x = 11, sv_ur_y = 10;
//		int sr_ll_x = 1, sr_ll_y = 9;
//		int sr_ur_x = 2, sr_ur_y = 11;
//		int sg_ll_x = 9, sg_ll_y = 1;
//		int sg_ur_x = 11, sg_ur_y = 2;



		int redTeam = 0, greenTeam = 0;
		int redCorner = 0, greenCorner = 0;
		int og = 0, or = 0;
		int red_ll_x = 0, red_ll_y = 0;
		int red_ur_x = 0, red_ur_y = 0;
		int green_ll_x = 0, green_ll_y = 0;
		int green_ur_x = 0, green_ur_y = 0;
		int zc_r_x = 0, zc_r_y = 0;
		int zo_r_x = 0, zo_r_y = 0;
		int zc_g_x = 0, zc_g_y = 0;
		int zo_g_x = 0, zo_g_y = 0;
		int sh_ll_x = 0, sh_ll_y = 0;
		int sh_ur_x = 0, sh_ur_y = 0;
		int sv_ll_x = 0, sv_ll_y = 0;
		int sv_ur_x = 0, sv_ur_y = 0;
		int sr_ll_x = 0, sr_ll_y = 0;
		int sr_ur_x = 0, sr_ur_y = 0;
		int sg_ll_x = 0, sg_ll_y = 0;
		int sg_ur_x = 0, sg_ur_y = 0;
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might occur
		try {
			Map data = conn.getData();

			// Example 1: Print out all received data
			System.out.println("Map:\n" + data);

			redTeam = ((Long) data.get("RedTeam")).intValue();
			System.out.println("Red Team: " + redTeam);
			greenTeam = ((Long) data.get("GreenTeam")).intValue();
			System.out.println("Green Team: " + greenTeam);

			redCorner = ((Long) data.get("RedCorner")).intValue();
			System.out.println("Red Corner: " + redCorner);
			greenCorner = ((Long) data.get("GreenCorner")).intValue();
			System.out.println("Green Corner: " + greenCorner);

			og = ((Long) data.get("OG")).intValue();
			System.out.println("Green opponent flag: " + og);
			or = ((Long) data.get("OR")).intValue();
			System.out.println("Red opponent flag: " + or);

			red_ll_x =  ((Long) data.get("Red_LL_x")).intValue();
			System.out.println("Data: " + red_ll_x);
			red_ll_y =  ((Long) data.get("Red_LL_y")).intValue();
			System.out.println("Data: " + red_ll_y);
			red_ur_x =  ((Long) data.get("Red_UR_x")).intValue();
			System.out.println("Data: " + red_ur_x);
			red_ur_y =  ((Long) data.get("Red_UR_y")).intValue();
			System.out.println("Data: " + red_ur_y);
			green_ll_x =  ((Long) data.get("Green_LL_x")).intValue();
			System.out.println("Data: " + green_ll_x);
			green_ll_y =  ((Long) data.get("Green_LL_y")).intValue();
			System.out.println("Data: " + green_ll_y);
			green_ur_x =  ((Long) data.get("Green_UR_x")).intValue();
			System.out.println("Data: " + green_ur_x);
			green_ur_y =  ((Long) data.get("Green_UR_y")).intValue();
			System.out.println("Data: " + green_ur_y);
			zc_r_x =  ((Long) data.get("ZC_R_x")).intValue();
			System.out.println("ZC_R_x: " + zc_r_x);
			zc_r_y =  ((Long) data.get("ZC_R_y")).intValue();
			System.out.println("ZC_R_y: " + zc_r_y);
			zo_r_x =  ((Long) data.get("ZO_R_x")).intValue();
			System.out.println("ZO_R_x" + zo_r_x);
			zo_r_y =  ((Long) data.get("ZO_R_y")).intValue();
			System.out.println("ZO_R_y: " + zo_r_y);
			zc_g_x =  ((Long) data.get("ZC_G_x")).intValue();
			System.out.println("ZC_G_x: " + zc_g_x);
			zc_g_y =  ((Long) data.get("ZC_G_y")).intValue();
			System.out.println("ZC_G_y" + zc_g_y);
			zo_g_x =  ((Long) data.get("ZO_G_x")).intValue();
			System.out.println("ZO_G_x: " + zo_g_x);
			zo_g_y =  ((Long) data.get("ZO_G_y")).intValue();
			System.out.println("ZO_G_y: " + zo_g_y);
			sh_ll_x =  ((Long) data.get("SH_LL_x")).intValue();
			System.out.println("Data: " + sh_ll_x);
			sh_ll_y =  ((Long) data.get("SH_LL_y")).intValue();
			System.out.println("Data: " + sh_ll_y);
			sh_ur_x =  ((Long) data.get("SH_UR_x")).intValue();
			System.out.println("Data: " + sh_ur_x);
			sh_ur_y =  ((Long) data.get("SH_UR_y")).intValue();
			System.out.println("Data: " + sh_ur_y);
			sv_ll_x =  ((Long) data.get("SV_LL_x")).intValue();
			System.out.println("Data: " + sv_ll_x);
			sv_ll_y =  ((Long) data.get("SV_LL_y")).intValue();
			System.out.println("Data: " + sv_ll_y);
			sv_ur_x =  ((Long) data.get("SV_UR_x")).intValue();
			System.out.println("Data: " + sv_ur_x);
			sv_ur_y =  ((Long) data.get("SV_UR_y")).intValue();
			System.out.println("Data: " + sv_ur_y);
			sr_ll_x =  ((Long) data.get("SR_LL_x")).intValue();
			System.out.println("Data: " + sr_ll_x);
			sr_ll_y =  ((Long) data.get("SR_LL_y")).intValue();
			System.out.println("Data: " + sr_ll_y);
			sr_ur_x =  ((Long) data.get("SR_UR_x")).intValue();
			System.out.println("Data: " + sr_ur_x);
			sr_ur_y =  ((Long) data.get("SR_UR_y")).intValue();
			System.out.println("Data: " + sr_ur_y);
			sg_ll_x =  ((Long) data.get("SG_LL_x")).intValue();
			System.out.println("Data: " + sg_ll_x);
			sg_ll_y =  ((Long) data.get("SG_LL_y")).intValue();
			System.out.println("Data: " + sg_ll_y);
			sg_ur_x =  ((Long) data.get("SG_UR_x")).intValue();
			System.out.println("Data: " + sg_ur_x);
			sg_ur_y =  ((Long) data.get("SG_UR_y")).intValue();
			System.out.println("Data: " + sg_ur_y);
		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}

		// clear the display
		@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[1];		// usData is the buffer in which data are returned
		UltrasonicPoller usPoller = null;									// the selected controller on each cycle


		//two classes to perform wall following on the flags and detect the color of each one.
		//BangBangController bb = new BangBangController(5,1,FORWARD_SPEED,ROTATE_SPEED,leftMotor,rightMotor);
		LightSensor ls = new LightSensor();

		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		Navigation na = new Navigation(odometer,leftMotor, rightMotor, ziplineMotor);

		
	//	Button.waitForAnyPress();

		ultraLoc = new UltrasonicLocalizer(na, odometer);

		odometer.start();
		OdometryDisplay od = new OdometryDisplay(odometer, t,ultraLoc);
		od.start();
	//	usPoller = new UltrasonicPoller(usDistance, usData, ultraLoc, bb);
		usPoller = new UltrasonicPoller(usDistance, usData, ultraLoc);
		usPoller.start();
		
		ultraLoc.doUltrasonicLocalization();
		
		while(na.isNavigating()){
		}
		
		try { 
			Thread.sleep(250);
		} catch (Exception e) {
		}
		lightLoc = new LightLocalizer(odometer, na);
		double x = ultraLoc.getLocX();
		double y = ultraLoc.getLocY();
		
		//this method make the robot close to the actual (0,0) point
		lightLoc.adjustPosition();
		lightLoc.doLightLocalization();
		while(na.isNavigating()){
		}

		if (TEAM_NUMBER == redTeam){
			odometer.setX(CORNERS[redCorner][0]*SQUARE_LENGTH);
			odometer.setY(CORNERS[redCorner][1]*SQUARE_LENGTH);
			odometer.setTheta(CORNERS[redCorner][2]);
			
		//	na.travelTo(CORNERS[redCorner][0]*SQUARE_LENGTH, zo_r_y*SQUARE_LENGTH);
			na.travelTo(zo_r_x*SQUARE_LENGTH, zo_r_y*SQUARE_LENGTH);
			na.makeTurn(-20, false, true);
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();

			while(na.isNavigating()){

			}
			odometer.setX(zo_r_x*SQUARE_LENGTH);
			odometer.setY(zo_r_y*SQUARE_LENGTH);
			odometer.setTheta(CORNERS[redCorner][2]);
			
			na.travelTo(zc_r_x*SQUARE_LENGTH, zc_r_y*SQUARE_LENGTH);
			double ziplineTheta = Math.atan(Math.abs(zc_r_y-zc_g_y)/Math.abs(zc_r_x-zc_g_x))*360.0/(2*Math.PI);
			na.turnTo(CORNERS[redCorner][2]+ziplineTheta);
			na.doZipline(3*ZIPLENGTH);
			while(na.isNavigating()){

			}
			
			odometer.setX(zc_g_x*SQUARE_LENGTH);
			odometer.setY(zc_g_y*SQUARE_LENGTH);
			odometer.setTheta(CORNERS[redCorner][2]+ziplineTheta);
			
			//arrive at the flag zone
			na.travelTo(zo_g_x*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			odometer.setX(zo_g_x*SQUARE_LENGTH);
			odometer.setY(zo_g_y*SQUARE_LENGTH);
			odometer.setTheta(CORNERS[redCorner][2]);
			
			na.travelTo(sg_ur_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
			na.travelTo(sg_ur_x*SQUARE_LENGTH, sg_ll_y*SQUARE_LENGTH);
			na.travelTo(sg_ll_x*SQUARE_LENGTH, sg_ll_y*SQUARE_LENGTH);
			na.travelTo(sg_ll_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
			na.travelTo(sg_ur_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
			
		}
		else if (TEAM_NUMBER == greenTeam){
			odometer.setX(CORNERS[greenCorner][0]*SQUARE_LENGTH);
			odometer.setY(CORNERS[greenCorner][1]*SQUARE_LENGTH);
			odometer.setTheta(CORNERS[greenCorner][2]);
			
//			if (!(zo_g_x == CORNERS[greenCorner][0]) && !(zo_g_y == CORNERS[greenCorner][1]))
//					na.travelTo(CORNERS[greenCorner][0]*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
			boolean gotToDestination = false;
			if(CORNERS[greenCorner][0] != zo_g_x){
				while(!gotToDestination){
					gotToDestination = na.travelTo(CORNERS[greenCorner][0]*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
					na.securityTurn();
					lightLoc = new LightLocalizer(odometer, na);
					lightLoc.midpointLocalization();
					odometer.setX(na.currentX);
					odometer.setY(na.currentY);
					odometer.setTheta(lightLoc.currentTheta);
				}
				gotToDestination = false;
			}
			while(!gotToDestination){
				gotToDestination = na.travelTo(zo_g_x*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
				double cTheta = odometer.getTheta();
				na.securityTurn();
				lightLoc = new LightLocalizer(odometer, na);
				lightLoc.midpointLocalization();
				odometer.setX(na.currentX);
				odometer.setY(na.currentY);
				odometer.setTheta(lightLoc.currentTheta);
			}
			
			System.out.println("x before zipline: " + odometer.getX());
			System.out.println("y before zipline: " + odometer.getY());
			System.out.println("Theta before zipline: " + odometer.getTheta());
			
			na.doZipline(zc_g_x * SQUARE_LENGTH, zc_g_y * SQUARE_LENGTH, 3*ZIPLENGTH);
			while(na.onZipline() || na.isNavigating()){

			}
			na.motorStop();
//			Button.waitForAnyPress();
			
//			odometer.setX(zc_r_x*SQUARE_LENGTH);
//			odometer.setY(zc_r_y*SQUARE_LENGTH);
//			odometer.setTheta(CORNERS[greenCorner][2]+ziplineTheta);
			
//			na.travelTo(zo_r_x*SQUARE_LENGTH, zo_r_y*SQUARE_LENGTH);
			na.securityTurn();
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){

			}
			odometer.setX(zo_r_x*SQUARE_LENGTH);
			odometer.setY(zo_r_y*SQUARE_LENGTH);
			odometer.setTheta(CORNERS[greenCorner][2]);

			
			gotToDestination = false;
			if(zo_r_x != sr_ur_x){
				while(!gotToDestination){
					gotToDestination = na.travelTo(zo_r_x*SQUARE_LENGTH, sr_ur_y*SQUARE_LENGTH);
					na.securityTurn();
					lightLoc.midpointLocalization();
					odometer.setX(na.currentX);
					odometer.setY(na.currentY);
					odometer.setTheta(lightLoc.currentTheta);
				}
			}
			while(!gotToDestination){
				gotToDestination = na.travelTo(sr_ur_x*SQUARE_LENGTH, sr_ur_y*SQUARE_LENGTH);
				na.securityTurn();
				lightLoc.midpointLocalization();
				odometer.setX(na.currentX);
				odometer.setY(na.currentY);
				odometer.setTheta(lightLoc.currentTheta);
			}
			
			
			na.travelTo(sr_ur_x*SQUARE_LENGTH, sr_ur_y*SQUARE_LENGTH);
			while(na.isNavigating()){

			}
			double curTheta = odometer.getTheta();
			na.securityTurn();
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){

			}
			odometer.setX(sr_ur_x*SQUARE_LENGTH);
			odometer.setY(sr_ur_y*SQUARE_LENGTH);
			odometer.setTheta(curTheta-90);
			
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
