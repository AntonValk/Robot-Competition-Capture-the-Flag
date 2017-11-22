package ca.mcgill.ecse211.project;

import java.util.Arrays;
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
	/**
	 * The server IP for the computer "controlling" the robot.
	 */
	private static final String SERVER_IP = "192.168.2.20";
	
	/**
	 * The team number.
	 */
	private static final int TEAM_NUMBER = 8;

	//Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	/**
	 * The pointer to the left motor.
	 */
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	/**
	 * The pointer to the right motor.
	 */
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	/**
	 * The pointer to the zipline motor.
	 */
	private static final EV3LargeRegulatedMotor ziplineMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	/**
	 * The pointer to the motor connected to the ultrasonic sensor (to turn it 45 degrees when detecting a flag).
	 */
	private static final EV3LargeRegulatedMotor ultrasonicMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	/**
	 * The pointer to the ultrasonic sensor.
	 */
	private static final Port usPort = LocalEV3.get().getPort("S1");

	/**
	 * The ultrasonic localizer.
	 */
	private static UltrasonicLocalizer ultraLoc;
//	private static BangBangController bb;

	/**
	 * The light localizer.
	 */
	private static LightLocalizer lightLoc;
	/**
	 * The value got by the front light sensor an updated by the LightSensor class.
	 */
	public static float lightValue;
	/**
	 * The speed at which the robot is traveling straight.
	 */
	public static final int FORWARD_SPEED = 170; //140
	/**
	 * The distance between the Ultrasonic sensor and the back light sensor.
	 */
	public static final double ROBOT_LENGTH = 10.2;
	/**
	 * The speed at which the robot is rotating.
	 */
	public static final int ROTATE_SPEED = 130; // 110
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
		{11,1,270},
		{11,11,180},
		{1,11,90}
	};

	/**
	 * Using the helper classes, the main method takes care of the localization, navigation and river traversal.
	 * 
	 */
	//@SuppressWarnings("rawtypes")
	public static void main(String[] args) {

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
		//@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[1];		// usData is the buffer in which data are returned
		UltrasonicPoller usPoller = null;									// the selected controller on each cycle


		//two classes to perform wall following on the flags and detect the color of each one.
		LightSensor ls = new LightSensor();

		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		Navigation na = new Navigation(odometer,leftMotor, rightMotor, ziplineMotor, ultrasonicMotor);
	//	bb = new BangBangController(na,odometer, 5,1,FORWARD_SPEED,ROTATE_SPEED,leftMotor,rightMotor);

		ultraLoc = new UltrasonicLocalizer(na, odometer, leftMotor, rightMotor);

		odometer.start();
		OdometryDisplay od = new OdometryDisplay(odometer, t,ultraLoc);
		od.start();
	//	usPoller = new UltrasonicPoller(usDistance, usData, ultraLoc, bb);
		usPoller = new UltrasonicPoller(usDistance, usData, ultraLoc);
		usPoller.start();
		
		//test zipline
		/*odometer.setX(SQUARE_LENGTH);
		odometer.setY(SQUARE_LENGTH);
		odometer.setTheta(0);
		//na.doZipline(zc_g_x * SQUARE_LENGTH, zc_g_y * SQUARE_LENGTH, 3*ZIPLENGTH);
		na.doZipline(1 * SQUARE_LENGTH, 2 * SQUARE_LENGTH, 3*ZIPLENGTH);
		leftMotor.stop();
		rightMotor.stop();
		Button.waitForAnyPress();*/
		
		//test bangbang
		/*System.out.println("start bangbang");
		ls.start();
		ultraLoc.doBangBang(10,2,50,100);
		Button.waitForAnyPress();*/
		
		//for test purpose, the robot will go to (1, 3), (3, 3), (3, 1), (1,1)		
		//ultraLoc.doGridTraversal(SQUARE_LENGTH,SQUARE_LENGTH,SQUARE_LENGTH, 3*SQUARE_LENGTH, 2);
		//ultraLoc.doGridTraversal(SQUARE_LENGTH,3*SQUARE_LENGTH,3*SQUARE_LENGTH, 3*SQUARE_LENGTH, 2);
		//ultraLoc.doGridTraversal(3*SQUARE_LENGTH,3*SQUARE_LENGTH,3*SQUARE_LENGTH, SQUARE_LENGTH, 2);
		//ultraLoc.doGridTraversal(3*SQUARE_LENGTH,SQUARE_LENGTH,SQUARE_LENGTH, SQUARE_LENGTH, 2);

		
		//comment for testing purpose
		ultraLoc.doUltrasonicLocalization();
		
		while(na.isNavigating()){
		}
		lightLoc = new LightLocalizer(odometer, na);
		double x = ultraLoc.getLocX();
		double y = ultraLoc.getLocY();
		
		//this method make the robot close to the actual (0,0) point
		lightLoc.adjustPosition();
		lightLoc.doLightLocalization();
		while(na.isNavigating()){
		}
		
		

//		if (TEAM_NUMBER == redTeam){
//			odometer.setX(CORNERS[redCorner][0]*SQUARE_LENGTH);
//			odometer.setY(CORNERS[redCorner][1]*SQUARE_LENGTH);
//			odometer.setTheta(CORNERS[redCorner][2]);
//			
//			na.travelTo(zo_r_x*SQUARE_LENGTH, CORNERS[redCorner][1]*SQUARE_LENGTH);
//			na.travelTo(zo_r_x*SQUARE_LENGTH, zo_r_y*SQUARE_LENGTH);
//
//			na.travelTo((sh_ll_x - 1)*SQUARE_LENGTH,(sh_ll_y + 1)*SQUARE_LENGTH);
//    		lightLoc.doLightLocalization();
//			na.makeTurn(90, false, true);
//			odometer.setX((sh_ll_x - 1)*SQUARE_LENGTH);
//			odometer.setY((sh_ll_y + 1)*SQUARE_LENGTH);
//			
//			//shadow crossing
//			na.travelTo(sh_ll_x*SQUARE_LENGTH,(sh_ll_y*SQUARE_LENGTH + 0.5*SQUARE_LENGTH));
//			na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH, (sh_ur_y-0.5)*SQUARE_LENGTH);
//			na.travelTo((sv_ll_x+ 0.5*SQUARE_LENGTH), sv_ll_y);
//			//GO BACK TO START ZONE
//			na.travelTo((sv_ll_x+ 1)*SQUARE_LENGTH, (sv_ll_y-1)*SQUARE_LENGTH);
//    		lightLoc.doLightLocalization();
//			na.makeTurn(90, false, true);
//			odometer.setX((sh_ll_x + 1)*SQUARE_LENGTH);
//			odometer.setY((sh_ll_y - 1)*SQUARE_LENGTH);
//    		
//			na.travelTo(sg_ur_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
//			//TODO:search for flag
//			
//			//GO BACK TO START ZONE - NOT USED FOR BETA DEMO
//			//na.travelTo(sg_ur_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
//
//			//SHALLOW CROSSING - NOT USED FOR BETA DEMO
//			/*
//			na.travelTo(sh_ll_x*SQUARE_LENGTH,(sh_ll_y*SQUARE_LENGTH + 0.5*SQUARE_LENGTH));
//			na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH, (sh_ur_y-0.5)*SQUARE_LENGTH);
//			na.travelTo((sv_ll_x+ 0.5*SQUARE_LENGTH), sv_ll_y);
//			 */
//
//			//FLAG SEARCH -  NOT USED FOR BETA DEMO
//			/*
//			ls.start();
//			bb.start();
//			while(true){
//				if(lightValue == colorValueOf_or){
//					Sound.playNote(Sound.FLUTE, 440, 250);
//					Sound.playNote(Sound.FLUTE, 440, 250);
//					Sound.playNote(Sound.FLUTE, 440, 250);
//					break;
//				}
//			}
//			 */
//
//			na.travelTo(zo_g_x*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
//			while(na.isNavigating()){
//
//			}
//		na.turnTo(-Math.atan(4/5)*360.0/(2*Math.PI)+360);
//			na.doZipline(3*ZIPLENGTH);
//			while(na.isNavigating()){
//
//			}
//			odometer.setX(zc_r_x*SQUARE_LENGTH);
//			odometer.setY(zc_r_y*SQUARE_LENGTH);
//			odometer.setTheta(-Math.atan((4/5)*360.0/(2*Math.PI)));
//			
//			na.travelTo(zo_r_x*SQUARE_LENGTH, zo_r_y*SQUARE_LENGTH);
//			lightLoc.doLightLocalization();
//			na.makeTurn(90, false, true);
//			odometer.setX(zo_r_x*SQUARE_LENGTH);
//			odometer.setY(zo_r_y*SQUARE_LENGTH);
//			odometer.setTheta(0);
//			
//			na.travelTo(zo_r_y*SQUARE_LENGTH, CORNERS[redCorner-1][1]*SQUARE_LENGTH);
//
//			na.travelTo(CORNERS[redCorner-1][0]*SQUARE_LENGTH, CORNERS[redCorner-1][1]*SQUARE_LENGTH);			
//		}
		if (TEAM_NUMBER == redTeam){
			/*odometer.setX(CORNERS[redCorner][0]*SQUARE_LENGTH);
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
			odometer.setTheta(CORNERS[redCorner][2]);*/
			
			
			//SHALLOW CROSSING
			odometer.setX(SQUARE_LENGTH);
			odometer.setY(SQUARE_LENGTH);
			odometer.setTheta(0);
			
			if(odometer.getX() < sh_ll_x*SQUARE_LENGTH){
				if(odometer.getY() > sv_ll_y*SQUARE_LENGTH){
					//hor right - ver down
					na.travelTo((sh_ll_x)*SQUARE_LENGTH,(sh_ll_y+0.5)*SQUARE_LENGTH);
					na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
					na.travelTo((sv_ur_x-0.5)*SQUARE_LENGTH,(sv_ur_y-0.5)*SQUARE_LENGTH);
					na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH,(sv_ll_y)*SQUARE_LENGTH);
					na.travelTo((sv_ll_x*SQUARE_LENGTH)-5,(sv_ll_y-1)*SQUARE_LENGTH);
				}else{
					//hor right - ver up
					na.travelTo((sh_ll_x)*SQUARE_LENGTH,(sh_ll_y+0.5)*SQUARE_LENGTH);
					na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
					na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH,(sv_ll_y+0.5)*SQUARE_LENGTH);
					na.travelTo((sv_ur_x-0.5)*SQUARE_LENGTH,(sv_ur_y)*SQUARE_LENGTH);
					na.travelTo((sv_ur_x-1)*SQUARE_LENGTH,(sv_ur_y+1)*SQUARE_LENGTH);
				}
			}else{
				if(odometer.getY() > sv_ll_y*SQUARE_LENGTH){
					//hor left - ver down
					na.travelTo((sh_ur_x)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
					na.travelTo((sh_ll_x+0.5)*SQUARE_LENGTH,(sh_ll_y+0.5)*SQUARE_LENGTH);
					na.travelTo((sv_ur_x-0.5)*SQUARE_LENGTH,(sv_ur_y-0.5)*SQUARE_LENGTH);
					na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH, sv_ll_y);
					na.travelTo((sv_ll_x*SQUARE_LENGTH)-5,(sv_ll_y-1)*SQUARE_LENGTH);
				}else{
					//hor left - ver up
					na.travelTo((sh_ur_x)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
					na.travelTo((sh_ll_x+0.5)*SQUARE_LENGTH, (sh_ll_y+0.5)*SQUARE_LENGTH);
					na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH, (sv_ll_y+0.5)*SQUARE_LENGTH);
					na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH, (sv_ur_y)*SQUARE_LENGTH);
					na.travelTo((sv_ur_x*SQUARE_LENGTH)+5,(sv_ur_y+1)*SQUARE_LENGTH);
				}
			}
			na.securityTurn();
			lightLoc.midpointLocalization();
			odometer.setX(na.currentX);
			odometer.setY(na.currentY);
			odometer.setTheta(lightLoc.currentTheta);
			System.out.println("X: " + odometer.getX());
			System.out.println("Y: " + odometer.getY());
			System.out.println("Theta: " + odometer.getTheta());
			leftMotor.stop();
			rightMotor.stop();
			Button.waitForAnyPress();
			//
			
			
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
			
//    		na.travelTo((sh_ll_x - 1)*SQUARE_LENGTH,(sh_ll_y + 1)*SQUARE_LENGTH);
//			lightLoc = new LightLocalizer(odometer, na);
//    		lightLoc.doLightLocalization();
//			na.makeTurn(90, false, true);
//			odometer.setX((sh_ll_x - 1)*SQUARE_LENGTH);
//			odometer.setY((sh_ll_y + 1)*SQUARE_LENGTH);
//			
//			//cross shallow
//    		na.travelTo(sh_ll_x*SQUARE_LENGTH,(sh_ll_y*SQUARE_LENGTH + 0.5*SQUARE_LENGTH));
//			na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH, (sh_ur_y-0.5)*SQUARE_LENGTH);
//			na.travelTo((sv_ll_x+ 0.5)*SQUARE_LENGTH, sv_ll_y*SQUARE_LENGTH);
//			//GO BACK TO START ZONE
//			
//			na.travelTo((sv_ll_x+ 1)*SQUARE_LENGTH, (sv_ll_y-1)*SQUARE_LENGTH);
//			lightLoc = new LightLocalizer(odometer, na);
//    		lightLoc.doLightLocalization();
//			na.makeTurn(90, false, true);
//			odometer.setX((sh_ll_x + 1)*SQUARE_LENGTH);
//			odometer.setY((sh_ll_y - 1)*SQUARE_LENGTH);
//    		
//			na.travelTo(sg_ur_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
//			
//			na.travelTo(CORNERS[greenCorner-1][0]*SQUARE_LENGTH, CORNERS[greenCorner-1][1]*SQUARE_LENGTH);
		}
		else if (TEAM_NUMBER == greenTeam){
			odometer.setX(CORNERS[greenCorner][0]*SQUARE_LENGTH);
			odometer.setY(CORNERS[greenCorner][1]*SQUARE_LENGTH);
			odometer.setTheta(CORNERS[greenCorner][2]);
		
			boolean gotToDestination = false;
			if(CORNERS[greenCorner][0] != zo_g_x && CORNERS[greenCorner][1] != zo_g_y){
				if (Math.abs(zo_g_x-CORNERS[greenCorner][0]) > Math.abs(zo_g_y-CORNERS[greenCorner][1])){
					while(!gotToDestination){
						gotToDestination = na.travelTo(zo_g_x*SQUARE_LENGTH, CORNERS[greenCorner][1]*SQUARE_LENGTH);
						na.securityTurn();
						lightLoc = new LightLocalizer(odometer, na);
						lightLoc.midpointLocalization();
						na.makeTurn(5, false,false);
						System.out.println("lightLoc1");
						odometer.setX(na.currentX);
						odometer.setY(na.currentY);
						odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
				}else{
					while(!gotToDestination){
						gotToDestination = na.travelTo(CORNERS[greenCorner][0]*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
						na.securityTurn();
						lightLoc = new LightLocalizer(odometer, na);
						lightLoc.midpointLocalization();
						na.makeTurn(5, false,false);
						System.out.println("lightLoc1");
						odometer.setX(na.currentX);
						odometer.setY(na.currentY);
						odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
				}
			}
			while(!gotToDestination){
				gotToDestination = na.travelTo(zo_g_x*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
				double cTheta = odometer.getTheta();
				na.securityTurn();
				lightLoc = new LightLocalizer(odometer, na);
				lightLoc.midpointLocalization();
				if(!gotToDestination) {
					na.makeTurn(5, false,true);
					while(na.isNavigating()){
						
					}
				}
				System.out.println("lightLoc2");
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
			
			na.securityTurn();
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){
				
			}
			odometer.setX(zo_r_x*SQUARE_LENGTH);
			odometer.setY(zo_r_y*SQUARE_LENGTH);
			double th = odometer.getTheta();
			if(Math.abs(th) < 15) odometer.setTheta(0);
			else if (Math.abs(th-90) < 15) odometer.setTheta(90);
			else if (Math.abs(th-180) < 15) odometer.setTheta(180);
			else if (Math.abs(th-270) < 15) odometer.setTheta(270);
			System.out.println("T is " + th);
			
			gotToDestination = false;
			if(zo_r_x != sr_ur_x && zo_r_y != sr_ur_y){
				if (zo_r_x != zc_r_x){
					while(!gotToDestination){
						gotToDestination = na.travelTo(zo_r_x*SQUARE_LENGTH, sr_ur_y*SQUARE_LENGTH);
						na.securityTurn();
						lightLoc = new LightLocalizer(odometer, na);
						lightLoc.midpointLocalization();
						if(!gotToDestination) {
							na.makeTurn(5, false,true);
							while(na.isNavigating()){
								
							}
						}
						odometer.setX(na.currentX);
						odometer.setY(na.currentY);
						odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
				}else if(zo_r_y != zc_r_y){
					while(!gotToDestination){
						gotToDestination = na.travelTo(sr_ur_x*SQUARE_LENGTH, zo_r_y*SQUARE_LENGTH);
						na.securityTurn();
						lightLoc = new LightLocalizer(odometer, na);
						lightLoc.midpointLocalization();
						if(!gotToDestination) {
							na.makeTurn(5, false,true);
							while(na.isNavigating()){
								
							}
						}
						odometer.setX(na.currentX);
						odometer.setY(na.currentY);
						odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
				}
			}
			while(!gotToDestination){
				gotToDestination = na.travelTo(sr_ur_x*SQUARE_LENGTH, sr_ur_y*SQUARE_LENGTH);
				na.securityTurn();
				lightLoc = new LightLocalizer(odometer, na);
				lightLoc.midpointLocalization();
				if(!gotToDestination) {
					na.makeTurn(5, false,true);
					while(na.isNavigating()){
						
					}
				}
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
			
			//FLAG SEARCH
			
			
			
//			ls.start();
//			bb.start();
//			while(true){
//				if(lightValue == og){
//					Sound.playNote(Sound.FLUTE, 440, 250);
//					Sound.playNote(Sound.FLUTE, 440, 250);
//					Sound.playNote(Sound.FLUTE, 440, 250);
//					break;
//				}
//			}
//			 
			
//    		na.travelTo((sh_ll_x - 1)*SQUARE_LENGTH,(sh_ll_y + 1)*SQUARE_LENGTH);
//			lightLoc = new LightLocalizer(odometer, na);
//    		lightLoc.doLightLocalization();
//			na.makeTurn(90, false, true);
//			odometer.setX((sh_ll_x - 1)*SQUARE_LENGTH);
//			odometer.setY((sh_ll_y + 1)*SQUARE_LENGTH);
			
			//cross shallow
    		na.travelTo(sh_ll_x*SQUARE_LENGTH,(sh_ll_y*SQUARE_LENGTH + 0.5*SQUARE_LENGTH));
			na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH, (sh_ur_y-0.5)*SQUARE_LENGTH);
			na.travelTo((sv_ll_x+ 0.5)*SQUARE_LENGTH, sv_ll_y*SQUARE_LENGTH);
			//GO BACK TO START ZONE
//			
//			na.travelTo((sv_ll_x+ 1)*SQUARE_LENGTH, (sv_ll_y-1)*SQUARE_LENGTH);
//    		lightLoc.doLightLocalization();
//			na.makeTurn(90, false, true);
//			odometer.setX((sh_ll_x + 1)*SQUARE_LENGTH);
//			odometer.setY((sh_ll_y - 1)*SQUARE_LENGTH);
//    		
//			na.travelTo(sg_ur_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
//			
//			na.travelTo(CORNERS[greenCorner-1][0]*SQUARE_LENGTH, CORNERS[greenCorner-1][1]*SQUARE_LENGTH);

		//	Button.waitForAnyPress();

		//	na.travelTo(sr_ll_x*SQUARE_LENGTH, sr_ur_y*SQUARE_LENGTH);

			

			//SHALLOW CROSSING - NOT USED FOR BETA DEMO
			/*
    		na.travelTo(sh_ll_x*SQUARE_LENGTH,(sh_ll_y*SQUARE_LENGTH + 0.5*SQUARE_LENGTH));
			na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH, (sh_ur_y-0.5)*SQUARE_LENGTH);
			na.travelTo((sv_ll_x+ 0.5*SQUARE_LENGTH), sv_ll_y);
			 */

			//GO BACK TO START ZONE
			//na.travelTo(sg_ll_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
