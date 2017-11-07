package ca.mcgill.ecse211.project;

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

public class CaptureFlag {


	// ** Set these as appropriate for your team and current situation **
	private static final String SERVER_IP = "192.168.2.3";
	private static final int TEAM_NUMBER = 8;

	// Enable/disable printing of debug info from the WiFi class
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

	//static values
	public static final int FORWARD_SPEED = 140;		  // the speed at which the robot is traveling straight
	public static final int ROTATE_SPEED = 70;		  // the speed at which the robot is rotating
	public static final double RADIUS = 2.12; 		  // radius of the wheel
	public static final double TRACK = 11.72; 		  // distance between wheels
	public static final int DISTANCE_THRESHOLD = 30;  // the distance from which the ultrasonic sensor detects the wall
	public static final int NOISE_MARGIN = 1;	  	  // the noise margin
	public static final double SQUARE_LENGTH = 30.48; // the length of the square tile
	public static final double ZIPLENGTH =70;
	public static final int[][] CORNERS = new int[][] {
		{1,1,0},
		{11,1,270},
		{11,11,180},
		{1,11,90}
	}; // the coordinates of corners
	public static int cornerCounter;

	@SuppressWarnings("rawtypes")
	public static void main(String[] args) {
		
		int redTeam = 0, greenTeam = 0;
		int redCorner = 3, greenCorner = 1;
		int og = 1, or = 1;
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
	      System.out.println("Red Team: " + redCorner);
	      greenCorner = ((Long) data.get("GreenCorner")).intValue();
	      System.out.println("Green Team: " + greenCorner);
	      
	      og = ((Long) data.get("OG")).intValue();
	      System.out.println("Green opponent flag: " + og);
	      or = ((Long) data.get("OR")).intValue();
	      System.out.println("Red opponent flag: " + or);

	      red_ll_x =  ((Long) data.get("Red_LL_x")).intValue();
	      red_ll_y =  ((Long) data.get("Red_LL_y")).intValue();
	      red_ur_x =  ((Long) data.get("Red_UR_x")).intValue();
	      red_ur_y =  ((Long) data.get("Red_UR_Y")).intValue();
	      green_ll_x =  ((Long) data.get("Green_LL_x")).intValue();
	      green_ll_y =  ((Long) data.get("Green_LL_y")).intValue();
	      green_ur_x =  ((Long) data.get("Green_UR_x")).intValue();
	      green_ur_y =  ((Long) data.get("Green_UR_x")).intValue();
	      zc_r_x =  ((Long) data.get("ZC_R_x")).intValue();
	      zc_r_y =  ((Long) data.get("ZC_R_Y")).intValue();
	      zo_r_x =  ((Long) data.get("ZO_R_x")).intValue();
	      zo_r_y =  ((Long) data.get("ZO_R_Y")).intValue();
	      zc_g_x =  ((Long) data.get("ZC_G_x")).intValue();
	      zc_g_y =  ((Long) data.get("ZC_G_y")).intValue();
	      zo_g_x =  ((Long) data.get("ZO_G_X")).intValue();
	      zo_g_y =  ((Long) data.get("ZO_G_Y")).intValue();
	      sh_ll_x =  ((Long) data.get("SH_LL_x")).intValue();
	      sh_ll_y =  ((Long) data.get("SH_LL_y")).intValue();
	      sh_ur_x =  ((Long) data.get("SH_UR_x")).intValue();
	      sh_ur_y =  ((Long) data.get("SH_UR_y")).intValue();
	      sv_ll_x =  ((Long) data.get("SV_LL_y")).intValue();
	      sv_ll_y =  ((Long) data.get("SV_LL_y")).intValue();
	      sv_ur_x =  ((Long) data.get("SV_UR_x")).intValue();
	      sv_ur_y =  ((Long) data.get("SV_UR_y")).intValue();
	      sr_ll_x =  ((Long) data.get("SR_LL_x")).intValue();
	      sr_ll_y =  ((Long) data.get("SR_LL_y")).intValue();
	      sr_ur_x =  ((Long) data.get("SR_UR_x")).intValue();
	      sr_ur_y =  ((Long) data.get("SR_UR_y")).intValue();
	      sg_ll_x =  ((Long) data.get("SG_LL_x")).intValue();
	      sg_ll_y =  ((Long) data.get("SG_LL_y")).intValue();
	      sg_ur_x =  ((Long) data.get("SG_UR_x")).intValue();
	      sg_ur_y =  ((Long) data.get("SG_UR_y")).intValue();

	    } catch (Exception e) {
	      System.err.println("Error: " + e.getMessage());
	    }
	    
	    Button.waitForAnyPress();

		// clear the display
		@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[1];		// usData is the buffer in which data are returned
		UltrasonicPoller usPoller = null;									// the selected controller on each cycle


		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		Navigation na = new Navigation(odometer,leftMotor, rightMotor, ziplineMotor);

		ultraLoc = new UltrasonicLocalizer(0, na, odometer);

		odometer.start();
		OdometryDisplay od = new OdometryDisplay(odometer, t,ultraLoc);
		od.start();
		usPoller = new UltrasonicPoller(usDistance, usData, ultraLoc);

		usPoller.start();
		ultraLoc.doUltrasonicLocalization();
		while(na.isNavigating()){
		}
		
		try { 
			Thread.sleep(500);
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
		na.travelTo(3, 3);

		//Wait until the robot stops moving, start the lightLocalization thread
		while(na.isNavigating()){
		}
		lightLoc.doLightLocalization();
		while(na.isNavigating()){
		}

		if (TEAM_NUMBER == redTeam){
			odometer.setX(CORNERS[redCorner-1][0]*SQUARE_LENGTH);
			odometer.setY(CORNERS[redCorner-1][1]*SQUARE_LENGTH);
			odometer.setTheta(CORNERS[redCorner-1][2]);
			
			na.travelTo(zo_r_x*SQUARE_LENGTH, CORNERS[redCorner-1][1]*SQUARE_LENGTH);
			na.travelTo(zo_r_x*SQUARE_LENGTH, zo_r_y*SQUARE_LENGTH);

			na.raphTravelTo((sh_ll_x - 1)*SQUARE_LENGTH,(sh_ll_y + 1)*SQUARE_LENGTH);
    		lightLoc.doLightLocalization();
			na.makeTurn(90);
			odometer.setX((sh_ll_x - 1)*SQUARE_LENGTH);
			odometer.setY((sh_ll_y + 1)*SQUARE_LENGTH);
			
			//shadow crossing
			na.raphTravelTo(sh_ll_x*SQUARE_LENGTH,(sh_ll_y*SQUARE_LENGTH + 0.5*SQUARE_LENGTH));
			na.raphTravelTo((sh_ur_x-0.5)*SQUARE_LENGTH, (sh_ur_y-0.5)*SQUARE_LENGTH);
			na.raphTravelTo((sv_ll_x+ 0.5*SQUARE_LENGTH), sv_ll_y);
			//GO BACK TO START ZONE
			na.raphTravelTo((sv_ll_x+ 1)*SQUARE_LENGTH, (sv_ll_y-1)*SQUARE_LENGTH);
    		lightLoc.doLightLocalization();
    		na.makeTurn(90);
			odometer.setX((sh_ll_x + 1)*SQUARE_LENGTH);
			odometer.setY((sh_ll_y - 1)*SQUARE_LENGTH);
    		
			na.raphTravelTo(sg_ur_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
			//TODO:search for flag
			
			na.travelTo(zo_g_x*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
			while(na.isNavigating()){
				
			}
			na.raphTurnTo(-Math.atan((4/5)*360.0/(2*Math.PI)));
			na.doZipline(3*ZIPLENGTH);
			while(na.isNavigating()){
				
			}
			odometer.setX(zc_r_x*SQUARE_LENGTH);
			odometer.setY(zc_r_y*SQUARE_LENGTH);
			odometer.setTheta(-Math.atan((4/5)*360.0/(2*Math.PI)));
			
			na.raphTravelTo(zo_r_x*SQUARE_LENGTH, zo_r_y*SQUARE_LENGTH);
			lightLoc.doLightLocalization();
			na.makeTurn(90);
			odometer.setX(zo_r_x*SQUARE_LENGTH);
			odometer.setY(zo_r_y*SQUARE_LENGTH);
			odometer.setTheta(0);
			
			na.travelTo(zo_r_y*SQUARE_LENGTH, CORNERS[redCorner-1][1]*SQUARE_LENGTH);

			na.raphTravelTo(CORNERS[redCorner-1][0]*SQUARE_LENGTH, CORNERS[redCorner-1][1]*SQUARE_LENGTH);			
		}
		else if(TEAM_NUMBER == greenTeam){
			odometer.setX(CORNERS[greenCorner-1][0]*SQUARE_LENGTH);
			odometer.setY(CORNERS[greenCorner-1][1]*SQUARE_LENGTH);
			odometer.setTheta(CORNERS[greenCorner-1][2]);
			
			na.raphTravelTo(CORNERS[greenCorner-1][0]*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
			na.raphTravelTo(zo_g_x*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){
				
			}
			na.makeTurn(180);
			odometer.setX(zo_g_x*SQUARE_LENGTH);
			odometer.setY(zo_g_y*SQUARE_LENGTH);
			odometer.setTheta(0);
			
			na.raphTravelTo(zc_g_x*SQUARE_LENGTH, zc_g_y*SQUARE_LENGTH);
			na.raphTurnTo(-Math.atan((4/5)*360.0/(2*Math.PI)));
			na.doZipline(3*ZIPLENGTH);
			while(na.isNavigating()){
				
			}
			odometer.setX(zc_r_x*SQUARE_LENGTH);
			odometer.setY(zc_r_y*SQUARE_LENGTH);
			odometer.setTheta(-Math.atan((4/5)*360.0/(2*Math.PI)));
			
			na.raphTravelTo(zo_r_x*SQUARE_LENGTH, zo_r_y*SQUARE_LENGTH);
			lightLoc.doLightLocalization();
			na.makeTurn(90);
			odometer.setX(zo_r_x*SQUARE_LENGTH);
			odometer.setY(zo_r_y*SQUARE_LENGTH);
			odometer.setTheta(0);
			
			//search for flag
			
			Sound.playNote(Sound.FLUTE, 440, 250);
			Sound.playNote(Sound.FLUTE, 440, 250);
			Sound.playNote(Sound.FLUTE, 440, 250);
			
    		na.raphTravelTo((sh_ll_x - 1)*SQUARE_LENGTH,(sh_ll_y + 1)*SQUARE_LENGTH);
    		lightLoc.doLightLocalization();
			na.makeTurn(90);
			odometer.setX((sh_ll_x - 1)*SQUARE_LENGTH);
			odometer.setY((sh_ll_y + 1)*SQUARE_LENGTH);
			
			//cross shallow
    		na.raphTravelTo(sh_ll_x*SQUARE_LENGTH,(sh_ll_y*SQUARE_LENGTH + 0.5*SQUARE_LENGTH));
			na.raphTravelTo((sh_ur_x-0.5)*SQUARE_LENGTH, (sh_ur_y-0.5)*SQUARE_LENGTH);
			na.raphTravelTo((sv_ll_x+ 0.5)*SQUARE_LENGTH, sv_ll_y*SQUARE_LENGTH);
			//GO BACK TO START ZONE
			
			na.raphTravelTo((sv_ll_x+ 1)*SQUARE_LENGTH, (sv_ll_y-1)*SQUARE_LENGTH);
    		lightLoc.doLightLocalization();
    		na.makeTurn(90);
			odometer.setX((sh_ll_x + 1)*SQUARE_LENGTH);
			odometer.setY((sh_ll_y - 1)*SQUARE_LENGTH);
    		
			na.raphTravelTo(sg_ur_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
			
			na.raphTravelTo(CORNERS[greenCorner-1][0]*SQUARE_LENGTH, CORNERS[greenCorner-1][1]*SQUARE_LENGTH);
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
