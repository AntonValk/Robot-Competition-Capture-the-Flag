package ca.mcgill.ecse211.lab4;

/** This class takes care of the robot navigation. It controls the movement by
 * 	implementing movement, turning, distance & angle methods.
 * 	@author Antonios Valkanas, Borui Tao
 * 	@version 1.0
 * 
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation{
	//variables only set once
	private Odometer odometer; 
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	//changing variables 
	private boolean isNavigating;
	private boolean isTurning;
	double[] position = new double[3];
	private double nowX;
	private double nowY;
	private double nowTheta;

	//constructor
	public Navigation(Odometer odometer,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	} 

	/**
	 * This method makes the robot to travel to a specific location (x,y)
	 * First it calculates the heading angle that the robot must face
	 * Then it gets the minimum turning angle which the robot must turn to
	 * Afterwards it calls the method makeCorrectedTurn() to turn to that angle.
	 * Finally it calculates the traveling distance and travel to that distance.
	 * 
	 * @param  x   The x-coordinate of the destination point 
	 * @param  y   The y-coordinate of the destination point 
	 */
	
	void travelTo(double x, double y){

		isNavigating=true;

		nowX = odometer.getX();
		nowY = odometer.getY();
		//calculate the angle we need to turn to
		double theta1 = Math.atan((y-nowY)/(x-nowX))*360.0/(2*Math.PI);
		if(x-nowX<0) theta1= 180.0 + theta1;
		//turn to the proper angle
		makeMinimumTurn(theta1);
		
		double travellingDis = Math.sqrt(Math.pow(x-nowX, 2) + Math.pow(y-nowY, 2));

		//drive forward
		leftMotor.setSpeed(LocalizationLab.FORWARD_SPEED);
		rightMotor.setSpeed(LocalizationLab.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(LocalizationLab.RADIUS, travellingDis), true);
		rightMotor.rotate(convertDistance(LocalizationLab.RADIUS, travellingDis), true);

		//keep calling turnto and checking the distance
		isNavigating=false;
	}
	
	
	/**
	 * This method makes the robot turn theta degrees.
	 * @param theta 		angle of rotation
	 */
	void makeTurn(double theta) {
	    leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	    rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);

	    // Rotate to new angle
	    leftMotor.rotate(convertAngle(LocalizationLab.RADIUS, LocalizationLab.TRACK, theta), true);
	    rightMotor.rotate(-convertAngle(LocalizationLab.RADIUS, LocalizationLab.TRACK, theta), true);
	  }


	/**
	 * This method makes the robot turn using the minimal angle. 
	 * The point is to avoid turns greater than 180 degrees by changing direction.
	 * @param theta 		angle of rotation
	 */
	void makeMinimumTurn(double theta){
		 theta = ((theta % 360) + 360) % 360;
		 if (theta >= 180) theta-=360;
		 leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		 rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);

		 // Rotate to new angle
		 leftMotor.rotate(convertAngle(LocalizationLab.RADIUS, LocalizationLab.TRACK, theta), true);
		 rightMotor.rotate(-convertAngle(LocalizationLab.RADIUS, LocalizationLab.TRACK, theta), false);
	}
	
	/**
	 * This method return true if both motors are moving or the robot is traveling or is turning
	 * 
	 * @return 			whether the motors are moving
	 */
	boolean isNavigating(){
		//returns true if the robot is either Navigating or Turning
		return isNavigating || isTurning || leftMotor.isMoving() && rightMotor.isMoving();
	}
	
	/**
	 * This method stops the robot from moving by setting the speed of motors to zero
	 */

	public void motorStop(){
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
	}
	
	/**
	 * This method get current odometer readings and populates the data to the class 
	 * fields nowX, nowY, nowTheta. 
	 * 
	 * @param odometer		the odometer object
	 */
	
	void getPosition(Odometer odometer){
		odometer.getPosition(position, new boolean[] {true, true, true});
		nowX = position[0];
		nowY = position[1];
		nowTheta = position[2];				 
	}

	/**
	 * This method converts an angle to actual wheel movement distance.
	 * @param radius 		the radius of the wheel
	 * @param distance 		the distance between the wheels. 
	 * @return 			    the distance that has been calculated 
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * This method converts a wheel distance to the equivalent agnle in terms of wheel rotation.
	 * @param radius 		the radius of the wheel
	 * @param width 		the distance between the wheels
	 * @param angle			the angle we wish to convert 
	 * @return 				conversion from angle to wheel rotation distance
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
