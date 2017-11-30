package ca.mcgill.ecse211.project;

/** 
 * Flag Search Controller class
 * @author Raphael Di Piazza
 * @version 1.0
 *  
 */

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class implements the flag search routine by executing a wall following until the right flag is captured.
 */
public class FlagSearchController {
	
	/**
	 * The Navigation class.
	 */
	private Navigation na;
	
	/**
	 * The UltrasonicLocalizer class;
	 */
	private UltrasonicLocalizer us;
	
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
	 * The constructor for the flag search controller.
	 * @param na			pointer to the navigation class.
	 * @param us			pointer to the ultrasonicLocalizer class.
	 * @param leftMotor		pointer to the left wheel motor.
	 * @param rightMotor	pointer to the right wheel motor.
	 */
	public FlagSearchController(Navigation na, UltrasonicLocalizer us, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
		this.na = na;
		this.us = us;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	/**
	 * This method implements the bangbang controller for the flag detection.
	 * The robot will navigate through the blocks until it gets the block with the correct color.
	 * @param bandCenter	The distance from the blocks.
	 * @param bandWidth		The error margin.
	 * @param motorLow		The low motor speed.
	 * @param motorHigh		The high motor speed.
	 */
	public void detectFlag(int bandCenter, int bandWidth, int motorLow, int motorHigh, int flag){
		na.rotateUltraMotor(45,false);
		float forwardLimit = 180;
		float backwardLimit = 1;
		float bwMotorLow = 50;
		float fwLimitSpd = 150;
		while(lightValue != CaptureFlag.flags[flag-1]){
			System.out.println("The distance is "+ us.distance);
			if (us.distance < backwardLimit){
				this.leftMotor.setSpeed(bwMotorLow);
				this.rightMotor.setSpeed(motorHigh);
				this.leftMotor.backward();
				this.rightMotor.backward();
			}else if(us.distance < (bandCenter - bandWidth) && us.distance > backwardLimit){
				this.leftMotor.setSpeed(motorLow);
				this.rightMotor.setSpeed(motorHigh);
				this.leftMotor.forward();
				this.rightMotor.forward();
			}else if(us.distance > (bandCenter - bandWidth) && us.distance < (bandCenter + bandWidth)){
				this.leftMotor.setSpeed(motorHigh);
				this.rightMotor.setSpeed(motorHigh);
				this.leftMotor.forward();
				this.rightMotor.forward();
			}else if(us.distance > (bandCenter + bandWidth) && us.distance < forwardLimit){
				this.leftMotor.setSpeed(motorHigh);
				this.rightMotor.setSpeed(motorLow);
				this.leftMotor.forward();
				this.rightMotor.forward();
			}else if(us.distance > forwardLimit){
				this.rightMotor.setSpeed(motorHigh);
				this.leftMotor.setSpeed(fwLimitSpd);
				this.leftMotor.forward();
				this.rightMotor.forward();
			}
		}
		this.rightMotor.stop();
		this.leftMotor.stop();
		Sound.playNote(Sound.FLUTE, 440, 250);
		Sound.playNote(Sound.FLUTE, 440, 250);
		Sound.playNote(Sound.FLUTE, 440, 250);
	}
	
}
