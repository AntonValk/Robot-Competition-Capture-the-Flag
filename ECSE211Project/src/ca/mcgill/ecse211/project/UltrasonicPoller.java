package ca.mcgill.ecse211.project;

/**
 * Ultrasonic poller class
 * @author Antonios Valkanas, Borui Tao
 * @version 1.0
 * 
 */

import lejos.robotics.SampleProvider;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
public class UltrasonicPoller extends Thread {
	/**
	 * The sample provider for the ultrasonic sensor.
	 */
	private SampleProvider us;
	/**
	 * The pointer to the ultrasonic localizer.
	 */
	private UltrasonicLocalizer ul;
	/**
	 * The array that saves the sensor data.
	 */
	private float[] usData;

	/**
	 * The constructor sets the objects used to updte the sensor's data.
	 */

	public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicLocalizer ul) {

		this.us = us;
		this.ul = ul;
		this.usData = usData;
	}

	/**
	 * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	 * [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		int distance;
		while (true) {
			us.fetchSample(usData, 0); // acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
			ul.processUSData(distance); // now take action depending on value
			//  bb.processUSData(distance);
			try {
				Thread.sleep(50);
			} catch (Exception e) { } // Poor man's timed sampling
		}
	}
}