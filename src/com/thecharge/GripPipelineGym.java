package com.thecharge;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import edu.wpi.first.wpilibj.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

/**
* GripPipelineGym class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class GripPipelineGym /*implements VisionPipeline*/ {

	//Outputs
	private Mat hslThresholdOutput = new Mat();
	private ArrayList<Line> findLinesOutput = new ArrayList<Line>();

	// This would typically be set comparable to JPGS_TO_C
	private static final boolean RUNNING_IMAGE_CAPTURE = false;

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	//@Override	
	public void process(Mat source0, double loHue, double hiHue, double loSat, double hiSat, double loLum, double hiLum) {
	

		// Step HSL_Threshold0:
		Mat hslThresholdInput = source0;
		
		double[] hslThresholdHue = {loHue, hiHue};
		double[] hslThresholdSaturation = {loSat, hiSat};
		double[] hslThresholdLuminance = {loLum, hiLum};
		
		hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hslThresholdOutput);

		// Save the output of the HSL processing
		//if (!RUNNING_IMAGE_CAPTURE) Imgcodecs.imwrite("HSL_Output_Image.jpg", hslThresholdOutput);

		// Step Find_Lines0:
		Mat findLinesInput = hslThresholdOutput;
        //Point ptTL = new Point(nomXTgt1L, yAtXFitTL);

		//Imgproc.line(findLinesInput, new Point(50,75), new Point(200, 80), new Scalar(255,255,255), 32);
		//if (!RUNNING_IMAGE_CAPTURE) Imgcodecs.imwrite("HSL_Output_Image.jpg", findLinesInput);

		findLines(findLinesInput, findLinesOutput);


		
		// Step Filter_Lines0:
		//ArrayList<Line> filterLinesLines = findLinesOutput;
		//double filterLinesMinLength = 20;
		//double[] filterLinesAngle = {55.03597122302158, 135.7679180887372};
		//filterLines(filterLinesLines, filterLinesMinLength, filterLinesAngle, filterLinesOutput);

	}

	/**
	 * This method is a generated getter for the output of a HSL_Threshold.
	 * @return Mat output from HSL_Threshold.
	 */
	public Mat hslThresholdOutput() {
		return hslThresholdOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Lines.
	 * @return ArrayList<Line> output from Find_Lines.
	 */
	public ArrayList<Line> findLinesOutput() {
		return findLinesOutput;
	}

	/**
	 * This method is a generated getter for the output of a Filter_Lines.
	 * @return ArrayList<Line> output from Filter_Lines.
	public ArrayList<Line> filterLinesOutput() {
		return filterLinesOutput;
	}
	 */


	/**
	 * Segment an image based on hue, saturation, and luminance ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param lum The min and max luminance
	 * @param output The image in which to store the output.
	 */
	private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum, Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
		Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
			new Scalar(hue[1], lum[1], sat[1]), out);
	}

	public static class Line {
		public final double x1, y1, x2, y2;
		public Line(double x1, double y1, double x2, double y2) {
			this.x1 = x1;
			this.y1 = y1;
			this.x2 = x2;
			this.y2 = y2;
		}
		public double lengthSquared() {
			return Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2);
		}
		public double length() {
			return Math.sqrt(lengthSquared());
		}
		public double angle() {
			return Math.toDegrees(Math.atan2(y2 - y1, x2 - x1));
		}
	}
	/**
	 * Finds all line segments in an image.
	 * @param input The image on which to perform the find lines.
	 * @param lineList The output where the lines are stored.
	 */
	private void findLines(Mat input, ArrayList<Line> lineList) {
		final LineSegmentDetector lsd = Imgproc.createLineSegmentDetector();
		final Mat lines = new Mat();
		lineList.clear();
		if (input.channels() == 1) {
			lsd.detect(input, lines);
		} else {
			final Mat tmp = new Mat();
			Imgproc.cvtColor(input, tmp, Imgproc.COLOR_BGR2GRAY);
			lsd.detect(tmp, lines);
		}
		if (!lines.empty()) {
			for (int i = 0; i < lines.rows(); i++) {
				lineList.add(new Line(lines.get(i, 0)[0], lines.get(i, 0)[1],
					lines.get(i, 0)[2], lines.get(i, 0)[3]));
			}
		}
	}
	

	/**
	 * Filters out lines that do not meet certain criteria.
	 * @param inputs The lines that will be filtered.
	 * @param minLength The minimum length of a line to be kept.
	 * @param angle The minimum and maximum angle of a line to be kept.
	 * @param outputs The output lines after the filter.
	private void filterLines(List<Line> inputs,double minLength,double[] angle,
		List<Line> outputs) {
		outputs = inputs.stream()
				.filter(line -> line.lengthSquared() >= Math.pow(minLength,2))
				.filter(line -> (line.angle() >= angle[0] && line.angle() <= angle[1])
				|| (line.angle() + 180.0 >= angle[0] && line.angle() + 180.0 <= angle[1]))
				.collect(Collectors.toList());
	}
	 */

}

