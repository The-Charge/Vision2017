package com.thecharge;

import java.util.ArrayList;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import com.thecharge.GripPipelineGym.Line;

public class HelloOCV {

	public static void main(String[] args) throws IOException {

		long pxlWidth = 0;
		long pxlHeight = 0;
		long ocvLineCount = 0;

		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		// Load a test image from file for development
		Mat image = Imgcodecs.imread("LTGym3ft.jpg");

		// Capture the image dimensions
		pxlWidth = image.width();
		pxlHeight = image.height();
		System.out.println("xdim = " + pxlWidth);
		System.out.println("ydim = " + pxlHeight);

		//System.out.println("mat = " + image.dump());

		// Using the class Pipeline, instantiate here with the specified image (replicate the class here)
		GripPipelineGym gp = new GripPipelineGym();
		gp.process(image);

		// Create a List (special Java Collection) of "line" type entries from the specified image processing class
		ArrayList<Line> lines = gp.findLinesOutput();
		ocvLineCount = lines.size();
		System.out.println("Number of lines = " + ocvLineCount);

		double[] xAvg = new double[(int) ocvLineCount];
		double[] ocvX1;
		double[] ocvX2;
		double[] ocvY1;
		double[] ocvY2;
		double[] ocvLen;
		double[] ocvAngle;
		
		for (int zLpCtr = 0; zLpCtr < ocvLineCount; zLpCtr++) {
			xAvg[zLpCtr] = (gp.findLinesOutput().get(zLpCtr).x1 + gp.findLinesOutput().get(zLpCtr).x2) / 2;
			System.out.println("Average x for line = " + xAvg[zLpCtr]);

		}

		// 4 Put the generated image back out to a file
		Imgcodecs.imwrite("RDW2619.jpg", gp.hslThresholdOutput());

		// Note the angles of the various lines
		for (int linecount = 0; linecount < lines.size(); linecount++) {
			System.out.println("Line angle " + linecount + " " + gp.findLinesOutput().get(linecount).angle());
		}

		// 6 findlines experiment focusing in particular on vertical lines

		// 7 Houghlines (standard) experiment

		// Save our line data out to a file
		String LineOut;

		PrintWriter outputStream = null;
		try {

			outputStream = new PrintWriter(new FileWriter("ocvLineOutput.txt"));

			for (int linecount = 0; linecount < lines.size(); linecount++) {
				LineOut = Double.toString(xAvg[linecount]);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).angle());
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).length());
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).x1);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).x2);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).y1);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).y2);
				// LineOut = String.format ("%.f4", xAvg);
				outputStream.println(LineOut);
			}

		} finally {
			if (outputStream != null) {
				outputStream.close();
			}
		}
	}

}
