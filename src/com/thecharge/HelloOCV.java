package com.thecharge;

import java.util.ArrayList;
import java.util.Arrays;
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
		int ocvLineCount = 0;

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

		
		TargetLine[] targetLines = new TargetLine[(int) ocvLineCount];

		for (int zLpCtr = 0; zLpCtr < ocvLineCount; zLpCtr++) {
			Line currentLine = gp.findLinesOutput().get(zLpCtr);
			targetLines[zLpCtr] = new TargetLine(currentLine);
			//System.out.println("Average x for line = " + xAvg[zLpCtr]);
			
				
		}

		Arrays.sort(targetLines);
		
		// 4 Put the generated image back out to a file
		Imgcodecs.imwrite("RDW2619.jpg", gp.hslThresholdOutput());

		// 6 findlines experiment focusing in particular on vertical lines

		// 7 Houghlines (standard) experiment

		// Save our line data out to a file
		String LineOut;

		PrintWriter outputStream = null;
		try {

			outputStream = new PrintWriter(new FileWriter("ocvLineOutput.txt"));

			outputStream.println("AvgX,Angle,Length,X1,X2,Y1,Y2,VLenPerc,IsVert,IsHorz");
			for (int linecount = 0; linecount < lines.size(); linecount++) {
				LineOut = Double.toString(xAvg[linecount]);
				LineOut = LineOut + "," + Double.toString(ocvAngle[linecount]);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).length());
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).x1);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).x2);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).y1);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).y2);
				LineOut = LineOut + "," + ocvVLenContr[linecount];
				LineOut = LineOut + "," + isVertcl[linecount];
				LineOut = LineOut + "," + isHorzntl[linecount];
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
