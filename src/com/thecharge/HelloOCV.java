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
		final double inchGapBetw = 6.25;	// Distance between reflective targets
		final double inchTgtWide = 2;		// Width of the reflective target
		final double inchTgtHigh = 5;		// Height of the reflective target
		final double inchGnd2Tgt = 10.75;	// Distance from the ground to the bottom of the target

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

		double[] xAvgDiff = new double[(int) ocvLineCount];
		
		TargetLine[] targetLines = new TargetLine[(int) ocvLineCount];

		// Create a list of line parameters that we need to sort as a group, like a row of values in a spreadsheet
		for (int zLpCtr = 0; zLpCtr < ocvLineCount; zLpCtr++) {
			Line currentLine = gp.findLinesOutput().get(zLpCtr);
			targetLines[zLpCtr] = new TargetLine(currentLine);				
		}
		
		// Sort the lines by the average X value of the lines with targetLines as the sorted array
		Arrays.sort(targetLines);
		//TargetLine[] vertLines  = (TargetLine[]) Arrays.stream(targetLines).filter(line->line.isVertical()).toArray();
			
		// Determine which lines probably group together based on spacing from other vertical lines
		double lastxAvg = 0;
		double maxDiffX = 0;
				
		for (int zLpCtr = 0; zLpCtr < ocvLineCount; zLpCtr++){
			//System.out.println("Loop Count = " + Integer.toString(zLpCtr));
			if (targetLines[zLpCtr].isVertical()) {
				//System.out.println("Boolean true for " + Integer.toString(zLpCtr));
				if (lastxAvg > 0){
					xAvgDiff[zLpCtr] = targetLines[zLpCtr].xAvg - lastxAvg;
					if (xAvgDiff[zLpCtr] > maxDiffX){
						maxDiffX = xAvgDiff[zLpCtr];
					}
					System.out.println("Differential xAvg = " + Double.toString(xAvgDiff[zLpCtr]));
				}
				// Note the xAvg value of the "previous" vertical line
				lastxAvg = targetLines[zLpCtr].xAvg;					
			}
		}
		System.out.println("The maximum differential was " + Double.toString(maxDiffX));
		
		//Sum the line lengths for grouped lines
		int vLineSet = 0;    // How many sets of vertical lines are observed
		double[] ttlVLens = new double[(int) ocvLineCount];     // An array of totalized line lengths, though we expect fewer entries than allocated
		double isSameLine = 0;	// Pixels between vertical lines to still consider associated
		double cumulLen = 0;	// Running cumulative length of the group of lines
		double lastAdjVline = 0;	// Position of the preceding vertical line
		
		// Use roughly +- 1/8" as the assumption that the lines represent a group, assuming that the maximum gap between lines
		//	corresponds with the 6.25" gap between targets.
		isSameLine = 0.25 * maxDiffX / inchGapBetw;
		if (isSameLine < 2){
			isSameLine = 2;
		}
		
		// If the first line is vertical, initialize the cumulative length to this length and note the x value
		if (targetLines[0].isVertical()) {
			cumulLen = targetLines[0].length;
			lastAdjVline = xAvgDiff[0];
		}
		
		// Now analyze the rest of the lines
		for (int zLpCtr = 1; zLpCtr < ocvLineCount; zLpCtr++){
			
			if ((xAvgDiff[zLpCtr] > isSameLine) && (targetLines[zLpCtr].isVertical()))   {
				// Capture the cumulative assessment of the line length
				ttlVLens[vLineSet] = cumulLen;
				if (targetLines[zLpCtr].isVertical())  {
					// Reset the cumulative determination to the length of the next line
					cumulLen = targetLines[zLpCtr].length;
					System.out.println("This line of length " + Double.toString(ttlVLens[vLineSet]));
					System.out.println("Its position is roughly " + Double.toString(lastAdjVline));
					lastAdjVline = targetLines[zLpCtr].xAvg;
					// Increment the count of grouped lines
					vLineSet +=1;
				}
			} else if (targetLines[zLpCtr].isVertical()) {
				// The line is vertical but is close enough in proximity to suggest it's the same line
				cumulLen += targetLines[zLpCtr].length;
				System.out.println("Expanded to " + Double.toString(cumulLen));
				lastAdjVline = targetLines[zLpCtr].xAvg;
			}
		}
		
		// We may need to record the length of the last line group
		if (xAvgDiff[(ocvLineCount - 1)] <= isSameLine){
			ttlVLens[vLineSet] = cumulLen;
			System.out.println("The last line is of length " + Double.toString(ttlVLens[vLineSet]));
			System.out.println("Its position is roughly " + Double.toString(targetLines[ocvLineCount - 1].xAvg));
		}
		System.out.println("The number of line sets is " + Integer.toString(vLineSet + 1));
		
		
		// Put the generated image back out to a file
		Imgcodecs.imwrite("RDW2619.jpg", gp.hslThresholdOutput());

		// findlines experiment focusing in particular on vertical lines

		// Houghlines (standard) experiment

		// Save our line data out to a file
		String LineOut;

		PrintWriter outputStream = null;
		try {

			outputStream = new PrintWriter(new FileWriter("ocvLineOutput.txt"));

			outputStream.println("AvgX,Angle,Length,X1,X2,Y1,Y2,VLenPerc,IsVert,IsHorz,XAvgDiff,TtlVLen");
			for (int linecount = 0; linecount < targetLines.length; linecount++) {
				/*
				 * LineOut = Double.toString(xAvg[linecount]);
				LineOut = LineOut + "," + Double.toString(ocvAngle[linecount]);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).length());
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).x1);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).x2);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).y1);
				LineOut = LineOut + "," + Double.toString(gp.findLinesOutput().get(linecount).y2);
				LineOut = LineOut + "," + ocvVLenContr[linecount];
				LineOut = LineOut + "," + isVertcl[linecount];
				LineOut = LineOut + "," + isHorzntl[linecount];
				 */
				// LineOut = String.format ("%.f4", xAvg);
				//if (targetLines[linecount].isVertical()) {
				LineOut = "" + targetLines[linecount];
				LineOut += "," + Double.toString(xAvgDiff[linecount]);
				LineOut += "," + Double.toString(ttlVLens[linecount]);
				outputStream.println(LineOut);					
				//}
			}

		} finally {
			if (outputStream != null) {
				outputStream.close();
			}
		}
	}

}
