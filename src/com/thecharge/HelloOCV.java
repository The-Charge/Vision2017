package com.thecharge;

import java.util.ArrayList;
import java.util.Arrays;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import com.thecharge.GripPipelineGym.Line;

public class HelloOCV {

	public static void main(String[] args) throws Exception {

		// Command = noun-verb
		// Method = verb-noun

		long pxlWidth = 0;
		long pxlHeight = 0;
		int ocvLineCount = 0;
		final double inchGapBetw = 6.25; // Distance between reflective targets
		final double inchTgtWide = 2; // Width of the reflective target
		final double inchTgtHigh = 5; // Height of the reflective target
		final double inchGnd2Tgt = 10.75; // Distance from the ground to the
											// bottom of the target

		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		// Load a test image from file for development
		Mat image = Imgcodecs.imread("LTGym3ft.jpg");

		// Capture the image dimensions
		pxlWidth = image.width();
		pxlHeight = image.height();
		System.out.println("xdim = " + pxlWidth);
		System.out.println("ydim = " + pxlHeight);

		// System.out.println("mat = " + image.dump());

		// Using the class Pipeline, instantiate here with the specified image
		// (replicate the class here)
		GripPipelineGym gp = new GripPipelineGym();
		gp.process(image);

		// Create a List (special Java Collection) of "line" type entries from
		// the specified image processing class
		ArrayList<Line> lines = gp.findLinesOutput();
		//ArrayList<Line> lines = gp.filterLinesOutput();
		ocvLineCount = lines.size();
		System.out.println("Number of lines = " + ocvLineCount);

		System.out.println(" ");
		
		double[] xAvgDiff = new double[(int) ocvLineCount];
		String[] edgeID = new String[(int) ocvLineCount];

		// Initialize the string array
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			edgeID[zLpCtr1] = "";
		}

		TargetLine[] targetLines = new TargetLine[(int) ocvLineCount];

		// Create a list of line parameters that we need to sort as a group,
		// like a row of values in a spreadsheet
		for (int zLpCtr = 0; zLpCtr < ocvLineCount; zLpCtr++) {
			Line currentLine = gp.findLinesOutput().get(zLpCtr);
			targetLines[zLpCtr] = new TargetLine(currentLine);
		}

		// Sort the lines by the average X value of the lines with targetLines
		// as the sorted array
		Arrays.sort(targetLines);
		// TargetLine[] vertLines = (TargetLine[])
		// Arrays.stream(targetLines).filter(line->line.isVertical()).toArray();

		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++){
			Imgproc.line(image, targetLines[zLpCtr1].point1(), targetLines[zLpCtr1].point2(), new Scalar(0,255,0),3);
		}
		
		Imgcodecs.imwrite("img_with_lines.jpg", image);
		
		// Determine which lines probably group together based on spacing from
		// other vertical lines
		double lastxAvg = 0;
		double maxDiffX = 0;

		for (int zLpCtr = 0; zLpCtr < ocvLineCount; zLpCtr++) {
			// System.out.println("Loop Count = " + Integer.toString(zLpCtr));
			if (targetLines[zLpCtr].isVertical()) {
				// System.out.println("Boolean true for " +
				// Integer.toString(zLpCtr));
				if (lastxAvg > 0) {
					xAvgDiff[zLpCtr] = targetLines[zLpCtr].xAvg - lastxAvg;
					if (xAvgDiff[zLpCtr] > maxDiffX) {
						maxDiffX = xAvgDiff[zLpCtr];
					}
					System.out.println("Differential xAvg = " + Double.toString(xAvgDiff[zLpCtr]));
				}
				// Note the xAvg value of the "previous" vertical line
				lastxAvg = targetLines[zLpCtr].xAvg;
			}
		}
		System.out.println("The maximum differential was " + Double.toString(maxDiffX));

		System.out.println(" ");
		
		// Sum the line lengths for grouped lines
		int vLineSet = 0; // How many sets of vertical lines are observed
		double[] ttlVLens = new double[(int) ocvLineCount]; // An array of
															// totalized line
															// lengths, though
															// we expect fewer
															// entries than
															// allocated
		double[] nomVlineX = new double[(int) ocvLineCount]; // Nominal x coordinate of the particular vertical line
		
		double isSameLine = 0; // Pixels between vertical lines to still
								// consider associated
		double cumulLen = 0; // Running cumulative length of the group of lines
		double lastAdjVline = 0; // Position of the preceding vertical line
		double firstAdjVline = 0; // Position of the first line in the grouping

		// Use roughly +- 1/8" as the assumption that the lines represent a
		// group, assuming that the maximum gap between lines
		// corresponds with the 6.25" gap between targets.
		isSameLine = 0.25 * maxDiffX / inchGapBetw;
		if (isSameLine < 2) {
			// It may not be reasonable to expect resolution beyond a couple of pixels
			isSameLine = 2;
		}

		// If the first line is vertical, initialize the cumulative length to
		// this length and note the x value
		if (targetLines[0].isVertical()) {
			cumulLen = targetLines[0].length;
			lastAdjVline = targetLines[0].xAvg;
			firstAdjVline = targetLines[0].xAvg;
		}

		// Now analyze the rest of the lines
		for (int zLpCtr = 1; zLpCtr < ocvLineCount; zLpCtr++) {

			// Note that we do nothing for non-vertical lines
			if (xAvgDiff[zLpCtr] > isSameLine) {
				// The line is vertical and assessed to be the first in the next group of vertical lines
				// Capture the cumulative assessment of the line length
				ttlVLens[vLineSet] = cumulLen;

				// Reset the cumulative determination to the length of the next line
				cumulLen = targetLines[zLpCtr].length;
				
				// Capture the nominal x coordinate
				nomVlineX[vLineSet] = (lastAdjVline + firstAdjVline) / 2;
				
				System.out.println("This line of length " + Double.toString(ttlVLens[vLineSet]));
				System.out.println("Its position is roughly " + Double.toString(nomVlineX[vLineSet]));
				
				// Capture the x coordinate for the first line in the new group
				firstAdjVline = targetLines[zLpCtr].xAvg;
				lastAdjVline = targetLines[zLpCtr].xAvg;
				
				// Increment the count of grouped lines
				vLineSet += 1;
				
			} else if (targetLines[zLpCtr].isVertical()) {
				// The line is vertical but is close enough in proximity to
				// suggest it's the same line
				cumulLen += targetLines[zLpCtr].length;
				System.out.println("Expanded to " + Double.toString(cumulLen));
				lastAdjVline = targetLines[zLpCtr].xAvg;
			}
		}

		// We may need to record the length of the last line group
		if ((xAvgDiff[(ocvLineCount - 1)] <= isSameLine) || (!targetLines[ocvLineCount - 1].isVertical())) {
			ttlVLens[vLineSet] = cumulLen;
			
			// Capture the nominal x coordinate
			nomVlineX[vLineSet] = (lastAdjVline + firstAdjVline) / 2;
			
			System.out.println("The last line is of length " + Double.toString(ttlVLens[vLineSet]));
			System.out.println("Its position is roughly " + Double.toString(nomVlineX[vLineSet]));
		} 

		// Estimate the actual length of the lines identified
		
		System.out.println("The number of line sets is " + Integer.toString(vLineSet + 1));
		// What do we do if we don't find at least 4 lines
		if (vLineSet < 3) {
			throw new Exception("vLineSet is less than 3 (less than 4 vertical lines).");
		}
		
		System.out.println(" ");
		
		// Find the longest contiguous chain of lines for each vertical line group identified
		double[] yminVlineEvl = new double[(int) ocvLineCount]; // Minimum y coordinate of the particular vertical line
		double[] ymaxVlineEvl = new double[(int) ocvLineCount]; // Maximum y coordinate of the particular vertical line
		double[] yminVlineRslt = new double[ocvLineCount]; // Minimum y coordinate of the particular vertical line set
		double[] ymaxVlineRslt = new double[ocvLineCount]; // Maximum y coordinate of the particular vertical line set
		double testY1 = 0;
		double testY2 = 0;
		double okVLGap = 12;
		int diffVLCount = 0;	// The count of the number of different vertical lines in the group, hopefully 1
		boolean wasAppd = false;
		
		// For each vertical line group, find the longest continguous series of associated segments (ideally 1 series)
		for (int zLpCtr1 = 0; zLpCtr1 <= vLineSet; zLpCtr1++) {
			System.out.println("Evaluating vertical line group " + Integer.toString(zLpCtr1));
			diffVLCount = 0;
			// For each original line relating to this line group, append as able
			for (int zLpCtr2 = 0; zLpCtr2 < ocvLineCount; zLpCtr2++) {
				if (targetLines[zLpCtr2].isVertical()) {
					if ((targetLines[zLpCtr2].xAvg >= nomVlineX[zLpCtr1] - isSameLine) && (targetLines[zLpCtr2].xAvg <= nomVlineX[zLpCtr1] + isSameLine)) {
						// At least for now, confirm that we're evaluating all of the appropriate veritical lines
						System.out.println("Evaluating for grouping vertical line " + Integer.toString(zLpCtr2));

						// Having found an(other) line within the line group, get the min and max and see if we can append it to one of the sets
						// or whether we possibly have to append the set
						if (targetLines[zLpCtr2].ocvY1 < targetLines[zLpCtr2].ocvY2) {
							testY1 = targetLines[zLpCtr2].ocvY1;
							testY2 = targetLines[zLpCtr2].ocvY2;
						} else {	
							testY1 = targetLines[zLpCtr2].ocvY2;
							testY2 = targetLines[zLpCtr2].ocvY1;
						}	
							
							
						if (diffVLCount == 0) {
							System.out.println("Initializing resulting vertical line " + Integer.toString(zLpCtr1));
							// Prepare either to compare this segment with other segments in hope of appending vertical lines...
							yminVlineEvl[diffVLCount] = testY1;
							ymaxVlineEvl[diffVLCount] = testY2;
							// But capture the result in case there are no more lines to append.
							yminVlineRslt[zLpCtr1] = yminVlineEvl[zLpCtr2];
							ymaxVlineRslt[zLpCtr1] = ymaxVlineEvl[zLpCtr2];
							diffVLCount ++;
						} else {
							// Assess whether this next segment is simply an extension of a previous segment
							wasAppd = false;
							for (int zLpCtr3 = 0; zLpCtr3 < diffVLCount; zLpCtr3++){
								if (testY2 > ymaxVlineEvl[zLpCtr3]){
									if (testY1 < (ymaxVlineEvl[zLpCtr3] + okVLGap)) {
										// Append the two lines
										ymaxVlineEvl[zLpCtr3] = testY2;
										if (testY1 < yminVlineEvl[zLpCtr3]) {
											// While we wouldn't expect this to ever happen, prepare for the unexpected
											yminVlineEvl[zLpCtr3] = testY1;
										}
										wasAppd = true;
										
										// Assess whether we now have a new longest combined line segment at this x value
										if ((ymaxVlineEvl[zLpCtr3] - yminVlineEvl[zLpCtr3]) > (ymaxVlineRslt[zLpCtr1] - yminVlineRslt[zLpCtr1])) {
											yminVlineRslt[zLpCtr1] = yminVlineEvl[zLpCtr3];
											ymaxVlineRslt[zLpCtr1] = ymaxVlineEvl[zLpCtr3];
										}
										
										// Having appended the segments, evaluate the ability to append additional segments
										for (int zLpCtr4 = 0; zLpCtr4 < diffVLCount; zLpCtr4++) {
											if (zLpCtr4 != zLpCtr3) {
												// Possibly append this new grouped segment with other segments 
												// before or after in the array, retaining lower array position
												if (ymaxVlineEvl[zLpCtr4] > ymaxVlineEvl[zLpCtr3]) {
													if ((yminVlineEvl[zLpCtr4] > (yminVlineEvl[zLpCtr3] - okVLGap)) && (yminVlineEvl[zLpCtr4] < (ymaxVlineEvl[zLpCtr3] + okVLGap))){
														// Append the lines
														if (zLpCtr3 == (diffVLCount - 1)) {
															// Update at zLpCtr4 and eliminate at zLpCtr3
															if (yminVlineEvl[zLpCtr3] < yminVlineEvl[zLpCtr4]) {
																yminVlineEvl[zLpCtr4] = yminVlineEvl[zLpCtr3];
															}
															// Leave ymaxVlineEvl[zLpCtr4] alone
															yminVlineEvl[zLpCtr3] = 0;
															ymaxVlineEvl[zLpCtr3] = 0;
															diffVLCount--; 
														} else if (zLpCtr4 == (diffVLCount - 1)) {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															ymaxVlineEvl[zLpCtr3] = ymaxVlineEvl[zLpCtr4];
															if (yminVlineEvl[zLpCtr4] < yminVlineEvl[zLpCtr3]) {
																yminVlineEvl[zLpCtr3] = yminVlineEvl[zLpCtr4];
															}
															yminVlineEvl[zLpCtr4] = 0;
															ymaxVlineEvl[zLpCtr4] = 0;
															diffVLCount--; 
														} else {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															if (yminVlineEvl[zLpCtr4] < yminVlineEvl[zLpCtr3]) {
																yminVlineEvl[zLpCtr3] = yminVlineEvl[zLpCtr4];
															}
															// Leave ymaxVlineEvl[zLpCtr3] alone
															yminVlineEvl[zLpCtr4] = 0;
															ymaxVlineEvl[zLpCtr4] = 0;
														}
														
														// Assess whether we now have a new longest combined line segment at this x value
														if ((ymaxVlineEvl[zLpCtr3] - yminVlineEvl[zLpCtr3]) > (ymaxVlineRslt[zLpCtr1] - yminVlineRslt[zLpCtr1])) {
															yminVlineRslt[zLpCtr1] = yminVlineEvl[zLpCtr3];
															ymaxVlineRslt[zLpCtr1] = ymaxVlineEvl[zLpCtr3];
														} else if ((ymaxVlineEvl[zLpCtr4] - yminVlineEvl[zLpCtr4]) > (ymaxVlineRslt[zLpCtr1] - yminVlineRslt[zLpCtr1])) {
															yminVlineRslt[zLpCtr1] = yminVlineEvl[zLpCtr4];
															ymaxVlineRslt[zLpCtr1] = ymaxVlineEvl[zLpCtr4];
														}
														
													}
												} else {
													if ((yminVlineEvl[zLpCtr3] > (yminVlineEvl[zLpCtr4] - okVLGap)) && (yminVlineEvl[zLpCtr3] < (ymaxVlineEvl[zLpCtr4] + okVLGap))){
														// Append the lines
														if (zLpCtr3 == (diffVLCount - 1)) {
															// Update at zLpCtr4 and eliminate at zLpCtr3
															if (yminVlineEvl[zLpCtr3] < yminVlineEvl[zLpCtr4]) {
																yminVlineEvl[zLpCtr4] = yminVlineEvl[zLpCtr3];
															}
															// Leave ymaxVlineEvl[zLpCtr4] alone
															yminVlineEvl[zLpCtr3] = 0;
															ymaxVlineEvl[zLpCtr3] = 0;
															diffVLCount--; 
														} else if (zLpCtr4 == (diffVLCount - 1)) {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															ymaxVlineEvl[zLpCtr3] = ymaxVlineEvl[zLpCtr4];
															if (yminVlineEvl[zLpCtr4] < yminVlineEvl[zLpCtr3]) {
																yminVlineEvl[zLpCtr3] = yminVlineEvl[zLpCtr4];
															}
															yminVlineEvl[zLpCtr4] = 0;
															ymaxVlineEvl[zLpCtr4] = 0;
															diffVLCount--; 
														} else {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															if (yminVlineEvl[zLpCtr4] < yminVlineEvl[zLpCtr3]) {
																yminVlineEvl[zLpCtr3] = yminVlineEvl[zLpCtr4];
															}
															// Leave ymaxVlineEvl[zLpCtr3] alone
															yminVlineEvl[zLpCtr4] = 0;
															ymaxVlineEvl[zLpCtr4] = 0;
														}
														
														// Assess whether we now have a new longest combined line segment at this x value
														if ((ymaxVlineEvl[zLpCtr3] - yminVlineEvl[zLpCtr3]) > (ymaxVlineRslt[zLpCtr1] - yminVlineRslt[zLpCtr1])) {
															yminVlineRslt[zLpCtr1] = yminVlineEvl[zLpCtr3];
															ymaxVlineRslt[zLpCtr1] = ymaxVlineEvl[zLpCtr3];
														} else if ((ymaxVlineEvl[zLpCtr4] - yminVlineEvl[zLpCtr4]) > (ymaxVlineRslt[zLpCtr1] - yminVlineRslt[zLpCtr1])) {
															yminVlineRslt[zLpCtr1] = yminVlineEvl[zLpCtr4];
															ymaxVlineRslt[zLpCtr1] = ymaxVlineEvl[zLpCtr4];
														}
														
													}
												}
											}
										}
										
										// Exit the loop having appended the segments
										zLpCtr3 = diffVLCount + 1;
									}
								} else {
									// So (testY2 <= ymaxVlineEvl[zLpCtr3])
									// Same logic but reversing the evaluation
									if (testY2 > (yminVlineEvl[zLpCtr3] - okVLGap)) {
										// Append the two lines
										yminVlineEvl[zLpCtr3] = testY1;
										wasAppd = true;
										
										// Assess whether we now have a new longest combined line segment at this x value
										if ((ymaxVlineEvl[zLpCtr3] - yminVlineEvl[zLpCtr3]) > (ymaxVlineRslt[zLpCtr1] - yminVlineRslt[zLpCtr1])) {
											yminVlineRslt[zLpCtr1] = yminVlineEvl[zLpCtr3];
											ymaxVlineRslt[zLpCtr1] = ymaxVlineEvl[zLpCtr3];
										}
																				
										// Having appended the segments, evaluate the ability to append additional segments
										for (int zLpCtr4 = 0; zLpCtr4 < diffVLCount; zLpCtr4++) {
											if (zLpCtr4 != zLpCtr3) {
												// Possibly append this new grouped segment with other segments 
												// before or after in the array, retaining lower array position
												if (ymaxVlineEvl[zLpCtr4] > ymaxVlineEvl[zLpCtr3]) {
													if ((yminVlineEvl[zLpCtr4] > (yminVlineEvl[zLpCtr3] - okVLGap)) && (yminVlineEvl[zLpCtr4] < (ymaxVlineEvl[zLpCtr3] + okVLGap))){
														// Append the lines
														if (zLpCtr3 == (diffVLCount - 1)) {
															// Update at zLpCtr4 and eliminate at zLpCtr3
															if (yminVlineEvl[zLpCtr3] < yminVlineEvl[zLpCtr4]) {
																yminVlineEvl[zLpCtr4] = yminVlineEvl[zLpCtr3];
															}
															// Leave ymaxVlineEvl[zLpCtr4] alone
															yminVlineEvl[zLpCtr3] = 0;
															ymaxVlineEvl[zLpCtr3] = 0;
															diffVLCount--; 
														} else if (zLpCtr4 == (diffVLCount - 1)) {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															ymaxVlineEvl[zLpCtr3] = ymaxVlineEvl[zLpCtr4];
															if (yminVlineEvl[zLpCtr4] < yminVlineEvl[zLpCtr3]) {
																yminVlineEvl[zLpCtr3] = yminVlineEvl[zLpCtr4];
															}
															yminVlineEvl[zLpCtr4] = 0;
															ymaxVlineEvl[zLpCtr4] = 0;
															diffVLCount--; 
														} else {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															if (yminVlineEvl[zLpCtr4] < yminVlineEvl[zLpCtr3]) {
																yminVlineEvl[zLpCtr3] = yminVlineEvl[zLpCtr4];
															}
															// Leave ymaxVlineEvl[zLpCtr3] alone
															yminVlineEvl[zLpCtr4] = 0;
															ymaxVlineEvl[zLpCtr4] = 0;
														}
														
														// Assess whether we now have a new longest combined line segment at this x value
														if ((ymaxVlineEvl[zLpCtr3] - yminVlineEvl[zLpCtr3]) > (ymaxVlineRslt[zLpCtr1] - yminVlineRslt[zLpCtr1])) {
															yminVlineRslt[zLpCtr1] = yminVlineEvl[zLpCtr3];
															ymaxVlineRslt[zLpCtr1] = ymaxVlineEvl[zLpCtr3];
														} else if ((ymaxVlineEvl[zLpCtr4] - yminVlineEvl[zLpCtr4]) > (ymaxVlineRslt[zLpCtr1] - yminVlineRslt[zLpCtr1])) {
															yminVlineRslt[zLpCtr1] = yminVlineEvl[zLpCtr4];
															ymaxVlineRslt[zLpCtr1] = ymaxVlineEvl[zLpCtr4];
														}
														
													}
												} else {
													if ((yminVlineEvl[zLpCtr3] > (yminVlineEvl[zLpCtr4] - okVLGap)) && (yminVlineEvl[zLpCtr3] < (ymaxVlineEvl[zLpCtr4] + okVLGap))){
														// Append the lines
														if (zLpCtr3 == (diffVLCount - 1)) {
															// Update at zLpCtr4 and eliminate at zLpCtr3
															if (yminVlineEvl[zLpCtr3] < yminVlineEvl[zLpCtr4]) {
																yminVlineEvl[zLpCtr4] = yminVlineEvl[zLpCtr3];
															}
															// Leave ymaxVlineEvl[zLpCtr4] alone
															yminVlineEvl[zLpCtr3] = 0;
															ymaxVlineEvl[zLpCtr3] = 0;
															diffVLCount--; 
														} else if (zLpCtr4 == (diffVLCount - 1)) {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															ymaxVlineEvl[zLpCtr3] = ymaxVlineEvl[zLpCtr4];
															if (yminVlineEvl[zLpCtr4] < yminVlineEvl[zLpCtr3]) {
																yminVlineEvl[zLpCtr3] = yminVlineEvl[zLpCtr4];
															}
															yminVlineEvl[zLpCtr4] = 0;
															ymaxVlineEvl[zLpCtr4] = 0;
															diffVLCount--; 
														} else {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															if (yminVlineEvl[zLpCtr4] < yminVlineEvl[zLpCtr3]) {
																yminVlineEvl[zLpCtr3] = yminVlineEvl[zLpCtr4];
															}
															// Leave ymaxVlineEvl[zLpCtr3] alone
															yminVlineEvl[zLpCtr4] = 0;
															ymaxVlineEvl[zLpCtr4] = 0;
														}
														
														// Assess whether we now have a new longest combined line segment at this x value
														if ((ymaxVlineEvl[zLpCtr3] - yminVlineEvl[zLpCtr3]) > (ymaxVlineRslt[zLpCtr1] - yminVlineRslt[zLpCtr1])) {
															yminVlineRslt[zLpCtr1] = yminVlineEvl[zLpCtr3];
															ymaxVlineRslt[zLpCtr1] = ymaxVlineEvl[zLpCtr3];
														} else if ((ymaxVlineEvl[zLpCtr4] - yminVlineEvl[zLpCtr4]) > (ymaxVlineRslt[zLpCtr1] - yminVlineRslt[zLpCtr1])) {
															yminVlineRslt[zLpCtr1] = yminVlineEvl[zLpCtr4];
															ymaxVlineRslt[zLpCtr1] = ymaxVlineEvl[zLpCtr4];
														}
														
													}
												}
											}
										}
										
										// Exit the loop having appended the segments together
										zLpCtr3 = diffVLCount + 1;
									}
								}
							}	
							if (!wasAppd) {
								// Record them separately for now
								yminVlineEvl[diffVLCount] = testY1;
								ymaxVlineEvl[diffVLCount] = testY2;
								diffVLCount ++;
							}
						}
					}
				}
			}
		}
		
		System.out.println(" ");
		
		int tgt1LeftXPtr = 0;
		int tgt1RightXPtr = 0;
		int tgt2LeftXPtr = 0;
		int tgt2RightXPtr = 0;
		
		// Find the best 4 lines to use in the analysis
		if (vLineSet == 3) {
			tgt1LeftXPtr = 0;
			tgt1RightXPtr = 1;
			tgt2LeftXPtr = 2;
			tgt2RightXPtr = 3;
		} else {
			throw new Exception("vLineSet is perhaps greater than 4 and handling logic is required.");
		}
		
		// Find the horizontal lines of the targets
		double[] xminHlineEvl = new double[(int) ocvLineCount]; // Minimum y coordinate of the particular vertical line
		double[] xmaxHlineEvl = new double[(int) ocvLineCount]; // Maximum y coordinate of the particular vertical line
		double[] xminHlineRslt = new double[ocvLineCount]; // Minimum y coordinate of the particular vertical line set
		double[] xmaxHlineRslt = new double[ocvLineCount]; // Maximum y coordinate of the particular vertical line set
		double testX1 = 0;
		double testX2 = 0;
		double okHLGap = 12;
		double nomYTgtTop = 0;
		double nomYTgtBtm = 0;
		double nomXTgt1L = 0;
		double nomXTgt1R = 0;
		double nomXTgt2L = 0;
		double nomXTgt2R = 0;

		// Initialize the arrays for the x max and min for the horizontal lines
		for (int zLpCtr1 = 0; zLpCtr1 <= vLineSet; zLpCtr1 ++) {
			xminHlineRslt[zLpCtr1] = pxlWidth;
			xmaxHlineRslt[zLpCtr1] = 0;
		}
		
 		// Note the nominal target top and bottom values (calculate a simple average from the vertical lines)
		for (int zLpCtr1 = 0; zLpCtr1 <= vLineSet; zLpCtr1 ++) {
			nomYTgtBtm += yminVlineRslt[zLpCtr1];
			nomYTgtTop += ymaxVlineRslt[zLpCtr1];
		}
		nomYTgtBtm = nomYTgtBtm/(vLineSet + 1);
		nomYTgtTop = nomYTgtTop/(vLineSet + 1);
		
		System.out.println("The top of the target is estimated at " + Double.toString(nomYTgtTop));
		System.out.println("The bottom of the target is estimated at " + Double.toString(nomYTgtBtm));
		System.out.println("The allowable error is " + Double.toString(okHLGap));
		
		// Note the nominal left and right target X values for each of the two targets
		nomXTgt1L = nomVlineX[tgt1LeftXPtr];
		System.out.println("Left edge of target 1 has x = " + Double.toString(nomXTgt1L));
		nomXTgt1R = nomVlineX[tgt1RightXPtr];
		System.out.println("Right edge of target 1 has x = " + Double.toString(nomXTgt1R));
		nomXTgt2L = nomVlineX[tgt2LeftXPtr];
		System.out.println("Left edge of target 2 has x = " + Double.toString(nomXTgt2L));
		nomXTgt2R = nomVlineX[tgt2RightXPtr];
		System.out.println("Right edge of target 2 has x = " + Double.toString(nomXTgt2R));
		System.out.println(" ");

		//int diffHLCount = 0;	// The count of the number of different horizontal lines in the group, hopefully 1
		// For each of the vertical lines, assess whether it is part of the top or the bottom of one of the rectangular targets
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			// No processing required for lines that aren't horizontal
			if (targetLines[zLpCtr1].isHorizontal()) {
				// For each of the grouped vertical line pairs, look for the target tops and bottoms
				
				System.out.println("Evaluating horizontal line " + Integer.toString(zLpCtr1));

				// Having found an(other) line within the line group, get the min and max and see if we can validate the target's horizontal lines
				if (targetLines[zLpCtr1].ocvX1 < targetLines[zLpCtr1].ocvX2) {
					testX1 = targetLines[zLpCtr1].ocvX1;
					testX2 = targetLines[zLpCtr1].ocvX2;
				} else {	
					testX1 = targetLines[zLpCtr1].ocvX2;
					testX2 = targetLines[zLpCtr1].ocvX1;
				}	
				
				// Because the line is roughly horizontal, it doesn't matter much whether we choose Y1 or Y2
				if ((targetLines[zLpCtr1].ocvY1 < (nomYTgtTop + okHLGap)) && (targetLines[zLpCtr1].ocvY1 > (nomYTgtTop - okHLGap))) {
					System.out.println("Initial top alignment for line " + Integer.toString(zLpCtr1));
					
					// Assess whether we're looking at the left or the right target for this line
					if ((testX1 > (nomXTgt1L - okHLGap)) && (testX2 < (nomXTgt1R + okHLGap))) {
						// Top left horizontal target line
						edgeID[zLpCtr1] = "1HT";
						xminHlineEvl[1] = testX1;
						xmaxHlineEvl[1] = testX2;
						if (testX1 < xminHlineRslt[1]) {
							xminHlineRslt[1] = testX1;
						}
						if (testX2 > xmaxHlineRslt[1]) {
							xmaxHlineRslt[1] = testX2;
						}
						
					} else if ((testX1 > (nomXTgt2L - okHLGap)) && (testX2 < (nomXTgt2R + okHLGap))) {
						// Top right horizontal target line
						edgeID[zLpCtr1] = "2HT";
						xminHlineEvl[3] = testX1;
						xmaxHlineEvl[3] = testX2;
						if (testX1 < xminHlineRslt[3]) {
							xminHlineRslt[3] = testX1;
						}
						if (testX2 > xmaxHlineRslt[3]) {
							xmaxHlineRslt[3] = testX2;
						}
						
					}
					
				} else if ((targetLines[zLpCtr1].ocvY1 < (nomYTgtBtm + okHLGap)) && (targetLines[zLpCtr1].ocvY1 > (nomYTgtBtm - okHLGap))) {
					System.out.println("Initial bottom alignment for line " + Integer.toString(zLpCtr1));

					// Assess whether we're looking at the left or the right target for this line
					if ((testX1 > (nomXTgt1L - okHLGap)) && (testX2 < (nomXTgt1R + okHLGap))) {
						// Bottom left horizontal target line
						edgeID[zLpCtr1] = "1HB";
						xminHlineEvl[0] = testX1;
						xmaxHlineEvl[0] = testX2;
						if (testX1 < xminHlineRslt[0]) {
							xminHlineRslt[0] = testX1;
						}
						if (testX2 > xmaxHlineRslt[0]) {
							xmaxHlineRslt[0] = testX2;
						}
						
					} else if ((testX1 > (nomXTgt2L - okHLGap)) && (testX2 < (nomXTgt2R + okHLGap))) {
						// Bottom right horizontal target line
						edgeID[zLpCtr1] = "2HB";
						xminHlineEvl[2] = testX1;
						xmaxHlineEvl[2] = testX2;
						if (testX1 < xminHlineRslt[2]) {
							xminHlineRslt[2] = testX1;
						}
						if (testX2 > xmaxHlineRslt[2]) {
							xmaxHlineRslt[2] = testX2;
						}
						
					}
					
				}
			}
		}
		
		// Put the generated image back out to a file
		Imgcodecs.imwrite("RDW2619.jpg", gp.hslThresholdOutput());

		// Get the accompanying horizontal lines in order to validate the height of the rectangles
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			
		}
		
		// findlines experiment focusing in particular on vertical lines
		
		// Houghlines (standard) experiment

		// Save our line data out to a file
		String LineOut;

		PrintWriter outputStream = null;
		try {

			outputStream = new PrintWriter(new FileWriter("ocvLineOutput.txt"));

			outputStream.println("AvgX,Angle,Length,X1,X2,Y1,Y2,VLenPerc,IsVert,IsHorz,XAvgDiff,Bounds");
			for (int linecount = 0; linecount < targetLines.length; linecount++) {
				LineOut = "" + targetLines[linecount];
				LineOut += "," + Double.toString(xAvgDiff[linecount]);
				LineOut += "," + edgeID[linecount];

				outputStream.println(LineOut);
				// }
			}

		} finally {
			if (outputStream != null) {
				outputStream.close();
			}
		}


	try {

		outputStream = new PrintWriter(new FileWriter("RsltLineOutput.txt"));

		outputStream.println("TtlVLen,VXCoord,VYmin,VYmax,HXmin,HXmax,Affil");
		for (int linecount = 0; linecount <= vLineSet; linecount++) {
			LineOut = Double.toString(ttlVLens[linecount]);
			LineOut += "," + Double.toString(nomVlineX[linecount]);
			LineOut += "," + Double.toString(yminVlineRslt[linecount]);
			LineOut += "," + Double.toString(ymaxVlineRslt[linecount]);
			LineOut += "," + Double.toString(xminHlineRslt[linecount]);
			LineOut += "," + Double.toString(xmaxHlineRslt[linecount]);
			LineOut += "," + edgeID[linecount];
			outputStream.println(LineOut);
			// }
		}

	} finally {
		if (outputStream != null) {
			outputStream.close();
		}
	}
	}
	
	//System.exit(0);
}



