package com.thecharge;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import com.thecharge.GripPipelineGym.Line;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class HelloOCV {
	private static boolean useVIDEO = false;

	public static void main(String[] args) throws Exception {
		// while(true);
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		Mat image = new Mat();
		
		VideoCapture camera = null;
		
		if (useVIDEO) {
			camera = new VideoCapture(1);
	    	if(!camera.isOpened()){
				throw new Exception("Can't open the camera.");
	    	}
		}
		//NetworkTable.setClientMode();
		//NetworkTable.setIPAddress("127.0.0.1");
		//NetworkTable table2Rbt = NetworkTable.getTable("Distance");
		//table2Rbt.putNumber("Distance", 0);

		for(int i=0; i<1; i++){
			if (!useVIDEO) {
				//String jpgFile = new String("LTGym3ft.jpg");
				String jpgFile = new String("LTGym6f45d.jpg");
				//String jpgFile = new String("LTGym6f70d.jpg");
				//String jpgFile = new String("LTGym8ft.jpg");
				//String jpgFile = new String("LTGym18ft.jpg");  // No lines found
				

				// Load a test image from file for development
				image = Imgcodecs.imread(jpgFile);

			} else {
				camera.read(image);
			}
			
			// Having selected a source, process the image
			processSingleImage(image);
		}
		if (useVIDEO){
			camera.release();
		}
	}

	private static void processSingleImage(Mat image) throws IOException, Exception {
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
		final double halfFieldAngleH = 34;	// Half of the angle of view for the camera in operation
		final double halfFieldAngleV = 21.3;	// Half of the angle of view for the camera in operation
		double halfFoViewH = 0;	// Half of the field of view in inches (horizontal)
		double halfFoViewV = 0;	// Half of the field of view in inches (vertical)
		double tanHlfAngleH = 0;	// Tangent of the half angle of field of view horizontally
		double tanHlfAngleV = 0;	// Tangent of the half angle of field of view vertically
		double targetWidth = 0;	// The width of the target in inches
		double dist2Target = 0;	// Calculated distance to the target in inches
		
		targetWidth = inchTgtWide + inchGapBetw + inchTgtWide;
		tanHlfAngleH = Math.tan(Math.toRadians(halfFieldAngleH));
		tanHlfAngleV = Math.tan(Math.toRadians(halfFieldAngleV));

		// Capture the image dimensions
		pxlWidth = image.width();
		pxlHeight = image.height();
		System.out.println("xdim = " + pxlWidth);
		System.out.println("ydim = " + pxlHeight);

		// Using the class Pipeline, instantiate here with the specified image
		// (replicate the class here)
		GripPipelineGym gp = new GripPipelineGym();
		double loHue = 74;
		double hiHue = 96;	// 93.99317406143345;
		double loSat = 45.86330935251798;
		double hiSat = 140;	//153;	// 128.80546075085323;
		double loLum = 80.26079136690647;
		double hiLum = 163.61774744027304;
		gp.process(image, loHue, hiHue, loSat, hiSat, loLum, hiLum);

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

		// Save our line data out to a file
		String LineOut;

		PrintWriter outputStream = null;
		try {

			outputStream = new PrintWriter(new FileWriter("srtLineOutput.txt"));

			outputStream.println("AvgX,Angle,Length,X1,X2,Y1,Y2,VLenPerc,IsVert,IsHorz");
			for (int linecount = 0; linecount < targetLines.length; linecount++) {
				LineOut = "" + targetLines[linecount];

				outputStream.println(LineOut);
				// }
			}

		} finally {
			if (outputStream != null) {
				outputStream.close();
			}
		}

		// Create a series of lines to overlay on the original image to assess the quality of the lines identified
		// Note the color spec for BGR rather than RGB
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++){
			Imgproc.line(image, targetLines[zLpCtr1].point1(), targetLines[zLpCtr1].point2(), new Scalar(0,255,255), 1);
		}

		// Save a copy of the amended file with the identified lines
		Imgcodecs.imwrite("img_with_lines.jpg", image);
		
		// Determine which vertical lines probably group together based on spacing from
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
				
				// If this happens to be the very first vertical line of the set, initialize that value as well
				if (firstAdjVline == 0) {
					firstAdjVline = lastAdjVline;
				}
			}
			// Note:  In this loop, nothing happens for lines that aren't vertical
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
		
		// For each vertical line group, find the longest contiguous series of associated segments (ideally 1 series)
		for (int zLpCtr1 = 0; zLpCtr1 <= vLineSet; zLpCtr1++) {
			System.out.println("Evaluating vertical line group " + Integer.toString(zLpCtr1));
			diffVLCount = 0;
			// For each original line relating to this line group, append as able
			for (int zLpCtr2 = 0; zLpCtr2 < ocvLineCount; zLpCtr2++) {
				if (targetLines[zLpCtr2].isVertical()) {
					if ((targetLines[zLpCtr2].xAvg >= nomVlineX[zLpCtr1] - isSameLine) && (targetLines[zLpCtr2].xAvg <= nomVlineX[zLpCtr1] + isSameLine)) {
						// At least for now, confirm that we're evaluating all of the appropriate vertical lines
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
							yminVlineRslt[zLpCtr1] = testY1;	//yminVlineEvl[zLpCtr2];
							ymaxVlineRslt[zLpCtr1] = testY2;	//ymaxVlineEvl[zLpCtr2];
							diffVLCount ++;
						} else {
							// Assess whether this next segment is simply an extension of a previous segment
							wasAppd = false;
							for (int zLpCtr3 = 0; zLpCtr3 < diffVLCount; zLpCtr3++){
								if (testY2 > ymaxVlineEvl[zLpCtr3]) {
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
															yminVlineEvl[zLpCtr3] = yminVlineEvl[zLpCtr4];
															if (ymaxVlineEvl[zLpCtr4] > ymaxVlineEvl[zLpCtr3]) {
																ymaxVlineEvl[zLpCtr3] = ymaxVlineEvl[zLpCtr4];
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
										
										// Exit the loop having appended the necessary segments together
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
		double rectRatio = 0;
		double refRatio = 0;
		double lTgtAccrW = 0;
		double rTgtAccrW = 0;
		
		refRatio = inchGapBetw / inchTgtWide;
		
		boolean spacedOK = false;
		double gap1 = 0;
		double gap2 = 0;
		double gap3 = 0;
		double lineWt = 0;
		double bestWt = 0;
		Integer[] vertSel; 
		vertSel = new Integer[4];
		for (int zLpCtr1 = 0; zLpCtr1 < 4; zLpCtr1++) {
			vertSel[zLpCtr1] = 0;
		}
		
		// Find the best 4 lines to use in the analysis
		if (vLineSet == 3) {
			// This represents the simplest scenario where we found exactly 4 vertical lines
			tgt1LeftXPtr = 0;
			tgt1RightXPtr = 1;
			tgt2LeftXPtr = 2;
			tgt2RightXPtr = 3;
			
			// Now verify that we have acceptable spacing to presume these to be our targets
			rectRatio = (nomVlineX[tgt2LeftXPtr] - nomVlineX[tgt1RightXPtr]);
			rectRatio = rectRatio / (nomVlineX[tgt1RightXPtr] - nomVlineX[tgt1LeftXPtr]);
			lTgtAccrW = rectRatio / refRatio;
			System.out.println("The left target accuracy is 1 : " + Double.toString(lTgtAccrW));
			
			rectRatio = (nomVlineX[tgt2LeftXPtr] - nomVlineX[tgt1RightXPtr]);
			rectRatio = rectRatio / (nomVlineX[tgt2RightXPtr] - nomVlineX[tgt2LeftXPtr]);
			lTgtAccrW = rectRatio / refRatio;
			System.out.println("The right target accuracy is 1 : " + Double.toString(lTgtAccrW));
			
		} else {
			// Find the three sets of signals that yield the best 6.25 / 2 spacing ratio
			for (int zLpCtr1 = 0; zLpCtr1 <= (vLineSet-3); zLpCtr1++) {
				for (int zLpCtr2 = 1; zLpCtr2 <= (vLineSet-2); zLpCtr2++) {
					for (int zLpCtr3 = 2; zLpCtr3 <= (vLineSet-1); zLpCtr3++) {
						for (int zLpCtr4 = 3; zLpCtr4 <= vLineSet; zLpCtr4++) {
							if ((zLpCtr1 < zLpCtr2) && (zLpCtr2 < zLpCtr3) && (zLpCtr3 < zLpCtr4)){
								System.out.println("Assessing line set : " + Integer.toString(zLpCtr1) + ":" + Integer.toString(zLpCtr2) + ":" + Integer.toString(zLpCtr3) + ":" + Integer.toString(zLpCtr4));
								spacedOK = false;
								lineWt = 0;
								gap1 = (nomVlineX[zLpCtr2] - nomVlineX[zLpCtr1]);
								gap2 = (nomVlineX[zLpCtr3] - nomVlineX[zLpCtr2]);
								gap3 = (nomVlineX[zLpCtr4] - nomVlineX[zLpCtr3]);
								System.out.println("Assessing gap of : " + Double.toString(gap1));
								System.out.println("Assessing gap of : " + Double.toString(gap2));
								System.out.println("Assessing gap of : " + Double.toString(gap3));
								if (((gap3 / gap1) < 1.5) && ((gap1 / gap3) < 1.5)) {
									if (((gap2 / gap1) > 3) && ((gap2 / gap3) > 3)) {
										if (((gap2 / gap1) < 10) && ((gap2 / gap3) < 10)) {
											spacedOK = true;
											lineWt = ttlVLens[zLpCtr1] + ttlVLens[zLpCtr2] + ttlVLens[zLpCtr3];
											if (lineWt > bestWt) {
												bestWt = lineWt;
												vertSel[0] = zLpCtr1;
												vertSel[1] = zLpCtr2;
												vertSel[2] = zLpCtr3;
												vertSel[3] = zLpCtr4;
											}
										}
									}
								}
							}
						}
					}
				}
			}
			
			System.out.println("Best weighted value : " + Double.toString(bestWt));
			
			if (bestWt == 0) {
				throw new Exception("vLineSet is perhaps greater than 4 and handling logic is required.");
			}
			System.out.println("Selecting verticals : " + Integer.toString(vertSel[0]) + ":" + Integer.toString(vertSel[1]) + ":" + Integer.toString(vertSel[2]) + ":" + Integer.toString(vertSel[3]));
			tgt1LeftXPtr = vertSel[0];
			tgt1RightXPtr = vertSel[1];
			tgt2LeftXPtr = vertSel[2];
			tgt2RightXPtr = vertSel[3];
		}
		System.out.println(" ");
		
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

		// At this point we can capture 6 preferred points associated with each target's horizontal line fits
		double[] xAtYfind = new double[12];
		double[] yAtYfind = new double[12];
		double[] y1AtYfind = new double[12];
		double stdErr = 0;		// Relative to the upcoming linefit we want to calculate standard error for an indication of quality
		double stdErrT = 0;
		double stdErrB = 0;
		double incrX = 0;
		incrX = (nomXTgt1R - nomXTgt1L) / 7;
		xAtYfind[0] = nomXTgt1L + incrX;
		yAtYfind[0] = 0;
		System.out.println("Find Y at X = " + xAtYfind[0]);
		for (int zLpCtr1 = 1; zLpCtr1 < 6; zLpCtr1++) {
			xAtYfind[zLpCtr1] = xAtYfind[zLpCtr1 - 1] + incrX;
			yAtYfind[zLpCtr1] = 0;
			System.out.println("Find Y at X = " + xAtYfind[zLpCtr1]);
		}
		
		incrX = (nomXTgt2R - nomXTgt2L) / 7;
		xAtYfind[6] = nomXTgt2L + incrX;
		yAtYfind[6] = 0;
		System.out.println("Find Y at X = " + xAtYfind[6]);
		for (int zLpCtr1 = 7; zLpCtr1 < 12; zLpCtr1++) {
			xAtYfind[zLpCtr1] = xAtYfind[zLpCtr1 - 1] + incrX;
			yAtYfind[zLpCtr1] = 0;
			System.out.println("Find Y at X = " + xAtYfind[zLpCtr1]);
		}
		System.out.println(" ");
		
		// For each of the horizontal lines, assess whether it is part of the top or the bottom of one of the rectangular targets
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
			} else if (targetLines[zLpCtr1].isVertical()) {
				// Make the vertical line associations having identified the four verticals of interest
				if ((targetLines[zLpCtr1].xAvg < (nomVlineX[0] + isSameLine)) && (targetLines[zLpCtr1].xAvg > (nomVlineX[0] - isSameLine))) {
					edgeID[zLpCtr1] = "1VL";
				} else if ((targetLines[zLpCtr1].xAvg < (nomVlineX[1] + isSameLine)) && (targetLines[zLpCtr1].xAvg > (nomVlineX[1] - isSameLine))) {
					edgeID[zLpCtr1] = "1VR";
				} else if ((targetLines[zLpCtr1].xAvg < (nomVlineX[2] + isSameLine)) && (targetLines[zLpCtr1].xAvg > (nomVlineX[2] - isSameLine))) {
					edgeID[zLpCtr1] = "2VL";
				} else if ((targetLines[zLpCtr1].xAvg < (nomVlineX[3] + isSameLine)) && (targetLines[zLpCtr1].xAvg > (nomVlineX[3] - isSameLine))) {
					edgeID[zLpCtr1] = "2VR";
				}
			}
		}
		
		// Put the generated image back out to a file
		Imgcodecs.imwrite("RDW2619.jpg", gp.hslThresholdOutput());

		
		
		
		
		
		
		
		
		
		// Get the accompanying horizontal lines in order to validate the height of the rectangles
		double mSlope = 0;		// slope of the line we use to fit the top of the target
		double yIntcpt = 0;		// y intercept for the line we use to fit to
		Random rand = new Random();
		System.out.println(" ");
		
		// Get linefit coordinates for the "top" horizontal lines of target 1
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "1HT") {
				System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
				mSlope = (targetLines[zLpCtr1].ocvY2 - targetLines[zLpCtr1].ocvY1) / (targetLines[zLpCtr1].ocvX2 - targetLines[zLpCtr1].ocvX1);
				yIntcpt = targetLines[zLpCtr1].ocvY1 - mSlope * targetLines[zLpCtr1].ocvX1;
				for (int zLpCtr2 = 0; zLpCtr2 < 6; zLpCtr2++) {
					// Don't generate linefit points that don't reside inside the identified line 
					if (targetLines[zLpCtr1].ocvX2 > targetLines[zLpCtr1].ocvX1) {
						testX1 = targetLines[zLpCtr1].ocvX1;
						testX2 = targetLines[zLpCtr1].ocvX2;
					} else {
						testX1 = targetLines[zLpCtr1].ocvX2;
						testX2 = targetLines[zLpCtr1].ocvX1;
					}
					if ((xAtYfind[zLpCtr2] >= testX1) && (xAtYfind[zLpCtr2] <= testX2)) {
						// We introduce a probablistic approach to overwriting the result from a previous line
						if (yAtYfind[zLpCtr2] == 0) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		
		// Now get linefit coordinates for the top horizontal lines of target 2
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "2HT") {
				System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
				mSlope = (targetLines[zLpCtr1].ocvY2 - targetLines[zLpCtr1].ocvY1) / (targetLines[zLpCtr1].ocvX2 - targetLines[zLpCtr1].ocvX1);
				yIntcpt = targetLines[zLpCtr1].ocvY1 - mSlope * targetLines[zLpCtr1].ocvX1;
				System.out.println("Slope and Intercept as " + Double.toString(mSlope) + " / " + Double.toString(yIntcpt));
				for (int zLpCtr2 = 6; zLpCtr2 < 12; zLpCtr2++) {
					// Don't generate linefit points that don't reside inside the identified line 
					if (targetLines[zLpCtr1].ocvX2 > targetLines[zLpCtr1].ocvX1) {
						testX1 = targetLines[zLpCtr1].ocvX1;
						testX2 = targetLines[zLpCtr1].ocvX2;
					} else {
						testX1 = targetLines[zLpCtr1].ocvX2;
						testX2 = targetLines[zLpCtr1].ocvX1;
					}
					if ((xAtYfind[zLpCtr2] >= testX1) && (xAtYfind[zLpCtr2] <= testX2)) {
						// We introduce a probablistic approach to overwriting the result from a previous line
						if (yAtYfind[zLpCtr2] == 0) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		
		System.out.println(" ");
		
		// Now make the calculations necessary to perform the slope / intercept calculations of the horizontal best fit
		double sumx = 0;
		double sumx2 = 0;
		double sumy = 0;
		int ptCount = 0;
		for (int zLpCtr1 = 0; zLpCtr1 < 12; zLpCtr1++) {
			if (yAtYfind[zLpCtr1] > 0) {
				ptCount ++;
				sumx += xAtYfind[zLpCtr1];
				sumx2 += xAtYfind[zLpCtr1] * xAtYfind[zLpCtr1];
				sumy += yAtYfind[zLpCtr1];
			}
		}
		
		double xbar = sumx / ptCount;
		double ybar = sumy / ptCount;
		
        // second pass: compute summary statistics
        double xxbar = 0.0, yybar = 0.0, xybar = 0.0;
        ptCount = 0;
        for (int zLpCtr1 = 0; zLpCtr1 < 12; zLpCtr1++) {
			if (yAtYfind[zLpCtr1] > 0) {
	            ptCount++;
	            xxbar += (xAtYfind[zLpCtr1]) * (xAtYfind[zLpCtr1]);
	            yybar += (yAtYfind[zLpCtr1]) * (yAtYfind[zLpCtr1]);
	            xybar += (xAtYfind[zLpCtr1]) * (yAtYfind[zLpCtr1]);
			}
        }
        
        // Finally, calculate the slope and y intercept of the best fit top line
        double topSlope = 0;
        double topIntercept = 0;
        double yAtXFitTL = 0;
        double yAtXFitTR = 0;
        double yFit = 0;
        topSlope = (ptCount * xybar) - (sumx * sumy);
        topSlope = topSlope / ((ptCount * xxbar) - (sumx * sumx));
        topIntercept = (ybar - topSlope * xbar);
        
        // At least while we want quality estimates, calculate error data from the linefit
        for (int i = 0; i < 12; i++) {
        	if (yAtYfind[i] > 0) {
        		yFit = topSlope * xAtYfind[i] + topIntercept;
        		y1AtYfind[i] += yFit - yAtYfind[i];
        		stdErrT = y1AtYfind[i] / ptCount;	// This will only be "correct" at the last point
        	}
        }
        stdErr = Math.sqrt(stdErrT);
        
		System.out.println("Determined slope / intercept of " + Double.toString(topSlope) + " / " + Double.toString(topIntercept));
        yAtXFitTL = topSlope * nomXTgt1L + topIntercept;
        yAtXFitTR = topSlope * nomXTgt2R + topIntercept;
		System.out.println("Top Left x / y of " + Double.toString(nomXTgt1L) + " / " + Double.toString(yAtXFitTL));
		System.out.println("Top Right x / y of " + Double.toString(nomXTgt2R) + " / " + Double.toString(yAtXFitTR));
		System.out.println(" ");

		
		
		
		
		
		
		
		
		
		
		
		// Repeat for the "bottom" line
		
		// Clear out the previous y values from the top line analysis (note that we reuse the x values)
		for (int i = 0; i < 12; i++) {
			yAtYfind[i] = 0;
			y1AtYfind[i] = 0;
		}
		
		// Get linefit coordinates for the "bottom" horizontal lines of target 1
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "1HB") {
				System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
				mSlope = (targetLines[zLpCtr1].ocvY2 - targetLines[zLpCtr1].ocvY1) / (targetLines[zLpCtr1].ocvX2 - targetLines[zLpCtr1].ocvX1);
				yIntcpt = targetLines[zLpCtr1].ocvY1 - mSlope * targetLines[zLpCtr1].ocvX1;
				for (int zLpCtr2 = 0; zLpCtr2 < 6; zLpCtr2++) {
					// Don't generate linefit points that don't reside inside the identified line 
					if (targetLines[zLpCtr1].ocvX2 > targetLines[zLpCtr1].ocvX1) {
						testX1 = targetLines[zLpCtr1].ocvX1;
						testX2 = targetLines[zLpCtr1].ocvX2;
					} else {
						testX1 = targetLines[zLpCtr1].ocvX2;
						testX2 = targetLines[zLpCtr1].ocvX1;
					}
					if ((xAtYfind[zLpCtr2] >= testX1) && (xAtYfind[zLpCtr2] <= testX2)) {
						// We introduce a probablistic approach to overwriting the result from a previous line
						if (yAtYfind[zLpCtr2] == 0) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		
		// Now get linefit coordinates for the "bottom" horizontal lines of target 2
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "2HB") {
				System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
				mSlope = (targetLines[zLpCtr1].ocvY2 - targetLines[zLpCtr1].ocvY1) / (targetLines[zLpCtr1].ocvX2 - targetLines[zLpCtr1].ocvX1);
				yIntcpt = targetLines[zLpCtr1].ocvY1 - mSlope * targetLines[zLpCtr1].ocvX1;
				System.out.println("Slope and Intercept as " + Double.toString(mSlope) + " / " + Double.toString(yIntcpt));
				for (int zLpCtr2 = 6; zLpCtr2 < 12; zLpCtr2++) {
					// Don't generate linefit points that don't reside inside the identified line 
					if (targetLines[zLpCtr1].ocvX2 > targetLines[zLpCtr1].ocvX1) {
						testX1 = targetLines[zLpCtr1].ocvX1;
						testX2 = targetLines[zLpCtr1].ocvX2;
					} else {
						testX1 = targetLines[zLpCtr1].ocvX2;
						testX2 = targetLines[zLpCtr1].ocvX1;
					}
					if ((xAtYfind[zLpCtr2] >= testX1) && (xAtYfind[zLpCtr2] <= testX2)) {
						// We introduce a probablistic approach to overwriting the result from a previous line
						if (yAtYfind[zLpCtr2] == 0) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		
		System.out.println(" ");
		
		// Re-initialize the variables used to calculate slope / intercept
		sumx = 0;
		sumx2 = 0;
		sumy = 0;
		ptCount = 0;
		for (int zLpCtr1 = 0; zLpCtr1 < 12; zLpCtr1++) {
			if (yAtYfind[zLpCtr1] > 0) {
				ptCount ++;
				sumx += xAtYfind[zLpCtr1];
				sumx2 += xAtYfind[zLpCtr1] * xAtYfind[zLpCtr1];
				sumy += yAtYfind[zLpCtr1];
			}
		}
		
		xbar = sumx / ptCount;
		ybar = sumy / ptCount;
		
        xxbar = 0.0;
        yybar = 0.0;
        xybar = 0.0;
        ptCount = 0;
        for (int zLpCtr1 = 0; zLpCtr1 < 12; zLpCtr1++) {
			if (yAtYfind[zLpCtr1] > 0) {
	            ptCount++;
	            xxbar += (xAtYfind[zLpCtr1]) * (xAtYfind[zLpCtr1]);
	            yybar += (yAtYfind[zLpCtr1]) * (yAtYfind[zLpCtr1]);
	            xybar += (xAtYfind[zLpCtr1]) * (yAtYfind[zLpCtr1]);
			}
        }
        
        // Finally, calculate the slope and y intercept of the best fit "bottom" line
        double bottomSlope = 0;
        double bottomIntercept = 0;
        double yAtXFitBL = 0;
        double yAtXFitBR = 0;
        bottomSlope = (ptCount * xybar) - (sumx * sumy);					// Numerator
        bottomSlope = bottomSlope / ((ptCount * xxbar) - (sumx * sumx));	// Numerator / Denominator
        bottomIntercept = (ybar - bottomSlope * xbar);

        // At least while we want quality estimates, calculate error data from the linefit
        for (int i = 0; i < 12; i++) {
        	if (yAtYfind[i] > 0) {
        		yFit = bottomSlope * xAtYfind[i] + bottomIntercept;
        		y1AtYfind[i] += yFit - yAtYfind[i];
        		stdErrB = y1AtYfind[i] / ptCount;	// This will only be "correct" at the last point
        	}
        }
        stdErr += Math.sqrt(stdErrB);
        
		System.out.println("Determined slope / intercept of " + Double.toString(bottomSlope) + " / " + Double.toString(bottomIntercept));
        yAtXFitBL = bottomSlope * nomXTgt1L + bottomIntercept;
        yAtXFitBR = bottomSlope * nomXTgt2R + bottomIntercept;
        Point ptTL = new Point(nomXTgt1L, yAtXFitTL);
        Point ptTR = new Point(nomXTgt2R, yAtXFitTR);
        Point ptBL = new Point(nomXTgt1L, yAtXFitBL);
        Point ptBR = new Point(nomXTgt2R, yAtXFitBR);
		System.out.println("Bottom Left x / y of " + Double.toString(nomXTgt1L) + " / " + Double.toString(yAtXFitBL));
		System.out.println("Bottom Right x / y of " + Double.toString(nomXTgt2R) + " / " + Double.toString(yAtXFitBR));
		System.out.println(" ");

		
		
		
		
		
		
		
		
		// Update our picture with the new determined top line
		Imgproc.line(image, ptTL, ptTR, new Scalar(0,0,255), 1);
		Imgproc.line(image, ptBL, ptBR, new Scalar(0,0,255), 1);
		Imgproc.line(image, ptTL, ptBL, new Scalar(0,0,255), 1);
		Imgproc.line(image, ptBR, ptTR, new Scalar(0,0,255), 1);

		// Save a copy of the amended file with the identified lines
		Imgcodecs.imwrite("img_with_lines.jpg", image);
		
		// Houghlines (standard) experiment

		// Save our line data out to a file

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

		outputStream.println("TtlVLen,VXCoord,VYmin,VYmax,HXmin,HXmax");
		for (int linecount = 0; linecount <= vLineSet; linecount++) {
			LineOut = Double.toString(ttlVLens[linecount]);
			LineOut += "," + Double.toString(nomVlineX[linecount]);
			LineOut += "," + Double.toString(yminVlineRslt[linecount]);
			LineOut += "," + Double.toString(ymaxVlineRslt[linecount]);
			LineOut += "," + Double.toString(xminHlineRslt[linecount]);
			LineOut += "," + Double.toString(xmaxHlineRslt[linecount]);
			outputStream.println(LineOut);
			// }
		}

	} finally {
		if (outputStream != null) {
			outputStream.close();
		}
	}
	
	try {

		outputStream = new PrintWriter(new FileWriter("XYLineFit.txt"));

		outputStream.println("xValue,yValue");
		for (int i = 0; i <= 11; i++) {
			LineOut = Double.toString(xAtYfind[i])  + "," + Double.toString(yAtYfind[i]);
			outputStream.println(LineOut);
			// }
		}

	} finally {
		if (outputStream != null) {
			outputStream.close();
		}
	}
	
	
	/*
    Point ptTL = new Point(nomXTgt1L, yAtXFitTL);
    Point ptTR = new Point(nomXTgt2R, yAtXFitTR);
    Point ptBL = new Point(nomXTgt1L, yAtXFitBL);
    Point ptBR = new Point(nomXTgt2R, yAtXFitBR);
    */
	
	double estTgtW = 0;
	double obsTgtW = 0;
	double angOfIncT = 0;	// Angle that the target is rotated relative to perpendicular
	double angOfIncR = 0;	// Angle that the target is positioned relative to the orientation of the robot
	final double rad2degr = 57.29577951; // 360 / 2 / pi()
	
	if ((yAtXFitTL - yAtXFitBL) > (yAtXFitTR - yAtXFitBR)) {
		
		// We are closer to the left edge of the target
		halfFoViewV = 0.5 * inchTgtHigh * pxlHeight / (yAtXFitTL - yAtXFitBL);
		dist2Target = halfFoViewV / tanHlfAngleV;
		System.out.println("The estimated distance to the target (in inches) is " + Double.toString(dist2Target));
		estTgtW = (targetWidth / inchTgtHigh) * (yAtXFitTL - yAtXFitBL);
		
		halfFoViewH = 0.5 * targetWidth * pxlWidth / (nomXTgt2R - nomXTgt1L);
		// Numerator, center of target to center of view in pixels
		angOfIncR = ((nomXTgt2R - nomXTgt1L) / 2) - (pxlWidth / 2);	
		// Convert pixels to inches
		angOfIncR = angOfIncR * 2 * halfFoViewH / pxlWidth;
		// Determine the angle
		angOfIncR = Math.atan(angOfIncR / dist2Target);
		// Convert to degrees
		angOfIncR = rad2degr * angOfIncR;
		
	} else {
		
		// We are closer to the right edge of the target
		halfFoViewV = 0.5 * inchTgtHigh * pxlHeight / (yAtXFitTR - yAtXFitBR);
		dist2Target = halfFoViewV / tanHlfAngleV;
		System.out.println("The estimated distance to the target (in inches) is " + Double.toString(dist2Target));
		estTgtW = (targetWidth / inchTgtHigh) * (yAtXFitTR - yAtXFitBR);

		halfFoViewH = 0.5 * targetWidth * pxlWidth / (nomXTgt2R - nomXTgt1L);
		// Numerator, center of target to center of view in pixels
		angOfIncR = ((nomXTgt2R - nomXTgt1L) / 2) - (pxlWidth / 2);	
		// Convert pixels to inches
		angOfIncR = angOfIncR * 2 * halfFoViewH / pxlWidth;
		// Determine the angle
		angOfIncR = Math.atan(angOfIncR / dist2Target);
		// Convert to degrees
		angOfIncR = rad2degr * angOfIncR;
		
	}
	// Note:  This will have to be corrected as it currently assumes a 90 degree angle of incidence
	obsTgtW = (nomXTgt2R - nomXTgt1L);
	angOfIncT = Math.acos(obsTgtW / estTgtW) * rad2degr;
	
	System.out.println("The estimated angle of incidence (in degrees) for the target is " + Double.toString(angOfIncT));
	System.out.println("The estimated angle of incidence (in degrees) for the robot is " + Double.toString(angOfIncR));
	
	//table2Rbt.
	}
}



