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
import org.opencv.videoio.VideoCapture;

import com.thecharge.GripPipelineGym.Line;

//Need to add java libraries
//import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class HelloOCV {
	private static boolean useVIDEO = false;
	
	private static int pxlWidth = 0;
	private static int pxlHeight = 0;
	private static int ocvLineCount = 0;
	private static final double INCH_GAP_BETW = 6.25; // Distance between reflective targets
	private static final double INCH_TGT_WIDE = 2; // Width of the reflective target
	private static final double INCH_TGT_HEIGHT = 5; // Height of the reflective target
	private static final double INCH_GND_TO_TGT = 10.75; // Distance from the ground to the bottom of the target
	private static final double HALF_FIELD_ANGLE = 34;	// Half of the angle of view for the camera in operation
	private static final double INCH_IS_SAME_LINE = 0.25;
	private static double halfFoView = 0;	// Half of the field of view in inches
	private static final double TAN_HALF_FIELD_ANGLE =  Math.tan(Math.toRadians(HALF_FIELD_ANGLE));	// Tangent of the half angle of field of view
	private static final double TARGET_WIDTH =  INCH_TGT_WIDE + INCH_GAP_BETW + INCH_TGT_WIDE;	// The width of the target in inches
	private static double dist2Target = 0;	// Calculated distance to the target in inches
	private static double loHue = 74;
	private static double hiHue = 96;	// 93.99317406143345;
	private static double loSat = 45.86330935251798;
	private static double hiSat = 140;	//153;	// 128.80546075085323;
	private static double loLum = 80.26079136690647;
	private static double hiLum = 163.61774744027304;
	public static double lastxAvg = 0;
	public static double maxDiffX = 0;
	public static int vLineSet = 0; // How many sets of vertical lines are observed
	public static double isSameLine = 0; // Pixels between vertical lines to still consider associated
	public static double cumulLen = 0; // Running cumulative length of the group of lines
	public static double lastAdjVerticalline = 0; // Position of the preceding vertical line
	public static double firstAdjVerticalline = 0; // Position of the first line in the grouping
	public static int tgt1LeftXPtr = 0;
	public static int tgt1RightXPtr = 0;
	public static int tgt2LeftXPtr = 0;
	public static int tgt2RightXPtr = 0;
	public static double nomXTgt1L = 0;
	public static double nomXTgt1R = 0;
	public static double nomXTgt2L = 0;
	public static double nomXTgt2R = 0;
	
	
	public static void main(String[] args) throws Exception {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		Mat image = new Mat();
		
		VideoCapture camera = null;
		
		if (useVIDEO) {
			camera = new VideoCapture(1);
	    	if(!camera.isOpened()){
				throw new Exception("Can't open the camera.");
	    	}
		}
		
		//Network Table Code
		/*
		NetworkTable.setClientMode();
		NetworkTable.setIPAddress("127.0.0.1");
		NetworkTable table = NetworkTable.getTable("RobotTbl");
		table.putNumber("Distance", 30);
		*/
		
		for (int i = 0; i < 1; i++){
			if (!useVIDEO) {
				String jpgFile = new String("LTGym3ft.jpg");
				//String jpgFile = new String("LTGym6f45d.jpg");
				//String jpgFile = new String("LTGym6f70d.jpg");
				//String jpgFile = new String("LTGym8ft.jpg");
				//String jpgFile = new String("LTGym18ft.jpg");  // No lines found
				

				// Load a test image from file for development
				image = Imgcodecs.imread(jpgFile);

			} 
			else
				camera.read(image);
			
			// Having selected a source, process the image
			processSingleImage(image);
		}
		if (useVIDEO)
			camera.release();
	}

	private static void processSingleImage(Mat image) throws IOException, Exception {

		// Using the class Pipeline, instantiate here with the specified image
		// (replicate the class here)
		GripPipelineGym gp = new GripPipelineGym();
		gp.process(image, loHue, hiHue, loSat, hiSat, loLum, hiLum);

		// Create a List (special Java Collection) of "line" type entries from
		// the specified image processing class
		ArrayList<Line> lines = gp.findLinesOutput();
		
		ocvLineCount = lines.size();
		showImageInfo(image);
		
		double[] xAvgDiff = new double[ocvLineCount];
		String[] edgeID = new String[ocvLineCount];

		// Initialize the string array
		for (int x = 0; x < ocvLineCount; x++) {
			edgeID[x] = "";
		}

		TargetLine[] targetLines = new TargetLine[ocvLineCount];

		// Create a list of line parameters that we need to sort as a group,
		// like a row of values in a spreadsheet
		for (int x = 0; x < ocvLineCount; x++) {
			Line currentLine = gp.findLinesOutput().get(x);
			targetLines[x] = new TargetLine(currentLine);
		}

		sortTargetlinesX(targetLines);
		exportLineData(targetLines);
		outputOverlayedLines(image, targetLines);
		calcXAvgDiff(xAvgDiff, targetLines);
		
		// Sum the line lengths for grouped lines

		calcIsSameLine();

		double[] totalizedVerticalLen = new double[ocvLineCount]; // An array of totalized line lengths, though we expect fewer entries than allocated
		double[] nominalVerticalLineX = new double[ocvLineCount]; // Nominal x coordinate of the particular vertical line

		groupVerticalLines(xAvgDiff, targetLines, totalizedVerticalLen, nominalVerticalLineX);
		
		// Find the longest contiguous chain of lines for each vertical line group identified
		double[] yMinVerticalLine = new double[ocvLineCount]; // Minimum y coordinate of the particular vertical line
		double[] yMaxVerticalLine = new double[ocvLineCount]; // Maximum y coordinate of the particular vertical line
		double[] yMinVerticalLineSet = new double[ocvLineCount]; // Minimum y coordinate of the particular vertical line set
		double[] yMaxVerticalLineSet = new double[ocvLineCount]; // Maximum y coordinate of the particular vertical line set
		
		findLongestContiguousVLines(targetLines, nominalVerticalLineX, yMinVerticalLine, yMaxVerticalLine, yMinVerticalLineSet, yMaxVerticalLineSet);
		findFourBestVLines(totalizedVerticalLen, nominalVerticalLineX);
		
		// Find the horizontal lines of the targets
		double[] xMinHorizontalLine = new double[ocvLineCount]; // Minimum y coordinate of the particular vertical line
		double[] xMaxHorizontalLine = new double[ocvLineCount]; // Maximum y coordinate of the particular vertical line
		double[] xMinHorizontalLineSet = new double[ocvLineCount]; // Minimum y coordinate of the particular vertical line set
		double[] xMaxHorizontalLineSet = new double[ocvLineCount]; // Maximum y coordinate of the particular vertical line set
		
		findHorizontalLines(edgeID, targetLines, nominalVerticalLineX, yMinVerticalLineSet, yMaxVerticalLineSet,
				xMinHorizontalLine, xMaxHorizontalLine, xMinHorizontalLineSet, xMaxHorizontalLineSet);
		
		// Put the generated image back out to a file
		Imgcodecs.imwrite("RDW2619.jpg", gp.hslThresholdOutput());

		// Get the accompanying horizontal lines in order to validate the height of the rectangles
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			
		}
		
		// findlines experiment focusing in particular on vertical lines
		
		// Houghlines (standard) experiment

		// We now want to get (8) x,y pairs for each of the (8) lines, 4 horizontal and 4 vertical
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			// First for the vertical lines
			for (int zLpCtr2 = 0; zLpCtr2 < 4; zLpCtr2++) {
				
			}
			
			// Then for the horizontal lines
			for (int zLpCtr2 = 0; zLpCtr2 < 4; zLpCtr2++) {
				
			}
			
		}
		
		// Save our line data out to a file
		
		PrintWriter outputStream = null;
		try {
			String LineOut;
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
		String LineOut;
		outputStream = new PrintWriter(new FileWriter("RsltLineOutput.txt"));

		outputStream.println("TtlVLen,VXCoord,VYmin,VYmax,HXmin,HXmax");
		for (int linecount = 0; linecount <= vLineSet; linecount++) {
			LineOut = Double.toString(totalizedVerticalLen[linecount]);
			LineOut += "," + Double.toString(nominalVerticalLineX[linecount]);
			LineOut += "," + Double.toString(yMinVerticalLineSet[linecount]);
			LineOut += "," + Double.toString(yMaxVerticalLineSet[linecount]);
			LineOut += "," + Double.toString(xMinHorizontalLineSet[linecount]);
			LineOut += "," + Double.toString(xMaxHorizontalLineSet[linecount]);
			outputStream.println(LineOut);
			// }
		}

	} finally {
		if (outputStream != null) {
			outputStream.close();
		}
	}
	
	// Note:  This will have to be corrected as it currently assumes a 90 degree angle of incidence
	halfFoView = 0.5 * TARGET_WIDTH * pxlWidth / (nomXTgt2R - nomXTgt1L);
	dist2Target = halfFoView / TAN_HALF_FIELD_ANGLE;
	System.out.println("The estimated distance to the target (in inches) is " + Double.toString(dist2Target));
	}

	private static void findHorizontalLines(String[] edgeID, TargetLine[] targetLines, double[] nominalVerticalLineX,
			double[] yMinVerticalLineSet, double[] yMaxVerticalLineSet, double[] xMinHorizontalLine,
			double[] xMaxHorizontalLine, double[] xMinHorizontalLineSet, double[] xMaxHorizontalLineSet) {
		double nomYTgtTop = 0;
		double nomYTgtBtm = 0;
		double testX1 = 0;
		double testX2 = 0;
		double okHLGap = 12;


		// Initialize the arrays for the x max and min for the horizontal lines
		for (int x = 0; x <= vLineSet; x ++) {
			xMinHorizontalLineSet[x] = pxlWidth;
			xMaxHorizontalLineSet[x] = 0;
		}
		
 		// Note the nominal target top and bottom values (calculate a simple average from the vertical lines)
		for (int x = 0; x <= vLineSet; x ++) {
			nomYTgtBtm += yMinVerticalLineSet[x];
			nomYTgtTop += yMaxVerticalLineSet[x];
		}
		nomYTgtBtm = nomYTgtBtm/(vLineSet + 1);
		nomYTgtTop = nomYTgtTop/(vLineSet + 1);
		
		System.out.println("The top of the target is estimated at " + Double.toString(nomYTgtTop));
		System.out.println("The bottom of the target is estimated at " + Double.toString(nomYTgtBtm));
		System.out.println("The allowable error is " + Double.toString(okHLGap));
		
		// Note the nominal left and right target X values for each of the two targets
		nomXTgt1L = nominalVerticalLineX[tgt1LeftXPtr];
		System.out.println("Left edge of target 1 has x = " + Double.toString(nomXTgt1L));
		nomXTgt1R = nominalVerticalLineX[tgt1RightXPtr];
		System.out.println("Right edge of target 1 has x = " + Double.toString(nomXTgt1R));
		nomXTgt2L = nominalVerticalLineX[tgt2LeftXPtr];
		System.out.println("Left edge of target 2 has x = " + Double.toString(nomXTgt2L));
		nomXTgt2R = nominalVerticalLineX[tgt2RightXPtr];
		System.out.println("Right edge of target 2 has x = " + Double.toString(nomXTgt2R));
		System.out.println(" ");

		//int diffHLCount = 0;	// The count of the number of different horizontal lines in the group, hopefully 1
		// For each of the vertical lines, assess whether it is part of the top or the bottom of one of the rectangular targets
		for (int x = 0; x < ocvLineCount; x++) {
			// No processing required for lines that aren't horizontal
			if (targetLines[x].isHorizontal()) {
				// For each of the grouped vertical line pairs, look for the target tops and bottoms
				
				System.out.println("Evaluating horizontal line " + Integer.toString(x));

				// Having found an(other) line within the line group, get the min and max and see if we can validate the target's horizontal lines
				if (targetLines[x].ocvX1 < targetLines[x].ocvX2) {
					testX1 = targetLines[x].ocvX1;
					testX2 = targetLines[x].ocvX2;
				} else {	
					testX1 = targetLines[x].ocvX2;
					testX2 = targetLines[x].ocvX1;
				}	
				
				// Because the line is roughly horizontal, it doesn't matter much whether we choose Y1 or Y2
				if ((targetLines[x].ocvY1 < (nomYTgtTop + okHLGap)) && (targetLines[x].ocvY1 > (nomYTgtTop - okHLGap))) {
					System.out.println("Initial top alignment for line " + Integer.toString(x));
					
					// Assess whether we're looking at the left or the right target for this line
					if ((testX1 > (nomXTgt1L - okHLGap)) && (testX2 < (nomXTgt1R + okHLGap))) {
						// Top left horizontal target line
						edgeID[x] = "1HT";
						xMinHorizontalLine[1] = testX1;
						xMaxHorizontalLine[1] = testX2;
						if (testX1 < xMinHorizontalLineSet[1]) {
							xMinHorizontalLineSet[1] = testX1;
						}
						if (testX2 > xMaxHorizontalLineSet[1]) {
							xMaxHorizontalLineSet[1] = testX2;
						}
						
					} else if ((testX1 > (nomXTgt2L - okHLGap)) && (testX2 < (nomXTgt2R + okHLGap))) {
						// Top right horizontal target line
						edgeID[x] = "2HT";
						xMinHorizontalLine[3] = testX1;
						xMaxHorizontalLine[3] = testX2;
						if (testX1 < xMinHorizontalLineSet[3]) {
							xMinHorizontalLineSet[3] = testX1;
						}
						if (testX2 > xMaxHorizontalLineSet[3]) {
							xMaxHorizontalLineSet[3] = testX2;
						}
						
					}
					
				} else if ((targetLines[x].ocvY1 < (nomYTgtBtm + okHLGap)) && (targetLines[x].ocvY1 > (nomYTgtBtm - okHLGap))) {
					System.out.println("Initial bottom alignment for line " + Integer.toString(x));

					// Assess whether we're looking at the left or the right target for this line
					if ((testX1 > (nomXTgt1L - okHLGap)) && (testX2 < (nomXTgt1R + okHLGap))) {
						// Bottom left horizontal target line
						edgeID[x] = "1HB";
						xMinHorizontalLine[0] = testX1;
						xMaxHorizontalLine[0] = testX2;
						if (testX1 < xMinHorizontalLineSet[0]) {
							xMinHorizontalLineSet[0] = testX1;
						}
						if (testX2 > xMaxHorizontalLineSet[0]) {
							xMaxHorizontalLineSet[0] = testX2;
						}
						
					} else if ((testX1 > (nomXTgt2L - okHLGap)) && (testX2 < (nomXTgt2R + okHLGap))) {
						// Bottom right horizontal target line
						edgeID[x] = "2HB";
						xMinHorizontalLine[2] = testX1;
						xMaxHorizontalLine[2] = testX2;
						if (testX1 < xMinHorizontalLineSet[2]) {
							xMinHorizontalLineSet[2] = testX1;
						}
						if (testX2 > xMaxHorizontalLineSet[2]) {
							xMaxHorizontalLineSet[2] = testX2;
						}
						
					}
					
				}
			} else if (targetLines[x].isVertical()) {
				// Make the vertical line associations having identified the four verticals of interest
				if ((targetLines[x].xAvg < (nominalVerticalLineX[0] + isSameLine)) && (targetLines[x].xAvg > (nominalVerticalLineX[0] - isSameLine))) {
					edgeID[x] = "1VL";
				} else if ((targetLines[x].xAvg < (nominalVerticalLineX[1] + isSameLine)) && (targetLines[x].xAvg > (nominalVerticalLineX[1] - isSameLine))) {
					edgeID[x] = "1VR";
				} else if ((targetLines[x].xAvg < (nominalVerticalLineX[2] + isSameLine)) && (targetLines[x].xAvg > (nominalVerticalLineX[2] - isSameLine))) {
					edgeID[x] = "2VL";
				} else if ((targetLines[x].xAvg < (nominalVerticalLineX[3] + isSameLine)) && (targetLines[x].xAvg > (nominalVerticalLineX[3] - isSameLine))) {
					edgeID[x] = "2VR";
				}
			}
		}
	}

	private static void findFourBestVLines(double[] totalizedVerticalLen, double[] nominalVerticalLineX)
			throws Exception {
		double rectRatio = INCH_GAP_BETW / INCH_TGT_WIDE;
		double refRatio = 0;
		double lTgtAccrW = 0;
		double rTgtAccrW = 0;
		boolean spacedOK = false;
		double gap1 = 0;
		double gap2 = 0;
		double gap3 = 0;
		double lineWt = 0;
		double bestWt = 0;
		Integer[] vertSel = new Integer[4]; 
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
			rectRatio = (nominalVerticalLineX[tgt2LeftXPtr] - nominalVerticalLineX[tgt1RightXPtr]);
			rectRatio = rectRatio / (nominalVerticalLineX[tgt1RightXPtr] - nominalVerticalLineX[tgt1LeftXPtr]);
			lTgtAccrW = rectRatio / refRatio;
			System.out.println("The left target accuracy is 1 : " + Double.toString(lTgtAccrW));
			
			rectRatio = (nominalVerticalLineX[tgt2LeftXPtr] - nominalVerticalLineX[tgt1RightXPtr]);
			rectRatio = rectRatio / (nominalVerticalLineX[tgt2RightXPtr] - nominalVerticalLineX[tgt2LeftXPtr]);
			lTgtAccrW = rectRatio / refRatio;
			System.out.println("The right target accuracy is 1 : " + Double.toString(lTgtAccrW));
			
		} else {
			// Find the three sets of signals that yield the best 6.25 / 2 spacing ratio
			for (int x = 0; x <= (vLineSet-3); x++) {
				for (int y = 1; y <= (vLineSet-2); y++) {
					for (int z = 2; z <= (vLineSet-1); z++) {
						for (int w = 3; w <= vLineSet; w++) {
							if ((x < y) && (y < z) && (z < w)){
								System.out.println("Assessing line set : " + Integer.toString(x) + ":" + Integer.toString(y) + ":" + Integer.toString(z) + ":" + Integer.toString(w));
								spacedOK = false;
								lineWt = 0;
								gap1 = (nominalVerticalLineX[y] - nominalVerticalLineX[x]);
								gap2 = (nominalVerticalLineX[z] - nominalVerticalLineX[y]);
								gap3 = (nominalVerticalLineX[w] - nominalVerticalLineX[z]);
								System.out.println("Assessing gap of : " + Double.toString(gap1));
								System.out.println("Assessing gap of : " + Double.toString(gap2));
								System.out.println("Assessing gap of : " + Double.toString(gap3));
								if (((gap3 / gap1) < 1.5) && ((gap1 / gap3) < 1.5)) {
									if (((gap2 / gap1) > 3) && ((gap2 / gap3) > 3)) {
										if (((gap2 / gap1) < 10) && ((gap2 / gap3) < 10)) {
											spacedOK = true;
											lineWt = totalizedVerticalLen[x] + totalizedVerticalLen[y] + totalizedVerticalLen[z];
											if (lineWt > bestWt) {
												bestWt = lineWt;
												vertSel[0] = x;
												vertSel[1] = y;
												vertSel[2] = z;
												vertSel[3] = w;
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
		System.out.println();
	}

	private static void findLongestContiguousVLines(TargetLine[] targetLines, double[] nominalVerticalLineX,
			double[] yMinVerticalLine, double[] yMaxVerticalLine, double[] yMinVerticalLineSet,
			double[] yMaxVerticalLineSet) {
		double testY1 = 0;
		double testY2 = 0;
		double okVLGap = 12;
		int diffVLCount = 0;	// The count of the number of different vertical lines in the group, hopefully 1
		boolean wasAppd = false;
		
		// For each vertical line group, find the longest contiguous series of associated segments (ideally 1 series)
		for (int x = 0; x <= vLineSet; x++) {
			System.out.println("Evaluating vertical line group " + Integer.toString(x));
			diffVLCount = 0;
			// For each original line relating to this line group, append as able
			for (int y = 0; y < ocvLineCount; y++) {
				if (targetLines[y].isVertical()) {
					if ((targetLines[y].xAvg >= nominalVerticalLineX[x] - isSameLine) && (targetLines[y].xAvg <= nominalVerticalLineX[x] + isSameLine)) {
						// At least for now, confirm that we're evaluating all of the appropriate vertical lines
						System.out.println("Evaluating for grouping vertical line " + Integer.toString(y));

						// Having found an(other) line within the line group, get the min and max and see if we can append it to one of the sets
						// or whether we possibly have to append the set
						if (targetLines[y].ocvY1 < targetLines[y].ocvY2) {
							testY1 = targetLines[y].ocvY1;
							testY2 = targetLines[y].ocvY2;
						} else {	
							testY1 = targetLines[y].ocvY2;
							testY2 = targetLines[y].ocvY1;
						}	
							
							
						if (diffVLCount == 0) {
							System.out.println("Initializing resulting vertical line " + Integer.toString(x));
							// Prepare either to compare this segment with other segments in hope of appending vertical lines...
							yMinVerticalLine[diffVLCount] = testY1;
							yMaxVerticalLine[diffVLCount] = testY2;
							// But capture the result in case there are no more lines to append.
							yMinVerticalLineSet[x] = testY1;	//yminVlineEvl[y];
							yMaxVerticalLineSet[x] = testY2;	//ymaxVlineEvl[y];
							diffVLCount ++;
						} else {
							// Assess whether this next segment is simply an extension of a previous segment
							wasAppd = false;
							for (int z = 0; z < diffVLCount; z++){
								if (testY2 > yMaxVerticalLine[z]) {
									if (testY1 < (yMaxVerticalLine[z] + okVLGap)) {
										// Append the two lines
										yMaxVerticalLine[z] = testY2;
										if (testY1 < yMinVerticalLine[z]) {
											// While we wouldn't expect this to ever happen, prepare for the unexpected
											yMinVerticalLine[z] = testY1;
										}
										wasAppd = true;
										
										// Assess whether we now have a new longest combined line segment at this x value
										if ((yMaxVerticalLine[z] - yMinVerticalLine[z]) > (yMaxVerticalLineSet[x] - yMinVerticalLineSet[x])) {
											yMinVerticalLineSet[x] = yMinVerticalLine[z];
											yMaxVerticalLineSet[x] = yMaxVerticalLine[z];
										}
										
										// Having appended the segments, evaluate the ability to append additional segments
										for (int zLpCtr4 = 0; zLpCtr4 < diffVLCount; zLpCtr4++) {
											if (zLpCtr4 != z) {
												// Possibly append this new grouped segment with other segments 
												// before or after in the array, retaining lower array position
												if (yMaxVerticalLine[zLpCtr4] > yMaxVerticalLine[z]) {
													if ((yMinVerticalLine[zLpCtr4] > (yMinVerticalLine[z] - okVLGap)) && (yMinVerticalLine[zLpCtr4] < (yMaxVerticalLine[z] + okVLGap))){
														// Append the lines
														if (z == (diffVLCount - 1)) {
															// Update at zLpCtr4 and eliminate at zLpCtr3
															if (yMinVerticalLine[z] < yMinVerticalLine[zLpCtr4]) {
																yMinVerticalLine[zLpCtr4] = yMinVerticalLine[z];
															}
															// Leave ymaxVlineEvl[zLpCtr4] alone
															yMinVerticalLine[z] = 0;
															yMaxVerticalLine[z] = 0;
															diffVLCount--; 
														} else if (zLpCtr4 == (diffVLCount - 1)) {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															yMaxVerticalLine[z] = yMaxVerticalLine[zLpCtr4];
															if (yMinVerticalLine[zLpCtr4] < yMinVerticalLine[z]) {
																yMinVerticalLine[z] = yMinVerticalLine[zLpCtr4];
															}
															yMinVerticalLine[zLpCtr4] = 0;
															yMaxVerticalLine[zLpCtr4] = 0;
															diffVLCount--; 
														} else {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															if (yMinVerticalLine[zLpCtr4] < yMinVerticalLine[z]) {
																yMinVerticalLine[z] = yMinVerticalLine[zLpCtr4];
															}
															// Leave ymaxVlineEvl[zLpCtr3] alone
															yMinVerticalLine[zLpCtr4] = 0;
															yMaxVerticalLine[zLpCtr4] = 0;
														}
														
														// Assess whether we now have a new longest combined line segment at this x value
														if ((yMaxVerticalLine[z] - yMinVerticalLine[z]) > (yMaxVerticalLineSet[x] - yMinVerticalLineSet[x])) {
															yMinVerticalLineSet[x] = yMinVerticalLine[z];
															yMaxVerticalLineSet[x] = yMaxVerticalLine[z];
														} else if ((yMaxVerticalLine[zLpCtr4] - yMinVerticalLine[zLpCtr4]) > (yMaxVerticalLineSet[x] - yMinVerticalLineSet[x])) {
															yMinVerticalLineSet[x] = yMinVerticalLine[zLpCtr4];
															yMaxVerticalLineSet[x] = yMaxVerticalLine[zLpCtr4];
														}
														
													}
												} else {
													if ((yMinVerticalLine[z] > (yMinVerticalLine[zLpCtr4] - okVLGap)) && (yMinVerticalLine[z] < (yMaxVerticalLine[zLpCtr4] + okVLGap))){
														// Append the lines
														if (z == (diffVLCount - 1)) {
															// Update at zLpCtr4 and eliminate at zLpCtr3
															if (yMinVerticalLine[z] < yMinVerticalLine[zLpCtr4]) {
																yMinVerticalLine[zLpCtr4] = yMinVerticalLine[z];
															}
															// Leave ymaxVlineEvl[zLpCtr4] alone
															yMinVerticalLine[z] = 0;
															yMaxVerticalLine[z] = 0;
															diffVLCount--; 
														} else if (zLpCtr4 == (diffVLCount - 1)) {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															yMaxVerticalLine[z] = yMaxVerticalLine[zLpCtr4];
															if (yMinVerticalLine[zLpCtr4] < yMinVerticalLine[z]) {
																yMinVerticalLine[z] = yMinVerticalLine[zLpCtr4];
															}
															yMinVerticalLine[zLpCtr4] = 0;
															yMaxVerticalLine[zLpCtr4] = 0;
															diffVLCount--; 
														} else {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															if (yMinVerticalLine[zLpCtr4] < yMinVerticalLine[z]) {
																yMinVerticalLine[z] = yMinVerticalLine[zLpCtr4];
															}
															// Leave ymaxVlineEvl[zLpCtr3] alone
															yMinVerticalLine[zLpCtr4] = 0;
															yMaxVerticalLine[zLpCtr4] = 0;
														}
														
														// Assess whether we now have a new longest combined line segment at this x value
														if ((yMaxVerticalLine[z] - yMinVerticalLine[z]) > (yMaxVerticalLineSet[x] - yMinVerticalLineSet[x])) {
															yMinVerticalLineSet[x] = yMinVerticalLine[z];
															yMaxVerticalLineSet[x] = yMaxVerticalLine[z];
														} else if ((yMaxVerticalLine[zLpCtr4] - yMinVerticalLine[zLpCtr4]) > (yMaxVerticalLineSet[x] - yMinVerticalLineSet[x])) {
															yMinVerticalLineSet[x] = yMinVerticalLine[zLpCtr4];
															yMaxVerticalLineSet[x] = yMaxVerticalLine[zLpCtr4];
														}
														
													}
												}
											}
										}
										
										// Exit the loop having appended the segments
										z = diffVLCount + 1;
									}
								} else {
									// So (testY2 <= ymaxVlineEvl[zLpCtr3])
									// Same logic but reversing the evaluation
									if (testY2 > (yMinVerticalLine[z] - okVLGap)) {
										// Append the two lines
										yMinVerticalLine[z] = testY1;
										wasAppd = true;
										
										// Assess whether we now have a new longest combined line segment at this x value
										if ((yMaxVerticalLine[z] - yMinVerticalLine[z]) > (yMaxVerticalLineSet[x] - yMinVerticalLineSet[x])) {
											yMinVerticalLineSet[x] = yMinVerticalLine[z];
											yMaxVerticalLineSet[x] = yMaxVerticalLine[z];
										}
																				
										// Having appended the segments, evaluate the ability to append additional segments
										for (int w = 0; w < diffVLCount; w++) {
											if (w != z) {
												// Possibly append this new grouped segment with other segments 
												// before or after in the array, retaining lower array position
												if (yMaxVerticalLine[w] > yMaxVerticalLine[z]) {
													if ((yMinVerticalLine[w] > (yMinVerticalLine[z] - okVLGap)) && (yMinVerticalLine[w] < (yMaxVerticalLine[z] + okVLGap))){
														// Append the lines
														if (z == (diffVLCount - 1)) {
															// Update at zLpCtr4 and eliminate at zLpCtr3
															if (yMinVerticalLine[z] < yMinVerticalLine[w]) {
																yMinVerticalLine[w] = yMinVerticalLine[z];
															}
															// Leave ymaxVlineEvl[zLpCtr4] alone
															yMinVerticalLine[z] = 0;
															yMaxVerticalLine[z] = 0;
															diffVLCount--; 
														} else if (w == (diffVLCount - 1)) {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															yMaxVerticalLine[z] = yMaxVerticalLine[w];
															if (yMinVerticalLine[w] < yMinVerticalLine[z]) {
																yMinVerticalLine[z] = yMinVerticalLine[w];
															}
															yMinVerticalLine[w] = 0;
															yMaxVerticalLine[w] = 0;
															diffVLCount--; 
														} else {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															if (yMinVerticalLine[w] < yMinVerticalLine[z]) {
																yMinVerticalLine[z] = yMinVerticalLine[w];
															}
															// Leave ymaxVlineEvl[zLpCtr3] alone
															yMinVerticalLine[w] = 0;
															yMaxVerticalLine[w] = 0;
														}
														
														// Assess whether we now have a new longest combined line segment at this x value
														if ((yMaxVerticalLine[z] - yMinVerticalLine[z]) > (yMaxVerticalLineSet[x] - yMinVerticalLineSet[x])) {
															yMinVerticalLineSet[x] = yMinVerticalLine[z];
															yMaxVerticalLineSet[x] = yMaxVerticalLine[z];
														} else if ((yMaxVerticalLine[w] - yMinVerticalLine[w]) > (yMaxVerticalLineSet[x] - yMinVerticalLineSet[x])) {
															yMinVerticalLineSet[x] = yMinVerticalLine[w];
															yMaxVerticalLineSet[x] = yMaxVerticalLine[w];
														}
														
													}
												} else {
													if ((yMinVerticalLine[z] > (yMinVerticalLine[w] - okVLGap)) && (yMinVerticalLine[z] < (yMaxVerticalLine[w] + okVLGap))){
														// Append the lines
														if (z == (diffVLCount - 1)) {
															// Update at zLpCtr4 and eliminate at zLpCtr3
															if (yMinVerticalLine[z] < yMinVerticalLine[w]) {
																yMinVerticalLine[w] = yMinVerticalLine[z];
															}
															// Leave ymaxVlineEvl[zLpCtr4] alone
															yMinVerticalLine[z] = 0;
															yMaxVerticalLine[z] = 0;
															diffVLCount--; 
														} else if (w == (diffVLCount - 1)) {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															yMinVerticalLine[z] = yMinVerticalLine[w];
															if (yMaxVerticalLine[w] > yMaxVerticalLine[z]) {
																yMaxVerticalLine[z] = yMaxVerticalLine[w];
															}
															yMinVerticalLine[w] = 0;
															yMaxVerticalLine[w] = 0;
															diffVLCount--; 
														} else {
															// Update at zLpCtr3 and eliminate at zLpCtr4
															if (yMinVerticalLine[w] < yMinVerticalLine[z]) {
																yMinVerticalLine[z] = yMinVerticalLine[w];
															}
															// Leave ymaxVlineEvl[zLpCtr3] alone
															yMinVerticalLine[w] = 0;
															yMaxVerticalLine[w] = 0;
														}
														
														// Assess whether we now have a new longest combined line segment at this x value
														if ((yMaxVerticalLine[z] - yMinVerticalLine[z]) > (yMaxVerticalLineSet[x] - yMinVerticalLineSet[x])) {
															yMinVerticalLineSet[x] = yMinVerticalLine[z];
															yMaxVerticalLineSet[x] = yMaxVerticalLine[z];
														} else if ((yMaxVerticalLine[w] - yMinVerticalLine[w]) > (yMaxVerticalLineSet[x] - yMinVerticalLineSet[x])) {
															yMinVerticalLineSet[x] = yMinVerticalLine[w];
															yMaxVerticalLineSet[x] = yMaxVerticalLine[w];
														}
														
													}
												}
											}
										}
										
										// Exit the loop having appended the necessary segments together
										z = diffVLCount + 1;
									}
								}
							}	
							if (!wasAppd) {
								// Record them separately for now
								yMinVerticalLine[diffVLCount] = testY1;
								yMaxVerticalLine[diffVLCount] = testY2;
								diffVLCount ++;
							}
						}
					}
				}
			}
		}
		
		System.out.println();
	}

	private static void groupVerticalLines(double[] xAvgDiff, TargetLine[] targetLines, double[] totalizedVerticalLen,
			double[] nominalVerticalLineX) throws Exception {
		// If the first line is vertical, initialize the cumulative length to
		// this length and note the x value
		if (targetLines[0].isVertical()) {
			cumulLen = targetLines[0].length;
			lastAdjVerticalline = targetLines[0].xAvg;
			firstAdjVerticalline = targetLines[0].xAvg;
		}

		// Now analyze the rest of the lines
		for (int x = 1; x < ocvLineCount; x++) {

			// Note that we do nothing for non-vertical lines
			if (xAvgDiff[x] > isSameLine) {
				// The line is vertical and assessed to be the first in the next group of vertical lines
				// Capture the cumulative assessment of the line length
				totalizedVerticalLen[vLineSet] = cumulLen;

				// Reset the cumulative determination to the length of the next line
				cumulLen = targetLines[x].length;
				
				// Capture the nominal x coordinate
				nominalVerticalLineX[vLineSet] = (lastAdjVerticalline + firstAdjVerticalline) / 2;
				
				System.out.println("This line of length " + Double.toString(totalizedVerticalLen[vLineSet]));
				System.out.println("Its position is roughly " + Double.toString(nominalVerticalLineX[vLineSet]));
				
				// Capture the x coordinate for the first line in the new group
				firstAdjVerticalline = targetLines[x].xAvg;
				lastAdjVerticalline = targetLines[x].xAvg;
				
				// Increment the count of grouped lines
				vLineSet += 1;
			} 
			else if (targetLines[x].isVertical()) 
			{
				// The line is vertical but is close enough in proximity to
				// suggest it's the same line
				cumulLen += targetLines[x].length;
				System.out.println("Expanded to " + Double.toString(cumulLen));
				lastAdjVerticalline = targetLines[x].xAvg;
				
				// If this happens to be the very first vertical line of the set, initialize that value as well
				if (firstAdjVerticalline == 0) {
					firstAdjVerticalline = lastAdjVerticalline;
				}
			}
			// Note:  In this loop, nothing happens for lines that aren't vertical
		}

		// We may need to record the length of the last line group
		if ((xAvgDiff[(ocvLineCount - 1)] <= isSameLine) || (!targetLines[ocvLineCount - 1].isVertical())) {
			totalizedVerticalLen[vLineSet] = cumulLen;
			
			// Capture the nominal x coordinate
			nominalVerticalLineX[vLineSet] = (lastAdjVerticalline + firstAdjVerticalline) / 2;
			
			System.out.println("The last line is of length " + Double.toString(totalizedVerticalLen[vLineSet]));
			System.out.println("Its position is roughly " + Double.toString(nominalVerticalLineX[vLineSet]));
		} 

		// Estimate the actual length of the lines identified
		
		System.out.println("The number of line sets is " + Integer.toString(vLineSet + 1));
		// What do we do if we don't find at least 4 lines
		if (vLineSet < 3) {
			throw new Exception("vLineSet is less than 3 (less than 4 vertical lines).");
		}
		System.out.println();
	}

	private static void calcIsSameLine() {
		// Use roughly +- 1/8" as the assumption that the lines represent a
		// group, assuming that the maximum gap between lines
		// corresponds with the 6.25" gap between targets.
		isSameLine = INCH_IS_SAME_LINE * maxDiffX / INCH_GAP_BETW;
		if (isSameLine < 2) {
			// It may not be reasonable to expect resolution beyond a couple of pixels
			isSameLine = 2;
		}
	}

	private static void calcXAvgDiff(double[] xAvgDiff, TargetLine[] targetLines) {
		// Determine which vertical lines probably group together based on spacing from
		// other vertical lines
		
		for (int x = 0; x < ocvLineCount; x++) {
			// System.out.println("Loop Count = " + Integer.toString(zLpCtr));
			if (targetLines[x].isVertical()) {
				// System.out.println("Boolean true for " +
				// Integer.toString(zLpCtr));
				if (lastxAvg > 0) {
					xAvgDiff[x] = targetLines[x].xAvg - lastxAvg;
					if (xAvgDiff[x] > maxDiffX) {
						maxDiffX = xAvgDiff[x];
					}
					System.out.println("Differential xAvg = " + Double.toString(xAvgDiff[x]));
				}
				// Note the xAvg value of the "previous" vertical line
				lastxAvg = targetLines[x].xAvg;
			}
		}
		System.out.println("The maximum differential was " + Double.toString(maxDiffX) + "\n");
	}

	private static void outputOverlayedLines(Mat image, TargetLine[] targetLines) {
		// Create a series of lines to overlay on the original image to assess the quality of the lines identified
		// Note the color spec for BGR rather than RGB
		for (int x = 0; x < ocvLineCount; x++){
			Imgproc.line(image, targetLines[x].point1(), targetLines[x].point2(), new Scalar(0,255,255), 1);
		}
		// Save a copy of the amended file with the identified lines
		Imgcodecs.imwrite("img_with_lines.jpg", image);
	}

	private static void exportLineData(TargetLine[] targetLines) throws IOException {
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
			if (outputStream != null)
				outputStream.close();
		}
	}

	private static void sortTargetlinesX(TargetLine[] targetLines) {
		// Sort the lines by the average X value of the lines with targetLines
		// as the sorted array
		Arrays.sort(targetLines);
		// TargetLine[] vertLines = (TargetLine[])
		// Arrays.stream(targetLines).filter(line->line.isVertical()).toArray();
	}

	// Capture the image dimensions
	private static void showImageInfo(Mat image) {
		pxlWidth = image.width();
		pxlHeight = image.height();
		System.out.println("xdim = " + pxlWidth);
		System.out.println("ydim = " + pxlHeight);
		System.out.println("Number of lines = " + ocvLineCount + "\n");
	}
}



