package com.thecharge;

import java.util.ArrayList;
import java.util.Arrays;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Random;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.core.Point;

import com.thecharge.GripPipelineGym.Line;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class HelloOCV {
	private static final boolean TROUBLESHOOTING_MODE = false;
	private static final boolean CALIBRATION_MODE = false;
	private static final boolean USE_VIDEO = true;
	private static final double INCH_GAP_BETW = 6.25; // Distance between reflective targets
	private static final double INCH_TGT_WIDE = 2; // Width of the reflective target
	private static final double INCH_TGT_HEIGHT = 5; // Height of the reflective target
	private static final double INCH_GND_TO_TGT = 10.75; // Distance from the ground to the bottom of the target
	private static final double HALF_FIELD_ANGLE_H = 34;	// Half of the angle of view for the camera in operation
	private static final double HALF_FIELD_ANGLE_V = 21.3;
	private static final double INCH_IS_SAME_LINE = 0.25;
	private static final double TARGET_WIDTH =  INCH_TGT_WIDE + INCH_GAP_BETW + INCH_TGT_WIDE;	// The width of the target in inches
	private static final double OK_HL_GAP = 12;
	private static final double RAD_TO_DEG = 57.29577951; // 360 / 2 / pi()
	private static int pxlWidth = 0;
	private static int pxlHeight = 0;
	private static int ocvLineCount = 0;
	private static int tgt1RightXPtr = 0;
	private static int tgt2LeftXPtr = 0;
	private static int tgt2RightXPtr = 0;
	private static double lTgtAccrW = 0;
	private static double rTgtAccrW = 0;
	private static int vLineSet = 0; // How many sets of vertical lines are observed
	private static int tgt1LeftXPtr = 0;
	private static int calibrPass = 0;
	private static int calibrPhase = 0;
	private static double calibrScore = 0;
	private static double halfFoViewH = 0;
	private static double halfFoViewV = 0;
	private static double tanHlfAngleH = Math.tan(Math.toRadians(HALF_FIELD_ANGLE_H));
	private static double tanHlfAngleV = Math.tan(Math.toRadians(HALF_FIELD_ANGLE_V));
	private static double dist2Target = 0;	// Calculated distance to the target in inches
	private static final double INITIAL_LO_HUE = 74;
	private static final double INITIAL_HI_HUE = 96;	// 93.99317406143345;
	private static final double INITIAL_LO_SATURATION = 45.86330935251798;
	private static final double INITIAL_HI_SATURATION = 140;	//153;	// 128.80546075085323;
	private static final double INITIAL_LO_LUMIN = 80.26079136690647;
	private static final double INITIAL_HI_LUMIN = 163.61774744027304;
	private static double loHue = 81;
	private static double hiHue = 96;	// 93.99317406143345;
	private static double loSat = 45.86330935251798;
	private static double hiSat = 140;	//153;	// 128.80546075085323;
	private static double loLum = 80.26079136690647;
	private static double hiLum = 163.61774744027304;
	private static double lastxAvg = 0;
	private static double maxDiffX = 0;
	private static double isSameLine = 0; // Pixels between vertical lines to still consider associated
	private static double cumulLen = 0; // Running cumulative length of the group of lines
	private static double lastAdjVerticalline = 0; // Position of the preceding vertical line
	private static double firstAdjVerticalline = 0; // Position of the first line in the grouping
	private static double nomXTgt1L = 0;
	private static double nomXTgt1R = 0;
	private static double nomXTgt2L = 0;
	private static double nomXTgt2R = 0;
	private static double[] xAtYfind = new double[12];
	private static double[] yAtYfind = new double[12];
	private static double[] y1AtYfind = new double[12];
	private static double stdErr = 0;		// Relative to the upcoming linefit we want to calculate standard error for an indication of quality
	private static double stdErrT = 0;
	private static double stdErrB = 0;
	private static double incrX = 0;
	private static double nomYTgtTop = 0;
	private static double nomYTgtBtm = 0;
	private static double testX1 = 0;
	private static double testX2 = 0;
	private static double estTgtW = 0;
	private static double obsTgtW = 0;
	private static double angOfIncT = 0;	// Angle that the target is rotated relative to perpendicular
	private static double angOfIncR = 0;	// Angle that the target is positioned relative to the orientation of the robot
	private static double topSlope = 0;
	private static double topIntercept = 0;
	private static double yAtXFitTL = 0;
	private static double yAtXFitTR = 0;
	private static double yFit = 0;
	private static double bottomSlope = 0;
	private static double bottomIntercept = 0;
	private static double yAtXFitBL = 0;
	private static double yAtXFitBR = 0;
	private static double optLoHue = 0;		// These are the determined optimal values during calibration
	private static double optHiHue = 0;
	private static double optLoSat = 0;
	private static double optHiSat = 0;
	private static double optLoLum = 0;
	private static double optHiLum = 0;
	private static double atOptLineCount = 0;	//  The ocvLineCount value at the optimum tuning
	//private static double atOptLoHue = 0;
	//private static double atOptHiHue = 0;
	//private static double atOptLoSat = 0;
	//private static double atOptHiSat = 0;
	//private static double atOptLoLum = 0;
	//private static double atOptHiLum = 0;
	private static double atOptVLnGr = 0;
	private static double atOptLFStErrB = 0;
	private static double atOptLFStErrT = 0;
	private static double atOptLTgtWdAccr = 0;
	private static double atOptRTgtWdAccr = 0;
	private static String jpgFile = "";
	private static double imageQuality = 0;
	private static double optimumQuality = 0;
	private static double lastScore = 0;
	private static long executionCount = 0;
	
	
	public static void main(String[] args) throws Exception {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		Mat image = new Mat();
		
		VideoCapture camera = null;
		
		if (USE_VIDEO) {
			camera = new VideoCapture(0);
	    	if(!camera.isOpened()){
				throw new Exception("Can't open the camera.");
	    	}
		}
		
		//Network Table Setup
		NetworkTable.setClientMode();
		//NetworkTable.setIPAddress("127.0.0.1");
		NetworkTable.setIPAddress("10.26.19.2");
		NetworkTable table2Rbt = NetworkTable.getTable("Distance");
		
		if (CALIBRATION_MODE) {
			calibrPass = 0;
		} else {
			calibrPass = 99;	// Typically we use 99 to execute one time;
		}
		
		do {
			if (!USE_VIDEO) {
				
				// While normally not executed, here's where we choose alternate hue / saturation / luminance values for analysis
				selectNextParameterSet();
				
				// Load a test image from file for development
				image = Imgcodecs.imread(jpgFile);

			} 
			else
				camera.read(image);
			
			// Having selected a source, process the image (this is the dominant call)
			processSingleImage(image);
			
			// Having concluded analysis, update the Network Tables
			table2Rbt.putNumber("Distance", dist2Target);
			table2Rbt.putNumber("RobotAngle", angOfIncR);
			table2Rbt.putNumber("TargetAngle", angOfIncT);
			table2Rbt.putNumber("Quality", imageQuality);
			table2Rbt.putNumber("ImageCount", executionCount);
			
			// Moving to continuous mode (or even calibration, some variables will need to be reset
			initializeForNextImage();
			executionCount ++;
			
		} while ((calibrPass < 99) || (USE_VIDEO));
			
		if (USE_VIDEO)
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
		
		// Continue processing only if we're getting an image
		ocvLineCount = lines.size();
		if (ocvLineCount > 3) {
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
			
			findHorizontalLines(edgeID, targetLines, nominalVerticalLineX, yMinVerticalLineSet, yMaxVerticalLineSet, xMinHorizontalLine, xMaxHorizontalLine, xMinHorizontalLineSet, xMaxHorizontalLineSet);
			generateGripImage(gp);
			findLineFit(image, edgeID, targetLines);
			saveLineData(xAvgDiff, edgeID, targetLines, totalizedVerticalLen, nominalVerticalLineX, yMinVerticalLineSet, yMaxVerticalLineSet, xMinHorizontalLineSet, xMaxHorizontalLineSet);
			printLineFit();
			calcAngOfIncR();
			calcTargetDistance();
			
			// For the case where we want to analyze a series of tuning choices, record the results to a file
			recordAnalysisResults();
			
		} else {
			dist2Target = 0;
			angOfIncT = 0;
			angOfIncR = 0;
			imageQuality = 0;
		}
	}

	private static void calcAngOfIncR() {
		/*
	    Point ptTL = new Point(nomXTgt1L, yAtXFitTL);
	    Point ptTR = new Point(nomXTgt2R, yAtXFitTR);
	    Point ptBL = new Point(nomXTgt1L, yAtXFitBL);
	    Point ptBR = new Point(nomXTgt2R, yAtXFitBR);
	    */
		
		if ((yAtXFitTL - yAtXFitBL) > (yAtXFitTR - yAtXFitBR)) {
			
			// We are closer to the left edge of the target
			halfFoViewV = 0.5 * INCH_TGT_HEIGHT * pxlHeight / (yAtXFitTL - yAtXFitBL);
			dist2Target = halfFoViewV / tanHlfAngleV;
			System.out.println("The estimated distance to the target (in inches) is " + Double.toString(dist2Target));
			estTgtW = (TARGET_WIDTH / INCH_TGT_HEIGHT) * (yAtXFitTL - yAtXFitBL);
			
		} else {
			
			// We are closer to the right edge of the target
			halfFoViewV = 0.5 * INCH_TGT_HEIGHT * pxlHeight / (yAtXFitTR - yAtXFitBR);	// 0.5 * 5 * (1080 / 378)
			dist2Target = halfFoViewV / tanHlfAngleV;
			System.out.println("The estimated distance to the target (in inches) is " + Double.toString(dist2Target));
			estTgtW = (TARGET_WIDTH / INCH_TGT_HEIGHT) * (yAtXFitTR - yAtXFitBR);
			
		}
		
		//
		halfFoViewH = 0.5 * INCH_TGT_WIDE * pxlWidth / (nomXTgt2R - nomXTgt1L);
		// Numerator, center of target to center of view in pixels
		angOfIncR = ((nomXTgt2R + nomXTgt1L) / 2) - (pxlWidth / 2);	
		// Convert pixels to inches
		angOfIncR = angOfIncR * 2 * halfFoViewH / pxlWidth;
		// Determine the angle
		angOfIncR = Math.atan(angOfIncR / dist2Target);
		// Convert to degrees
		angOfIncR = RAD_TO_DEG * angOfIncR;
	}

	private static void initializeForNextImage() {
		// There are many variables that get initialized once for the first image but need re-initialized
		lastxAvg = 0;
		maxDiffX = 0;
		tgt1RightXPtr = 0;
		tgt2LeftXPtr = 0;
		tgt2RightXPtr = 0;
		lTgtAccrW = 0;
		rTgtAccrW = 0;
		vLineSet = 0; 
		tgt1LeftXPtr = 0;
		dist2Target = 0;
		lastxAvg = 0;
		maxDiffX = 0;
		isSameLine = 0;
		cumulLen = 0;
		lastAdjVerticalline = 0; 
		firstAdjVerticalline = 0;
		nomXTgt1L = 0;
		nomXTgt1R = 0;
		nomXTgt2L = 0;
		nomXTgt2R = 0;
		stdErr = 0;
		stdErrT = 0;
		stdErrB = 0;
		incrX = 0;
		nomYTgtTop = 0;
		nomYTgtBtm = 0;
		testX1 = 0;
		testX2 = 0;
		estTgtW = 0;
		obsTgtW = 0;
		angOfIncT = 0;
		angOfIncR = 0;
		topSlope = 0;
		topIntercept = 0;
		yAtXFitTL = 0;
		yAtXFitTR = 0;
		yFit = 0;
		bottomSlope = 0;
		bottomIntercept = 0;
		yAtXFitBL = 0;
		yAtXFitBR = 0;
	}
	
	private static void printLineFit() throws IOException {
		PrintWriter outputStream = null;
		try {
			
			String LineOut;
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
	}

	private static void selectNextParameterSet() {
		
		jpgFile = new String("LTGym3ft.jpg");
		//String jpgFile = new String("LTGym6f45d.jpg");
		//String jpgFile = new String("LTGym6f70d.jpg");
		//String jpgFile = new String("LTGym8ft.jpg");
		//String jpgFile = new String("LTGym18ft.jpg");  // No lines found
		//private static int calibrPass = 0;
		//private static boolean doCalibrate = false;
		//CalibrationData[] calibrRcd = new CalibrationData();

		// The first pass (analysisPassCount = 0), create a set of tests but take the use the defined initial set
		if (CALIBRATION_MODE) {
			if (calibrPass == 0) {
				loHue = INITIAL_LO_HUE;
				hiHue = INITIAL_HI_HUE;
				loSat = INITIAL_LO_SATURATION;
				hiSat = INITIAL_HI_SATURATION;
				loLum = INITIAL_LO_LUMIN;
				hiLum = INITIAL_HI_LUMIN;
				optLoHue = INITIAL_LO_HUE;
				optHiHue = INITIAL_HI_HUE;
				optLoSat = INITIAL_LO_SATURATION;
				optHiSat = INITIAL_HI_SATURATION;
				optLoLum = INITIAL_LO_LUMIN;
				optHiLum = INITIAL_HI_LUMIN;
			} else {
				// For all subsequent passes, instantiate the next set in the series
				if (calibrPhase == 0) {
					
					// See if narrowing the low Hue results in results that are better or worse than our best
					loHue = loHue + 1;
				} else if (calibrPhase == 1) {
					
					// See if expanding the low Hue results in results that are better or worse than our best
					if (loHue > INITIAL_LO_HUE) {
						loHue = INITIAL_LO_HUE - 1;
					} else {
						loHue -= 1;
					}
				} else if (calibrPhase == 2) {
					
					// See if expanding the high Hue results in results that are better or worse than our best
					if (loHue < INITIAL_LO_HUE) {
						loHue = INITIAL_LO_HUE;
					} 
					hiHue -= 1;
				} else {
					calibrPhase = 99;	// Signaling the end if calibration.  Save the result
				}
			}
		} else {
			loHue = INITIAL_LO_HUE;
			hiHue = INITIAL_HI_HUE;
			loSat = INITIAL_LO_SATURATION;
			hiSat = INITIAL_HI_SATURATION;
			loLum = INITIAL_LO_LUMIN;
			hiLum = INITIAL_HI_LUMIN;			
		}
		
		
		// This class needs to interact closely with recordAnalysisResults to assess whether the last test was better or worse
		
	}
	private static void recordAnalysisResults() throws IOException {
		// Here we record the quality of the analysis from the most recent image analysis
		imageQuality = 0;
		
		if (ocvLineCount > 8) {
			if (ocvLineCount < 50) {
				imageQuality += 0.3;
			} else {
				imageQuality += 0.1;
			}
		}
		
		if (vLineSet == 3) {
			imageQuality += 0.6;
		} else if (vLineSet == 4) {
			imageQuality += 0.45;
		} else if (vLineSet == 5) {
			imageQuality += 0.3;
		} else if (vLineSet == 6) {
			imageQuality += 0.2;
		} else if (vLineSet == 7) {
			imageQuality += 0.1;
		}
		
		imageQuality -= (stdErrT / 3);
		imageQuality -= (stdErrB / 4);
		
		imageQuality -= (lTgtAccrW - 1);
		imageQuality -= (rTgtAccrW - 1);
		
		if (imageQuality < 0) {
			imageQuality = 0;
		}
		
		if (imageQuality > 1) {
			imageQuality = 1;
		}
		
		if (CALIBRATION_MODE) {
			calibrScore = imageQuality;
			if (calibrPass == 0) {
				atOptVLnGr = vLineSet + 1;
				atOptLFStErrB = stdErrB;
				atOptLFStErrT = stdErrT;
				atOptLTgtWdAccr = lTgtAccrW;
				atOptRTgtWdAccr = rTgtAccrW;
				atOptLineCount = ocvLineCount;
								
				// Choose an efficient means to repetitively append an analysis file (create new here)
				try(PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter("calibr_data.txt", false)))) {
				    out.println("FileImage,loHue,hiHue,loSat,hiSat,loLum,hiLum,LineCount,VLnGrps,LnFitStErrB,LnFitStErrT,LTAccur,RTAccur,Dist,AngleT,AngleR,Score");
				}catch (IOException e) {
				    System.err.println(e);
				}			
				
				
			} else if (calibrPhase >= 99) {

				// Choose an efficient means to repetitively append an analysis file (append here)
				try(PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter("calibr_data.txt", true)))) {
					String LineOut;
					LineOut = jpgFile;
					LineOut += "," + Double.toString(loHue);
					LineOut += "," + Double.toString(hiHue);
					LineOut += "," + Double.toString(loSat);
					LineOut += "," + Double.toString(hiSat);
					LineOut += "," + Double.toString(loLum);
					LineOut += "," + Double.toString(hiLum);
					LineOut += "," + Integer.toString(ocvLineCount);
					LineOut += "," + Integer.toString(vLineSet + 1);
					LineOut += "," + Double.toString(stdErrB);
					LineOut += "," + Double.toString(stdErrT);	//lTgtAccrW
					LineOut += "," + Double.toString(lTgtAccrW);
					LineOut += "," + Double.toString(rTgtAccrW);
					LineOut += "," + Double.toString(dist2Target);
					LineOut += "," + Double.toString(angOfIncT);
					LineOut += "," + Double.toString(angOfIncR);
					LineOut += "," + Double.toString(calibrScore);
					out.println(LineOut);
				}catch (IOException e) {
				    System.err.println(e);
				}			
				
			} else {

				if (imageQuality > optimumQuality) {
					optimumQuality = imageQuality;
					optLoHue = loHue;
					optHiHue = hiHue;
					optLoSat = loSat;
					optHiSat = hiSat;
					optLoLum = loLum;
					optHiLum = hiLum;
					atOptVLnGr = vLineSet + 1;
					atOptLFStErrB = stdErrB;
					atOptLFStErrT = stdErrT;
					atOptLTgtWdAccr = lTgtAccrW;
					atOptRTgtWdAccr = rTgtAccrW;
					atOptLineCount = ocvLineCount;
				}

				// Choose an efficient means to repetitively append an analysis file (append here)
				try(PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter("calibr_data.txt", true)))) {
					String LineOut;
					LineOut = jpgFile;
					LineOut += "," + Double.toString(loHue);
					LineOut += "," + Double.toString(hiHue);
					LineOut += "," + Double.toString(loSat);
					LineOut += "," + Double.toString(hiSat);
					LineOut += "," + Double.toString(loLum);
					LineOut += "," + Double.toString(hiLum);
					LineOut += "," + Integer.toString(ocvLineCount);
					LineOut += "," + Integer.toString(vLineSet + 1);
					LineOut += "," + Double.toString(stdErrB);
					LineOut += "," + Double.toString(stdErrT);	//lTgtAccrW
					LineOut += "," + Double.toString(lTgtAccrW);
					LineOut += "," + Double.toString(rTgtAccrW);
					LineOut += "," + Double.toString(dist2Target);
					LineOut += "," + Double.toString(angOfIncT);
					LineOut += "," + Double.toString(angOfIncR);
					LineOut += "," + Double.toString(calibrScore);
					out.println(LineOut);
				}catch (IOException e) {
				    System.err.println(e);
				}			

			
			}
			// As a last step in the calibration pass, determine when we go to the next calibration phase
			if (calibrPass == 1) {
				lastScore = imageQuality;
			} else {
				if (imageQuality < lastScore) {
					calibrPhase ++;
					lastScore = 0;
				} else {
					lastScore = imageQuality;
				}
			}
			
			if ((calibrPhase == 1) && (loHue <= 1)) {
				calibrPhase ++;
				lastScore = 0;
			}
			
			if (calibrPass < 100) calibrPass ++;
		}
	}
	
	private static void findLineFit(Mat image, String[] edgeID, TargetLine[] targetLines) {
		// Get the accompanying horizontal lines in order to validate the height of the rectangles
		double mSlope = 0;		// slope of the line we use to fit the top of the target
		double yIntcpt = 0;		// y intercept for the line we use to fit to
		Random rand = new Random();
		if (TROUBLESHOOTING_MODE) System.out.println(" ");
		
		// Get linefit coordinates for the "top" horizontal lines of target 1
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "1HT") {
				if (TROUBLESHOOTING_MODE) System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
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
							if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		
		// Now get linefit coordinates for the top horizontal lines of target 2
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "2HT") {
				if (TROUBLESHOOTING_MODE) System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
				mSlope = (targetLines[zLpCtr1].ocvY2 - targetLines[zLpCtr1].ocvY1) / (targetLines[zLpCtr1].ocvX2 - targetLines[zLpCtr1].ocvX1);
				yIntcpt = targetLines[zLpCtr1].ocvY1 - mSlope * targetLines[zLpCtr1].ocvX1;
				if (TROUBLESHOOTING_MODE) System.out.println("Slope and Intercept as " + Double.toString(mSlope) + " / " + Double.toString(yIntcpt));
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
							if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		
		
		if (TROUBLESHOOTING_MODE) System.out.println(" ");
		
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
        topSlope = (ptCount * xybar) - (sumx * sumy);
        topSlope = topSlope / ((ptCount * xxbar) - (sumx * sumx));
        topIntercept = (ybar - topSlope * xbar);
        
        // At least while we want quality estimates, calculate error data from the linefit
        for (int i = 0; i < 12; i++) {
        	if (yAtYfind[i] > 0) {
        		yFit = topSlope * xAtYfind[i] + topIntercept;
        		y1AtYfind[i] += (yFit - yAtYfind[i]) * (yFit - yAtYfind[i]);
        		stdErrT = y1AtYfind[i] / ptCount;	// This will only be "correct" at the last point
        	}
        }
        stdErr = Math.sqrt(stdErrT);
        
		if (TROUBLESHOOTING_MODE) System.out.println("Determined slope / intercept of " + Double.toString(topSlope) + " / " + Double.toString(topIntercept));
        yAtXFitTL = topSlope * nomXTgt1L + topIntercept;
        yAtXFitTR = topSlope * nomXTgt2R + topIntercept;
		if (TROUBLESHOOTING_MODE) System.out.println("Top Left x / y of " + Double.toString(nomXTgt1L) + " / " + Double.toString(yAtXFitTL));
		if (TROUBLESHOOTING_MODE) System.out.println("Top Right x / y of " + Double.toString(nomXTgt2R) + " / " + Double.toString(yAtXFitTR));
		if (TROUBLESHOOTING_MODE) System.out.println(" ");
		
		// Repeat for the "bottom" line
		
		// Clear out the previous y values from the top line analysis (note that we reuse the x values)
		for (int i = 0; i < 12; i++) {
			yAtYfind[i] = 0;
			y1AtYfind[i] = 0;
		}
		// Get linefit coordinates for the "bottom" horizontal lines of target 1		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "1HB") {
				if (TROUBLESHOOTING_MODE) System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
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
							if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		
		// Now get linefit coordinates for the "bottom" horizontal lines of target 2
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "2HB") {
				if (TROUBLESHOOTING_MODE) System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
				mSlope = (targetLines[zLpCtr1].ocvY2 - targetLines[zLpCtr1].ocvY1) / (targetLines[zLpCtr1].ocvX2 - targetLines[zLpCtr1].ocvX1);
				yIntcpt = targetLines[zLpCtr1].ocvY1 - mSlope * targetLines[zLpCtr1].ocvX1;
				if (TROUBLESHOOTING_MODE) System.out.println("Slope and Intercept as " + Double.toString(mSlope) + " / " + Double.toString(yIntcpt));
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
							if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		
		if (TROUBLESHOOTING_MODE) System.out.println(" ");
		
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
        bottomSlope = (ptCount * xybar) - (sumx * sumy);					// Numerator
        bottomSlope = bottomSlope / ((ptCount * xxbar) - (sumx * sumx));	// Numerator / Denominator
        bottomIntercept = (ybar - bottomSlope * xbar);

        // At least while we want quality estimates, calculate error data from the linefit
        for (int i = 0; i < 12; i++) {
        	if (yAtYfind[i] > 0) {
        		yFit = bottomSlope * xAtYfind[i] + bottomIntercept;
        		y1AtYfind[i] += (yFit - yAtYfind[i]) * (yFit - yAtYfind[i]);
        		stdErrB = y1AtYfind[i] / ptCount;	// This will only be "correct" at the last point
        	}
        }
        stdErr += Math.sqrt(stdErrB);
        
		if (TROUBLESHOOTING_MODE) System.out.println("Determined slope / intercept of " + Double.toString(bottomSlope) + " / " + Double.toString(bottomIntercept));
        yAtXFitBL = bottomSlope * nomXTgt1L + bottomIntercept;
        yAtXFitBR = bottomSlope * nomXTgt2R + bottomIntercept;
        Point ptTL = new Point(nomXTgt1L, yAtXFitTL);
        Point ptTR = new Point(nomXTgt2R, yAtXFitTR);
        Point ptBL = new Point(nomXTgt1L, yAtXFitBL);
        Point ptBR = new Point(nomXTgt2R, yAtXFitBR);
		if (TROUBLESHOOTING_MODE) System.out.println("Bottom Left x / y of " + Double.toString(nomXTgt1L) + " / " + Double.toString(yAtXFitBL));
		if (TROUBLESHOOTING_MODE) System.out.println("Bottom Right x / y of " + Double.toString(nomXTgt2R) + " / " + Double.toString(yAtXFitBR));
		if (TROUBLESHOOTING_MODE) System.out.println(" ");

		// Update our picture with the new determined top line
		Imgproc.line(image, ptTL, ptTR, new Scalar(0,0,255), 1);
		Imgproc.line(image, ptBL, ptBR, new Scalar(0,0,255), 1);
		Imgproc.line(image, ptTL, ptBL, new Scalar(0,0,255), 1);
		Imgproc.line(image, ptBR, ptTR, new Scalar(0,0,255), 1);

		// Save a copy of the amended file with the identified lines
		Imgcodecs.imwrite("img_with_lines.jpg", image);
	}

	private static void calcTargetDistance() {
		// Note:  This will have to be corrected as it currently assumes a 90 degree angle of incidence
		obsTgtW = (nomXTgt2R - nomXTgt1L);
		angOfIncT = Math.acos(obsTgtW / estTgtW) * RAD_TO_DEG;
		
		System.out.println("The estimated angle of incidence (in degrees) for the target is " + Double.toString(angOfIncT));
		System.out.println("The estimated angle of incidence (in degrees) for the robot is " + Double.toString(angOfIncR));
		
		//table2Rbt.}
	}
	
	private static void saveLineData(double[] xAvgDiff, String[] edgeID, TargetLine[] targetLines,
			double[] totalizedVerticalLen, double[] nominalVerticalLineX, double[] yMinVerticalLineSet,
			double[] yMaxVerticalLineSet, double[] xMinHorizontalLineSet, double[] xMaxHorizontalLineSet)
			throws IOException {
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
	}

	private static void generateGripImage(GripPipelineGym gp) {
		// Put the generated image back out to a file
		Imgcodecs.imwrite("RDW2619.jpg", gp.hslThresholdOutput());
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
		
		if (TROUBLESHOOTING_MODE) System.out.println("The top of the target is estimated at " + Double.toString(nomYTgtTop));
		if (TROUBLESHOOTING_MODE) System.out.println("The bottom of the target is estimated at " + Double.toString(nomYTgtBtm));
		if (TROUBLESHOOTING_MODE) System.out.println("The allowable error is " + Double.toString(okHLGap));
		
		// Note the nominal left and right target X values for each of the two targets
		nomXTgt1L = nominalVerticalLineX[tgt1LeftXPtr];
		if (TROUBLESHOOTING_MODE) System.out.println("Left edge of target 1 has x = " + Double.toString(nomXTgt1L));
		nomXTgt1R = nominalVerticalLineX[tgt1RightXPtr];
		if (TROUBLESHOOTING_MODE) System.out.println("Right edge of target 1 has x = " + Double.toString(nomXTgt1R));
		nomXTgt2L = nominalVerticalLineX[tgt2LeftXPtr];
		if (TROUBLESHOOTING_MODE) System.out.println("Left edge of target 2 has x = " + Double.toString(nomXTgt2L));
		nomXTgt2R = nominalVerticalLineX[tgt2RightXPtr];
		if (TROUBLESHOOTING_MODE) System.out.println("Right edge of target 2 has x = " + Double.toString(nomXTgt2R));
		if (TROUBLESHOOTING_MODE) System.out.println(" ");


		// At this point we can capture 6 preferred points associated with each target's horizontal line fits
		incrX = (nomXTgt1R - nomXTgt1L) / 7;
		xAtYfind[0] = nomXTgt1L + incrX;
		yAtYfind[0] = 0;
		if (TROUBLESHOOTING_MODE) System.out.println("Find Y at X = " + xAtYfind[0]);
		for (int zLpCtr1 = 1; zLpCtr1 < 6; zLpCtr1++) {
			xAtYfind[zLpCtr1] = xAtYfind[zLpCtr1 - 1] + incrX;
			yAtYfind[zLpCtr1] = 0;
			if (TROUBLESHOOTING_MODE) System.out.println("Find Y at X = " + xAtYfind[zLpCtr1]);
		}
		
		incrX = (nomXTgt2R - nomXTgt2L) / 7;
		xAtYfind[6] = nomXTgt2L + incrX;
		yAtYfind[6] = 0;
		if (TROUBLESHOOTING_MODE) System.out.println("Find Y at X = " + xAtYfind[6]);
		for (int zLpCtr1 = 7; zLpCtr1 < 12; zLpCtr1++) {
			xAtYfind[zLpCtr1] = xAtYfind[zLpCtr1 - 1] + incrX;
			yAtYfind[zLpCtr1] = 0;
			if (TROUBLESHOOTING_MODE) System.out.println("Find Y at X = " + xAtYfind[zLpCtr1]);
		}
		if (TROUBLESHOOTING_MODE) System.out.println(" ");
		
		for (int x = 0; x < ocvLineCount; x++) {
			// No processing required for lines that aren't horizontal
			if (targetLines[x].isHorizontal()) {
				// For each of the grouped vertical line pairs, look for the target tops and bottoms
				
				if (TROUBLESHOOTING_MODE) System.out.println("Evaluating horizontal line " + Integer.toString(x));

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
					if (TROUBLESHOOTING_MODE) System.out.println("Initial top alignment for line " + Integer.toString(x));
					
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
					if (TROUBLESHOOTING_MODE) System.out.println("Initial bottom alignment for line " + Integer.toString(x));

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

	private static void findFourBestVLines(double[] totalizedVerticalLen, double[] nominalVerticalLineX) throws Exception {
		double rectRatio = 0;
		double refRatio = INCH_GAP_BETW / INCH_TGT_WIDE;
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
			if (TROUBLESHOOTING_MODE) System.out.println("The left target accuracy is 1 : " + Double.toString(lTgtAccrW));
			
			rectRatio = (nominalVerticalLineX[tgt2LeftXPtr] - nominalVerticalLineX[tgt1RightXPtr]);
			rectRatio = rectRatio / (nominalVerticalLineX[tgt2RightXPtr] - nominalVerticalLineX[tgt2LeftXPtr]);
			rTgtAccrW = rectRatio / refRatio;
			if (TROUBLESHOOTING_MODE) System.out.println("The right target accuracy is 1 : " + Double.toString(rTgtAccrW));
			
		} else {
			// Find the three sets of signals that yield the best 6.25 / 2 spacing ratio
			for (int x = 0; x <= (vLineSet-3); x++) {
				for (int y = 1; y <= (vLineSet-2); y++) {
					for (int z = 2; z <= (vLineSet-1); z++) {
						for (int w = 3; w <= vLineSet; w++) {
							if ((x < y) && (y < z) && (z < w)){
								if (TROUBLESHOOTING_MODE) System.out.println("Assessing line set : " + Integer.toString(x) + ":" + Integer.toString(y) + ":" + Integer.toString(z) + ":" + Integer.toString(w));
								spacedOK = false;
								lineWt = 0;
								gap1 = (nominalVerticalLineX[y] - nominalVerticalLineX[x]);
								gap2 = (nominalVerticalLineX[z] - nominalVerticalLineX[y]);
								gap3 = (nominalVerticalLineX[w] - nominalVerticalLineX[z]);
								if (TROUBLESHOOTING_MODE) System.out.println("Assessing gap of : " + Double.toString(gap1));
								if (TROUBLESHOOTING_MODE) System.out.println("Assessing gap of : " + Double.toString(gap2));
								if (TROUBLESHOOTING_MODE) System.out.println("Assessing gap of : " + Double.toString(gap3));
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
			
			if (TROUBLESHOOTING_MODE) System.out.println("Best weighted value : " + Double.toString(bestWt));
			
			if (bestWt == 0) {
				throw new Exception("vLineSet is perhaps greater than 4 and handling logic is required.");
			}
			if (TROUBLESHOOTING_MODE) System.out.println("Selecting verticals : " + Integer.toString(vertSel[0]) + ":" + Integer.toString(vertSel[1]) + ":" + Integer.toString(vertSel[2]) + ":" + Integer.toString(vertSel[3]));
			tgt1LeftXPtr = vertSel[0];
			tgt1RightXPtr = vertSel[1];
			tgt2LeftXPtr = vertSel[2];
			tgt2RightXPtr = vertSel[3];
		}
		if (TROUBLESHOOTING_MODE) System.out.println();
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
			if (TROUBLESHOOTING_MODE) System.out.println("Evaluating vertical line group " + Integer.toString(x));
			diffVLCount = 0;
			// For each original line relating to this line group, append as able
			for (int y = 0; y < ocvLineCount; y++) {
				if (targetLines[y].isVertical()) {
					if ((targetLines[y].xAvg >= nominalVerticalLineX[x] - isSameLine) && (targetLines[y].xAvg <= nominalVerticalLineX[x] + isSameLine)) {
						// At least for now, confirm that we're evaluating all of the appropriate vertical lines
						if (TROUBLESHOOTING_MODE) System.out.println("Evaluating for grouping vertical line " + Integer.toString(y));

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
							if (TROUBLESHOOTING_MODE) System.out.println("Initializing resulting vertical line " + Integer.toString(x));
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
		
		if (TROUBLESHOOTING_MODE) System.out.println();
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
				
				if (TROUBLESHOOTING_MODE) System.out.println("This line of length " + Double.toString(totalizedVerticalLen[vLineSet]));
				if (TROUBLESHOOTING_MODE) System.out.println("Its position is roughly " + Double.toString(nominalVerticalLineX[vLineSet]));
				
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
				if (TROUBLESHOOTING_MODE) System.out.println("Expanded to " + Double.toString(cumulLen));
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
			
			if (TROUBLESHOOTING_MODE) System.out.println("The last line is of length " + Double.toString(totalizedVerticalLen[vLineSet]));
			if (TROUBLESHOOTING_MODE) System.out.println("Its position is roughly " + Double.toString(nominalVerticalLineX[vLineSet]));
		} 

		// Estimate the actual length of the lines identified
		
		if (TROUBLESHOOTING_MODE) System.out.println("The number of line sets is " + Integer.toString(vLineSet + 1));
		// What do we do if we don't find at least 4 lines
		//TODO: Account for a situation where there are less than 4 vertical line sets
		if (vLineSet < 3) {
			//throw new Exception("vLineSet is less than 3 (less than 4 vertical lines).");
			if (dist2Target < .5) {
				//DriveXFeet(.5)
			}
			else{
				//Adjust the robot
			}
		}
		if (TROUBLESHOOTING_MODE) System.out.println();
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
					if (TROUBLESHOOTING_MODE) System.out.println("Differential xAvg = " + Double.toString(xAvgDiff[x]));
				}
				// Note the xAvg value of the "previous" vertical line
				lastxAvg = targetLines[x].xAvg;
			}
		}
		if (TROUBLESHOOTING_MODE) System.out.println("The maximum differential was " + Double.toString(maxDiffX) + "\n");
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
		if (TROUBLESHOOTING_MODE) System.out.println("xdim = " + pxlWidth);
		if (TROUBLESHOOTING_MODE) System.out.println("ydim = " + pxlHeight);
		if (TROUBLESHOOTING_MODE) System.out.println("Number of lines = " + ocvLineCount + "\n");
	}
}



