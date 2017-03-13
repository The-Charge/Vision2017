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
import org.opencv.core.Size;
import org.opencv.core.Scalar;
//import org.opencv.highgui.Highgui;        
//import org.opencv.highgui.VideoCapture;        
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;
import org.opencv.videoio.VideoCapture;
import org.opencv.core.Point;
import org.opencv.videoio.Videoio;

import com.thecharge.GripPipelineGym.Line;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;

public class HelloOCV {
	private static final boolean TROUBLESHOOTING_MODE = false;
	private static final boolean USE_VIDEO = true;
	private static final boolean JPGS_TO_C = false;
	private static final boolean WRITE_NETW_TBLS = false;
	private static final boolean INIT_WEBCAM_SETTINGS = false;
	private static final boolean CALIBRATION_MODE = false;
	private static boolean jpgMemMgmt = true;		// Recommended to be set true until the memory management issue is solved
	private static boolean analyzeCamera = false;	// Generate a CSV file tracking camera settings vs results
	private static final int MAX_CALIBR_PASS = 999;
	private static boolean userStop = false;
	private static final double INCH_GAP_BETW = 6.375; // Distance between reflective targets
	private static final double INCH_TGT_WIDE = 1.875; // Width of the reflective target
	private static final double INCH_TGT_HEIGHT = 5; // Height of the reflective target
	private static final double HALF_FIELD_ANGLE_H = 34;	// Half of the angle of view for the camera in operation
	private static final double HALF_FIELD_ANGLE_V = 20.59;	// 21.3
	private static final double INCH_IS_SAME_LINE = 0.25;
	private static final double ROBOT_TURN_RADIUS = 18;		// Preferred turn radius in inches for the robot
	private static final double DISTANCE_ON_COURSE = 24;	// Inches over which we prefer to approach the robot straight on
	private static final double TARGET_WIDTH =  INCH_TGT_WIDE + INCH_GAP_BETW + INCH_TGT_WIDE;	// The width of the target in inches
	private static final double RAD_TO_DEG = 57.29577951; // 360 / 2 / pi()
	private static int pxlWidth = 0;
	private static int pxlHeight = 0;
	private static int ocvLineCount = 0;
	private static int tgt1LeftXPtr = 0;
	private static int tgt1RightXPtr = 0;
	private static int tgt2LeftXPtr = 0;
	private static int tgt2RightXPtr = 0;
	private static double lTgtAccrW = 0;
	private static double rTgtAccrW = 0;
	private static int vLineSet = 0; // How many sets of vertical lines are observed
	private static int calibrPass = 0;
	private static int calibrPhase = 0;
	private static double calibrScore = 0;
	private static double halfFoViewH = 0;
	private static double halfFoViewV = 0;
	private static double tanHlfAngleH = Math.tan(Math.toRadians(HALF_FIELD_ANGLE_H));
	private static double tanHlfAngleV = Math.tan(Math.toRadians(HALF_FIELD_ANGLE_V));
	private static double pxlH2Vratio = 1.129265637;	// Carefully measuring a known sized target by inches then pixels
	private static double dist2Target = 0;	// Calculated distance to the target in inches
	private static final double INITIAL_LO_HUE = 86;	//83;	//88;	//40,65,68,32, 73;		//74;
	private static final double INITIAL_HI_HUE = 102;	//108;	//94;	//142,120,117, 103;	//96;	// 93.99317406143345;
	private static final double INITIAL_LO_SATURATION = 144;	//112;	//183;	//156,211, 14;	//40;	//45.86330935251798;
	private static final double INITIAL_HI_SATURATION = 255;	//255;	//250;	//255, 255;	//140;	//153;	// 128.80546075085323;
	private static final double INITIAL_LO_LUMIN = 44;	//71;	//26;	//89,99,66, 135;	//80.26079136690647;
	private static final double INITIAL_HI_LUMIN = 255;	//244;	//132;	//133,255,166, 235;	//163.61774744027304;
	//LTGym8ft => 81 / 114 / 7 / 140 / 85 / 254
	//BreakRoom => 71 / 110 / 17 / 253 / 12 / 255
	//LTGym6f70d.jpg => 83 / 102 / 57 / 255 / 71 / 185
	//BreakRoom0221 =>  73 / 171 / 3 / 61 / 96 / 181
	private static final double INITIAL_WC_BRIGHTNESS = 24; //34 worked well initially;	//48;	//0;	//128;
	private static final double INITIAL_WC_CONTRAST = 39;	//69;	//38;	//32;
	private static final double INITIAL_WC_EXPOSURE = -2;	//-2;
	private static final double INITIAL_WC_GAIN = 0;	//1;	//0;
	private static final double INITIAL_WC_SATURATION = 39;	//76;	//99;	//32;
	private static final double INITIAL_WC_WHTBALBLU = 6142;	//3311;	//6023;	//2800;
	private static final double INITIAL_WC_WHTBALRED = -1;	//-1;
	private static final double INITIAL_WC_HUE = 4.78E7;	//1.54E7;	//3.7E7;
	private static boolean lastTestInCalbrPh = false;
	private static double loHue = 0;	// 81 from optimization;
	private static double hiHue = 0;	// 93.99317406143345;
	private static double loSat = 0;	// 45.86330935251798;
	private static double hiSat = 0;	// 140;	//153;	// 128.80546075085323;
	private static double loLum = 0;	// 80.26079136690647;
	private static double hiLum = 0;	// 163.61774744027304;
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
	private static double[] xAtYfind = new double[32];	// 6 points from each of the horizontals; 4 top vertical; 4 bottom vertical
	private static double[] yAtYfind = new double[32];
	private static double[] y1AtYfind = new double[32];	
	private static double stdErr = 0;		// Relative to the upcoming linefit we want to calculate standard error for an indication of quality
	private static double stdErrT = 0;
	private static double stdErrB = 0;
	private static double incrX = 0;
	private static double minYGoodLine = 0;	// The minimum Y value observed for a line assessed to be part of the target
	private static double maxYGoodLine = 0;	// The maximum Y value observed for a line assessed to be part of the target
	private static double nomYTgtTop = 0;
	private static double nomYTgtBtm = 0;
	private static double testX1 = 0;
	private static double testX2 = 0;
	private static double estTgtW = 0;
	private static double obsTgtW = 0;
	private static double angOfIncT = 0;	// Angle that the target is rotated relative to perpendicular
	private static double angOfIncR = 0;	// Angle that the target is positioned relative to the orientation of the robot
	private static double angleTrajectory = 0;	// Calculated angle of trajectory for the robot
	private static double topSlope = 0;
	private static double topIntercept = 0;
	private static double yAtXFitTL = 0;
	private static double yAtXFitTR = 0;
	private static double yAtXFitBL = 0;
	private static double yAtXFitBR = 0;
	private static double yFit = 0;
	private static double bottomSlope = 0;
	private static double bottomIntercept = 0;
	private static double bestWt = 0;
	private static double optLoHue = 0;		// These are the determined optimal values during calibration
	private static double optHiHue = 0;
	private static double optLoSat = 0;
	private static double optHiSat = 0;
	private static double optLoLum = 0;
	private static double optHiLum = 0;
	private static double optBrightness = 0;
	private static double[] altWCBrightness = new double[8];
	private static double[] altWCContrast = new double[8];
	private static double atOptLineCount = 0;	//  The ocvLineCount value at the optimum tuning
	private static double atOptVLnGr = 0;
	private static double atOptLFStErrB = 0;
	private static double atOptLFStErrT = 0;
	private static double atOptLTgtWdAccr = 0;
	private static double atOptRTgtWdAccr = 0;
	private static String jpgFile = "";
	private static double imageQuality = 0;
	private static double optimumQuality = 0;
	private static double percHLFPts = 0;		// Percent of the horizontal line fit points that were found
	private static long executionCount = 0;
	private static int poorImageCount = 0;
	private static boolean revertToJPG = false;
	private static Mat hslTO;					// The hslThresholdOutput storage matrix
	private static Mat srcImage;				// The source image
	//private static Number hiPixelValue = 0;
	private static int webcamSettings = 0;
	private static VideoCapture camera = null;
	
	public static void main(String[] args) throws Exception {
		Double dist2TargetTemp = 0.0;
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		//System.loadLibrary("opencv_ffmpeg300_64");
		Mat image = new Mat();
		
		if (USE_VIDEO) {
			camera = new VideoCapture(0);
			//camera = new VideoCapture("GymCartApprch.wmv");
	    	if(!camera.isOpened()){
				System.out.println("The camera didn't open.");
				throw new Exception("Can't open the camera.");
	    	}
	    	
	    	// While we're trying to understand the camera behavior, generate an analysis file
			if ((analyzeCamera) && (USE_VIDEO)) {			
				// Choose an efficient means to repetitively append an analysis file (create new here)
				try(PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter("WebCamVRslt.csv", false)))) {
				    out.println("FPS,Brightness,Contrast,Exposure,Gain,Saturation,WBBluU,WBRedV,Hue,Aperture,AutoExpos,AutoFocus,Backlight,ExpProgr,Mode,Distance,Score");
				}catch (IOException e) {
				    System.err.println(e);
				}			
			}	

	    	// Assuming we're approaching the target, reduce brightness 
	    	altWCBrightness[0] = INITIAL_WC_BRIGHTNESS;	// Nominally 24
	    	altWCBrightness[1] = 18;
	    	altWCBrightness[2] = 12;
	    	altWCBrightness[3] = 6;
	    	altWCBrightness[4] = 0;
	    	altWCBrightness[5] = 36;
	    	altWCBrightness[6] = 30;
	    	altWCBrightness[7] = 24;
	    	
	    	altWCContrast[0] = INITIAL_WC_CONTRAST;		// Nominally 39
	    	altWCContrast[1] = 20;		
	    	altWCContrast[2] = 0;	
	    	altWCContrast[3] = INITIAL_WC_CONTRAST;		
	    	altWCContrast[4] = 20;	
	    	altWCContrast[5] = 0;		
	    	altWCContrast[6] = INITIAL_WC_CONTRAST;	
	    	altWCContrast[7] = 30;		
	    	
	    	double camSetting = 0;
	    	//5 - CV_CAP_PROP_FPS Frame rate.
	    	camSetting = camera.get(Videoio.CAP_PROP_APERTURE);		//-1
	    	camSetting = camera.get(Videoio.CAP_PROP_AUTOFOCUS);	//-1
	    	camSetting = camera.get(Videoio.CAP_PROP_FPS);			//0
	    	camSetting = camera.get(Videoio.CAP_PROP_FRAME_HEIGHT);	//480
	    	camSetting = camera.get(Videoio.CAP_PROP_FRAME_WIDTH);	//640
	    	camSetting = camera.get(Videoio.CAP_PROP_CONVERT_RGB);	//-1
	    	
	    	
	    	camSetting = camera.get(Videoio.CAP_PROP_BRIGHTNESS);	
	    	if (INIT_WEBCAM_SETTINGS) camera.set(Videoio.CAP_PROP_BRIGHTNESS, INITIAL_WC_BRIGHTNESS);	// 18, 66, 142, 32, 73 // 66  //10 - CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
	    	
	    	camSetting = camera.get(Videoio.CAP_PROP_CONTRAST);	
	    	if (INIT_WEBCAM_SETTINGS) camera.set(Videoio.CAP_PROP_CONTRAST, INITIAL_WC_CONTRAST);		// 90, 128, 32, 66, 128, 42, 32  //11 - CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
	    	
	    	camSetting = camera.get(Videoio.CAP_PROP_EXPOSURE);	
	    	if (INIT_WEBCAM_SETTINGS) camera.set(Videoio.CAP_PROP_EXPOSURE, INITIAL_WC_EXPOSURE); 	// -3, -1, -2 -3 -2, -1  // 15 - CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
	    	
	    	camSetting = camera.get(Videoio.CAP_PROP_GAIN);	
	    	if (INIT_WEBCAM_SETTINGS) camera.set(Videoio.CAP_PROP_GAIN, INITIAL_WC_GAIN);			// 16, 8, 44, 9, 40, 9, 44  // 14 - CV_CAP_PROP_GAIN Gain of the image (only for cameras).
	    	
	    	camSetting = camera.get(Videoio.CAP_PROP_SATURATION);	
	    	if (INIT_WEBCAM_SETTINGS) camera.set(Videoio.CAP_PROP_SATURATION, INITIAL_WC_SATURATION);			// 255, 209, 186, 35, 255 // 12 - CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
	    	
	    	camSetting = camera.get(Videoio.CAP_PROP_WHITE_BALANCE_BLUE_U);	
	    	if (INIT_WEBCAM_SETTINGS) camera.set(Videoio.CAP_PROP_WHITE_BALANCE_BLUE_U, INITIAL_WC_WHTBALBLU);		//6500, 2800, 6500
	    	
	    	camSetting = camera.get(Videoio.CAP_PROP_WHITE_BALANCE_RED_V);	
	    	if (INIT_WEBCAM_SETTINGS) camera.set(Videoio.CAP_PROP_WHITE_BALANCE_RED_V, INITIAL_WC_WHTBALRED);		//-1, 6500, -1, 6500
	    	
	    	camSetting = camera.get(Videoio.CAP_PROP_HUE);	
	    	if (INIT_WEBCAM_SETTINGS) camera.set(Videoio.CAP_PROP_HUE, INITIAL_WC_HUE);	// 3.6305E7, 4.04987E7, 4.3513e7, 3.84e7, //13 - CV_CAP_PROP_HUE Hue of the image (only for cameras).
	    	
		} else {
			analyzeCamera = false;
		}
		
		//Network Table Setup
		/*
		 * NetworkTable.setClientMode();
		if (WRITE_NETW_TBLS) {
			NetworkTable.setIPAddress("10.26.19.2");
		} else {
			NetworkTable.setIPAddress("127.0.0.1");
		}
		NetworkTable table2Rbt = NetworkTable.getTable("Vision");
		*/
		
		if (CALIBRATION_MODE) {
			calibrPass = 0;
		} else {
			calibrPass = MAX_CALIBR_PASS;	// Typically we use 99 to execute one time;
		}
		
		/*
		// Open up a GUI for user interactivity
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                createAndShowGUI();
            }
        });  
        */
        
		System.out.println("Starting");
		do {
			if (!USE_VIDEO) {
				
				// While normally not executed, here's where we choose alternate hue / saturation / luminance values for analysis
				selectNextParameterSet();
				
				// Load a test image from file for development
				image = Imgcodecs.imread(jpgFile);
				if (TROUBLESHOOTING_MODE) srcImage = image;

				/*
				// Experiment with reading a wmv file rather than a jpg
				VideoCapture cap = null;
				cap = new VideoCapture("GymCartApprch.wmv");
				cap.open("GymCartApprch.wmv");
				cap.read(image);
				*/
				
			} else if (revertToJPG) {
				// Save off the image that brought on this condition
				if (TROUBLESHOOTING_MODE && (webcamSettings == 0)) {
					Imgcodecs.imwrite("Problem_Image.jpg", srcImage);
				}
				
				selectNextParameterSet();
				
				// Choose an alternate brightness setting assuming that we may have the wrong setup
				if (INIT_WEBCAM_SETTINGS) camera.set(Videoio.CAP_PROP_BRIGHTNESS, altWCBrightness[webcamSettings]);
		    	
		    	// At least temporarily we will need to process a jpg to free up memory
				image = Imgcodecs.imread("DummyImage.jpg");
				if (TROUBLESHOOTING_MODE) srcImage = image;
				revertToJPG = false;
			} else {
				selectNextParameterSet();
				System.out.println("Reading next image");
				camera.read(image);
				if (TROUBLESHOOTING_MODE) srcImage = image;
			}
			
			// Having selected a source, process the image (this is the dominant call)
			processSingleImage(image);
			
			/*
			if (WRITE_NETW_TBLS) {
				// Having concluded analysis, update the Network Tables
				table2Rbt.putNumber("Distance", dist2Target/12);
				table2Rbt.putNumber("RobotAngle", angleTrajectory);		// This now depicts the recommended angle from current course.
				table2Rbt.putNumber("TargetAngle", angOfIncT);
				table2Rbt.putNumber("Quality", imageQuality);
				table2Rbt.putNumber("ImageCount", executionCount);
			}
			*/
			
			// Moving to continuous mode (or even calibration, some variables will need to be reset
			dist2TargetTemp = dist2Target;
			initializeForNextImage();
			executionCount ++;
			//System.currentTimeMillis()
			
		} while ((calibrPass < MAX_CALIBR_PASS) || (USE_VIDEO));
		//} while (executionCount < 1);
		//} while (executionCount < 300);
		//} while (imageQuality > 0.5);
		//} while ((dist2Target == 0) || (Double.toString(dist2Target).equals("NaN")));
		//} while (dist2TargetTemp < 15);
		//} while ((dist2TargetTemp == 0) || (dist2TargetTemp.isNaN()));
		
		if (USE_VIDEO)
			camera.release();
	}

	private static void createAndShowGUI() {
        //Create and set up the window.
		// Establish a frame where we'll allow the operator to tune and view
        JFrame ocvframe = new JFrame("visionFRC2619");
        ocvframe.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
 
        //Add the ubiquitous "Hello World" label.
        JLabel ocvlabel = new JLabel("ocv2619");
        ocvframe.getContentPane().add(ocvlabel);
 
        //Display the window.
        ocvframe.pack();
        ocvframe.setVisible(true);
        
    }
	
	private static void processSingleImage(Mat image) throws IOException, Exception {

		if (executionCount == 0) Imgcodecs.imwrite("OriginalImage.jpg", image);

		// Using the class Pipeline, instantiate here with the specified image
		// (replicate the class here)
		GripPipelineGym gp = new GripPipelineGym();
		gp.process(image, loHue, hiHue, loSat, hiSat, loLum, hiLum);
		//hiPixelValue = gp.getMatInfoHighValue();

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

			
			// Create a color "matrix" from the B&W matrix for use during analysis
			Mat temp = gp.hslThresholdOutput();
			hslTO = new Mat();
			Imgproc.cvtColor(temp, hslTO, Imgproc.COLOR_GRAY2BGR);
			
			for (int x = 0; x < ocvLineCount; x++){
				Imgproc.line(hslTO, targetLines[x].point1(), targetLines[x].point2(), new Scalar(10,10,250), 1);
			}
			// Save a copy of the amended file with the identified lines
			if (JPGS_TO_C) Imgcodecs.imwrite("HSLout_with_lines.jpg", hslTO);


			
			
			// Sort the line fragments by the average x1/x2
			sortTargetlinesX(targetLines);
			
			// Save our line data to srtLineOutput.txt
			exportLineData(targetLines);
			
			// Overlay the original image with the identified lines to Image_with_lines.jpg
			outputOverlayedLines(image, targetLines);
			
			// Characterize the differential spacing between vertical line groups
			calcXAvgDiff(xAvgDiff, targetLines);
			
			// Determine that differential between x values at which we would consider the lines to be the same line
			calcIsSameLine();
	
			double[] totalizedVerticalLen = new double[ocvLineCount]; // An array of totalized line lengths, though we expect fewer entries than allocated
			double[] nominalVerticalLineX = new double[ocvLineCount]; // Nominal x coordinate of the particular vertical line
	
			// Assemble line segments into groups with an effective length, determining the number of line groups = vLineSet + 1
			groupVerticalLines(xAvgDiff, targetLines, totalizedVerticalLen, nominalVerticalLineX);
			
			if (vLineSet < 3) {
				// Capture the image for later analysis
				if (vLineSet > 0) {
					if (poorImageCount == 0) {
						Imgcodecs.imwrite("Image_for_PostAnalysis.jpg", image);
					}
					poorImageCount++;

					// Estimate the distance to the target based on the available information
					
				}
				
				// Reduce the certainty of the analysis significantly
				recordAnalysisResults();
				
			} else {
				// Find the longest contiguous chain of lines for each vertical line group identified
				double[] yMinVerticalLine = new double[ocvLineCount]; // Minimum y coordinate of the particular vertical line
				double[] yMaxVerticalLine = new double[ocvLineCount]; // Maximum y coordinate of the particular vertical line
				double[] yMinVerticalLineSet = new double[ocvLineCount]; // Minimum y coordinate of the particular vertical line set
				double[] yMaxVerticalLineSet = new double[ocvLineCount]; // Maximum y coordinate of the particular vertical line set
				
				// Evaluating the various vertical line segments, look for contiguous sets that might constitute part of the target
				findLongestContiguousVLines(targetLines, nominalVerticalLineX, yMinVerticalLine, yMaxVerticalLine, yMinVerticalLineSet, yMaxVerticalLineSet);
				
				// While we hope for four vertical lines, choose the best four if there are more
				findFourBestVLines(totalizedVerticalLen, nominalVerticalLineX);
				
				// Establish upper and lower limits on our horizontal line Y values just in case
				minYGoodLine = pxlHeight;
				if (minYGoodLine > yMinVerticalLineSet[tgt1LeftXPtr]) {
					minYGoodLine = yMinVerticalLineSet[tgt1LeftXPtr];
				}
				if (minYGoodLine > yMinVerticalLineSet[tgt1RightXPtr]) {
					minYGoodLine = yMinVerticalLineSet[tgt1RightXPtr];
				}
				if (minYGoodLine > yMinVerticalLineSet[tgt2LeftXPtr]) {
					minYGoodLine = yMinVerticalLineSet[tgt2LeftXPtr];
				}
				if (minYGoodLine > yMinVerticalLineSet[tgt2RightXPtr]) {
					minYGoodLine = yMinVerticalLineSet[tgt2RightXPtr];
				}
				
				if (maxYGoodLine < yMaxVerticalLineSet[tgt1LeftXPtr]) {
					maxYGoodLine = yMaxVerticalLineSet[tgt1LeftXPtr];
				}
				if (maxYGoodLine < yMaxVerticalLineSet[tgt1RightXPtr]) {
					maxYGoodLine = yMaxVerticalLineSet[tgt1RightXPtr];
				}
				if (maxYGoodLine < yMaxVerticalLineSet[tgt2LeftXPtr]) {
					maxYGoodLine = yMaxVerticalLineSet[tgt2LeftXPtr];
				}
				if (maxYGoodLine < yMaxVerticalLineSet[tgt2RightXPtr]) {
					maxYGoodLine = yMaxVerticalLineSet[tgt2RightXPtr];
				}
				
				// Get the corresponding extreme (x,y) points from the vertical lines
				xAtYfind[12] = nominalVerticalLineX[tgt1LeftXPtr];
				yAtYfind[12] = yMaxVerticalLineSet[tgt1LeftXPtr];
				
				xAtYfind[13] = nominalVerticalLineX[tgt1RightXPtr];
				yAtYfind[13] = yMaxVerticalLineSet[tgt1RightXPtr];
				
				xAtYfind[14] = nominalVerticalLineX[tgt2LeftXPtr];
				yAtYfind[14] = yMaxVerticalLineSet[tgt2LeftXPtr];
				
				xAtYfind[15] = nominalVerticalLineX[tgt2RightXPtr];
				yAtYfind[15] = yMaxVerticalLineSet[tgt2RightXPtr];
				
				xAtYfind[28] = nominalVerticalLineX[tgt1LeftXPtr];
				yAtYfind[28] = yMinVerticalLineSet[tgt1LeftXPtr];
				
				xAtYfind[29] = nominalVerticalLineX[tgt1RightXPtr];
				yAtYfind[29] = yMinVerticalLineSet[tgt1RightXPtr];
				
				xAtYfind[30] = nominalVerticalLineX[tgt2LeftXPtr];
				yAtYfind[30] = yMinVerticalLineSet[tgt2LeftXPtr];
				
				xAtYfind[31] = nominalVerticalLineX[tgt2RightXPtr];
				yAtYfind[31] = yMinVerticalLineSet[tgt2RightXPtr];
				
				if (bestWt > 0) {
					// Find the horizontal lines of the targets
					double[] xMinHorizontalLine = new double[ocvLineCount]; // Minimum y coordinate of the particular vertical line
					double[] xMaxHorizontalLine = new double[ocvLineCount]; // Maximum y coordinate of the particular vertical line
					double[] xMinHorizontalLineSet = new double[ocvLineCount]; // Minimum y coordinate of the particular vertical line set
					double[] xMaxHorizontalLineSet = new double[ocvLineCount]; // Maximum y coordinate of the particular vertical line set
					
					// Having chosen our four best vertical lines, determine where we'll look for our horizontal boundaries of the targets
					findHorizontalLines(edgeID, targetLines, nominalVerticalLineX, yMinVerticalLineSet, yMaxVerticalLineSet, xMinHorizontalLine, xMaxHorizontalLine, xMinHorizontalLineSet, xMaxHorizontalLineSet);
					
					// While actually derived some time ago, write to file the HSL output
					generateGripImage(gp);
					
					// Using x,y pairs from predefined positions within the line segments, perform a line fit for target top and bottom 
					findLineFit(image, edgeID, targetLines);
					
					// Generate the files ocvLineOutput.txt and RsltLineOutput.txt
					saveLineData(xAvgDiff, edgeID, targetLines, totalizedVerticalLen, nominalVerticalLineX, yMinVerticalLineSet, yMaxVerticalLineSet, xMinHorizontalLineSet, xMaxHorizontalLineSet);
					
					// Generate the file XYLineFit.txt
					printLineFit();
					
					// Calculate the angle of incidence for the Robot as well as the estimated distance from the robot
					calcAngOfIncR();
					
					// Calculate the angle of incidence of the target and recommend a new direction for the robot to take
					calcTargetDistance();
				}
				
				// For the case where we want to analyze a series of tuning choices, record the results to a file (Image Quality)
				recordAnalysisResults();
				
				// There may be cases where we want to analyze an image taken from video
				if (USE_VIDEO && (imageQuality < 0.2)) {
					if (poorImageCount == 0) {
						Imgcodecs.imwrite("Image_for_PostAnalysis.jpg", image);
					}
					poorImageCount++;
				}
			}
		} else {
			dist2Target = 0;
			angOfIncT = 0;
			angOfIncR = 0;
			angleTrajectory = 0;
			imageQuality = 0;
			//System.out.println("Less than 3 lines were found in the image :" + Double.toString(ocvLineCount));
			System.out.println("Less than 3 lines were found in the image: " + ocvLineCount);
			if (jpgMemMgmt) revertToJPG = true;
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
			estTgtW = estTgtW / pxlH2Vratio;
		} else {
			
			// We are closer to the right edge of the target
			halfFoViewV = 0.5 * INCH_TGT_HEIGHT * pxlHeight / (yAtXFitTR - yAtXFitBR);	// 0.5 * 5 * (1080 / 378)
			dist2Target = halfFoViewV / tanHlfAngleV;
			System.out.println("The estimated distance to the target (in inches) is " + Double.toString(dist2Target));
			estTgtW = (TARGET_WIDTH / INCH_TGT_HEIGHT) * (yAtXFitTR - yAtXFitBR);
			estTgtW = estTgtW / pxlH2Vratio;
		}
		
		//
		halfFoViewH = 0.5 * TARGET_WIDTH * pxlWidth / (nomXTgt2R - nomXTgt1L);
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
		percHLFPts = 0;
		minYGoodLine = 0;
		maxYGoodLine = 0;
		//hiPixelValue = 0;
	}
	
	private static void printLineFit() throws IOException {
		PrintWriter outputStream = null;
		try {
			
			String LineOut;
			outputStream = new PrintWriter(new FileWriter("XYLineFit.txt"));

			outputStream.println("xValue,yValue");
			for (int i = 0; i <= 31; i++) {
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
		
		if (USE_VIDEO) {
			// We may want to alter camera settings if we aren't getting data
			if (revertToJPG) {
				if (webcamSettings == 7) {
					webcamSettings = 0;
				} else {
					webcamSettings ++;
				}
			}
		} else {
			//jpgFile = new String("Picture 6.jpg");
			//jpgFile = new String("LTGym3ft.jpg");	
			//jpgFile = new String("Kitchen58inLt.jpg");
			//jpgFile = new String("KitchLtOn24in.jpg");
			//jpgFile = new String("ConfRoom12in.jpg");
			//jpgFile = new String("ConfRoom104in.jpg");
			//jpgFile = new String("KitchLtOn20in60d.jpg");
			//jpgFile = new String("KtchGrTgtTape50in0d.jpg");
			//jpgFile = new String("XYCalibration23in0d.jpg");
			//jpgFile = new String("KitchFaceSun20in0d.jpg");
			//jpgFile = new String("KitchenTemp.jpg");
			//jpgFile = new String("KitchLtOn46in45d.jpg");
			//jpgFile = new String("OriginalVImage.jpg");
			//jpgFile = new String("LTGym6f45d.jpg");
			//jpgFile = new String("LTGym6f70d.jpg");
			//jpgFile = new String("LTGym8ft.jpg");
			//jpgFile = new String("LTGym18ft.jpg"); 
			//jpgFile = new String("BreakRoom0221.jpg");
			jpgFile = new String("TestImage.jpg");
			//jpgFile = new String("Image_for_PostAnalysis.jpg");
		}

		// By default, we clear this variable and only set it if applicable
		lastTestInCalbrPh = false;
		
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
					loHue += 1;
					if (loHue >= (hiHue - 1)) lastTestInCalbrPh = true;
					
				} else if (calibrPhase == 1) {
					
					// See if expanding the low Hue results in results that are better or worse than our best
					loHue -= 1;
					if (loHue <=  32) lastTestInCalbrPh = true;
					
				} else if (calibrPhase == 2) {
					
					// See if expanding the high Hue results in results that are better or worse than our best
					hiHue += 1;
					if (hiHue >= 255) lastTestInCalbrPh = true;
					
				} else if (calibrPhase == 3) {
					
					// See if expanding the low Hue results in results that are better or worse than our best
					hiHue -= 1;
					if (hiHue <=  (loHue + 1)) lastTestInCalbrPh = true;
					
				} else if (calibrPhase == 4) {
					
					// See if expanding the high Hue results in results that are better or worse than our best
					loSat += 1;
					if (loSat >= (hiSat - 1)) lastTestInCalbrPh = true;
					
				} else if (calibrPhase == 5) {
					
					// See if expanding the low Hue results in results that are better or worse than our best
					loSat -= 1;
					if (loSat <=  1) lastTestInCalbrPh = true;
					
				} else if (calibrPhase == 6) {
					
					// See if expanding the high Hue results in results that are better or worse than our best
					hiSat += 1;
					if (hiSat >= 255) lastTestInCalbrPh = true;
					
				} else if (calibrPhase == 7) {
					
					// See if expanding the low Hue results in results that are better or worse than our best
					hiSat -= 1;
					if (hiSat <=  (loSat + 1)) lastTestInCalbrPh = true;
					
				} else if (calibrPhase == 8) {
					
					// See if expanding the high Hue results in results that are better or worse than our best
					loLum += 1;
					if (loLum >= (hiLum - 1)) lastTestInCalbrPh = true;
					
				} else if (calibrPhase == 9) {
					
					// See if expanding the low Hue results in results that are better or worse than our best
					loLum -= 1;
					if (loLum <=  1) lastTestInCalbrPh = true;
					
				} else if (calibrPhase == 10) {
					
					// See if expanding the high Hue results in results that are better or worse than our best
					hiLum += 1;
					if (hiLum >= 255) lastTestInCalbrPh = true;
					
				} else if (calibrPhase == 11) {
					
					// See if expanding the low Hue results in results that are better or worse than our best
					hiLum -= 1;
					if (hiLum <=  (loLum + 1)) lastTestInCalbrPh = true;
					
				} else {
					calibrPhase = (MAX_CALIBR_PASS + 1);	// Signaling the end if calibration.  Save the result
				}
			}
		} else {
			// Since we're not in calibration mode, simply set our values as specified initial sets
			loHue = INITIAL_LO_HUE;
			hiHue = INITIAL_HI_HUE;
			loSat = INITIAL_LO_SATURATION;
			hiSat = INITIAL_HI_SATURATION;
			loLum = INITIAL_LO_LUMIN;
			hiLum = INITIAL_HI_LUMIN;	
			lastTestInCalbrPh = true;
		}
		
		
		// This class needs to interact closely with recordAnalysisResults to assess whether the last test was better or worse
		
	}
	private static void recordAnalysisResults() throws IOException {
		// Here we record the quality of the analysis from the most recent image analysis
		double stopQuality = 0;
		
		imageQuality = 0;
		
		if (ocvLineCount > 8) {
			if (ocvLineCount < 50) {
				imageQuality += 0.4;
			} else {
				imageQuality += 0.2;
			}
		}
		
		if (vLineSet == 3) {
			imageQuality += 0.5;
		} else if (vLineSet == 4) {
			imageQuality += 0.45;
		} else if (vLineSet == 5) {
			imageQuality += 0.4;
		} else if (vLineSet == 6) {
			imageQuality += 0.35;
		} else if (vLineSet == 7) {
			imageQuality += 0.3;
		}
		
		imageQuality -= (stdErrT / 5);
		imageQuality -= (stdErrB / 7);
		
		imageQuality -= 0.2 * (lTgtAccrW - 1) * (lTgtAccrW - 1);
		imageQuality -= 0.2 * (rTgtAccrW - 1) * (rTgtAccrW - 1);
		
		if (dist2Target > 1) {
			imageQuality += .1;
		} else {
			imageQuality = 0.1 * imageQuality;
		}
		
		// Reduce the rating according to the number of points missing from the linefit data
		imageQuality = imageQuality * percHLFPts;
		
		if(imageQuality < 0.5) {
			if (TROUBLESHOOTING_MODE) Imgcodecs.imwrite("Problem_Image.jpg", srcImage);
			stopQuality = imageQuality;
		}
		
		// Finally, ensure that we return a rating between 0 and 1
		if (imageQuality < 0) {
			imageQuality = 0;
		}
		
		if (imageQuality > 1) {
			imageQuality = 1;
		}
		
		//if (TROUBLESHOOTING_MODE) System.out.println("The analysis confidence value was " + Double.toString(imageQuality));
		System.out.println("The analysis confidence value was " + Double.toString(imageQuality));
		System.out.println(" ");

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
				    out.println("FileImage,loHue,hiHue,loSat,hiSat,loLum,hiLum,LineCount,VLnGrps,LnFitStErrB,LnFitStErrT,LTAccur,RTAccur,Dist,AngleT,AngleR,%LFPts,Score");
				}catch (IOException e) {
				    System.err.println(e);
				}			
				
				
			} else if (calibrPhase >= 99) {

				// Signal the end
				calibrPass = MAX_CALIBR_PASS;
				
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
					LineOut += "," + Double.toString(percHLFPts);
					LineOut += "," + Double.toString(calibrScore);
					out.println(LineOut);
				}catch (IOException e) {
				    System.err.println(e);
				}			
				
			} else {

				// Being in the middle of calibration, evaluate the most recent analysis
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
					LineOut += "," + Double.toString(stdErrT);
					LineOut += "," + Double.toString(lTgtAccrW);
					LineOut += "," + Double.toString(rTgtAccrW);
					LineOut += "," + Double.toString(dist2Target);
					LineOut += "," + Double.toString(angOfIncT);
					LineOut += "," + Double.toString(angOfIncR);
					LineOut += "," + Double.toString(percHLFPts);
					LineOut += "," + Double.toString(calibrScore);
					out.println(LineOut);
				}catch (IOException e) {
				    System.err.println(e);
				}			

			
			}
			
			// As a last step in the calibration pass, determine when we go to the next calibration phase
			if (imageQuality < (0.5 * optimumQuality)) {
				// Since we reset to the optimum settings at each phase, don't bother letting this fall off too far
				calibrPhase ++;
				//optimumQuality = 0;
				
				// Restore the optimum values
				loHue = optLoHue;
				hiHue = optHiHue;
				loSat = optLoSat;
				hiSat = optHiSat;
				loLum = optLoLum;
				hiLum = optHiLum;
					
			} else if (lastTestInCalbrPh) {
				// If we've just finished testing the last value for the current phase, move to the next
				calibrPhase++;
				
				// Restore the optimum values
				loHue = optLoHue;
				hiHue = optHiHue;
				loSat = optLoSat;
				hiSat = optHiSat;
				loLum = optLoLum;
				hiLum = optHiLum;
			}
			
			if (calibrPass < 100) calibrPass ++;
			
		} else if ((analyzeCamera) && (USE_VIDEO)) {
			// Collect a running tally of camera settings vs results
			
			// Choose an efficient means to repetitively append an analysis file (append here)
			try(PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter("WebCamVRslt.csv", true)))) {
				String LineOut;
				LineOut = Double.toString(camera.get(Videoio.CAP_PROP_FPS));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_BRIGHTNESS));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_CONTRAST));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_EXPOSURE));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_GAIN));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_SATURATION));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_WHITE_BALANCE_BLUE_U));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_WHITE_BALANCE_RED_V));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_HUE));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_APERTURE));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_AUTO_EXPOSURE));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_AUTOFOCUS));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_BACKLIGHT));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_EXPOSUREPROGRAM));
				LineOut += "," + Double.toString(camera.get(Videoio.CAP_PROP_MODE));
				LineOut += "," + Double.toString(dist2Target);
				LineOut += "," + Double.toString(imageQuality);
				out.println(LineOut);
			}catch (IOException e) {
			    System.err.println(e);
			}

			
		}
	}
	
	private static void findLineFit(Mat image, String[] edgeID, TargetLine[] targetLines) {
		// Get the accompanying horizontal lines in order to validate the height of the rectangles
		double mSlope = 0;		// slope of the line we use to fit the top of the target
		double yIntcpt = 0;		// y intercept for the line we use to fit to
		Random rand = new Random();
		int foundCount = 0;		// Watch how many data points we find
		if (TROUBLESHOOTING_MODE) System.out.println(" ");
		
		// Clear out the previous image's y1 values from the line analysis
		for (int i = 0; i < 32; i++) {
			y1AtYfind[i] = 0;
		}
		
		// Get linefit coordinates for the "top" horizontal lines of target 1
		foundCount = 0;
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "1HT") {
				//if (TROUBLESHOOTING_MODE) System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
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
							foundCount++;
							//if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							//if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		if (foundCount > 0) {
			// Having found horizontal line data for 1HT, ignore the associated vertical line data which isn't likely to help
			yAtYfind[12] = 0;
			yAtYfind[13] = 0;
		}
		
		// Now get linefit coordinates for the top horizontal lines of target 2
		foundCount = 0;
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "2HT") {
				//if (TROUBLESHOOTING_MODE) System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
				mSlope = (targetLines[zLpCtr1].ocvY2 - targetLines[zLpCtr1].ocvY1) / (targetLines[zLpCtr1].ocvX2 - targetLines[zLpCtr1].ocvX1);
				yIntcpt = targetLines[zLpCtr1].ocvY1 - mSlope * targetLines[zLpCtr1].ocvX1;
				//if (TROUBLESHOOTING_MODE) System.out.println("Slope and Intercept as " + Double.toString(mSlope) + " / " + Double.toString(yIntcpt));
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
							foundCount++;
							//if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							//if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		if (foundCount > 0) {
			// Having found horizontal line data for 2HT, ignore the associated vertical line data which isn't likely to help
			yAtYfind[14] = 0;
			yAtYfind[15] = 0;
		}
		
		// Note that (x,y) pairs 12 - 15 were grabbed from the vertical line max values
		
		if (TROUBLESHOOTING_MODE) {
			System.out.println(" ");
		}
		
		// Now make the calculations necessary to perform the slope / intercept calculations of the horizontal best fit
		double sumx = 0;
		double sumx2 = 0;
		double sumy = 0;
		int ptCount = 0;
		for (int zLpCtr1 = 0; zLpCtr1 < 16; zLpCtr1++) {
			if (yAtYfind[zLpCtr1] > 0) {
				ptCount ++;
				sumx += xAtYfind[zLpCtr1];
				sumx2 += xAtYfind[zLpCtr1] * xAtYfind[zLpCtr1];
				sumy += yAtYfind[zLpCtr1];
			}
		}
		
		// While this isn't yet a percent, we start by totaling the points found
		percHLFPts = ptCount;
		
		// Avoid a 0 / 0 situation 
		if (ptCount == 0) ptCount = 1;
		
		double xbar = sumx / ptCount;
		double ybar = sumy / ptCount;
		
        // second pass: compute summary statistics
        double xxbar = 0.0, yybar = 0.0, xybar = 0.0;
        ptCount = 0;
        for (int zLpCtr1 = 0; zLpCtr1 < 16; zLpCtr1++) {
			if (yAtYfind[zLpCtr1] > 0) {
	            ptCount++;
	            xxbar += (xAtYfind[zLpCtr1]) * (xAtYfind[zLpCtr1]);
	            yybar += (yAtYfind[zLpCtr1]) * (yAtYfind[zLpCtr1]);
	            xybar += (xAtYfind[zLpCtr1]) * (yAtYfind[zLpCtr1]);
			}
        }
        
        // Finally, calculate the slope and y intercept of the best fit top line
        if (ptCount > 1) {
	        topSlope = (ptCount * xybar) - (sumx * sumy);
	        topSlope = topSlope / ((ptCount * xxbar) - (sumx * sumx));
	        topIntercept = (ybar - topSlope * xbar);
        } else {
        	topSlope = 0;
        	topIntercept = maxYGoodLine;
        }        	
        
        // At least while we want quality estimates, calculate error data from the line-fit
        if (ptCount > 1) {
        	for (int i = 0; i < 16; i++) {
	        	if (yAtYfind[i] > 0) {
	        		yFit = topSlope * xAtYfind[i] + topIntercept;
	        		y1AtYfind[i] += (yFit - yAtYfind[i]) * (yFit - yAtYfind[i]);
	        		stdErrT = y1AtYfind[i] / ptCount;	// This will only be "correct" at the last point
	        	}
	        }
	        stdErrT = Math.sqrt(stdErrT);
        } else {
        	stdErrT = 50;
        }
        
		if (TROUBLESHOOTING_MODE) System.out.println("Determined slope / intercept of " + Double.toString(topSlope) + " / " + Double.toString(topIntercept));
        yAtXFitTL = topSlope * nomXTgt1L + topIntercept;
        yAtXFitTR = topSlope * nomXTgt2R + topIntercept;
		//if (TROUBLESHOOTING_MODE) System.out.println("Top Left x / y of " + Double.toString(nomXTgt1L) + " / " + Double.toString(yAtXFitTL));
		//if (TROUBLESHOOTING_MODE) System.out.println("Top Right x / y of " + Double.toString(nomXTgt2R) + " / " + Double.toString(yAtXFitTR));
		
		// Get linefit coordinates for the "bottom" horizontal lines of target 1		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
		foundCount = 0;
        for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "1HB") {
				//if (TROUBLESHOOTING_MODE) System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
				mSlope = (targetLines[zLpCtr1].ocvY2 - targetLines[zLpCtr1].ocvY1) / (targetLines[zLpCtr1].ocvX2 - targetLines[zLpCtr1].ocvX1);
				yIntcpt = targetLines[zLpCtr1].ocvY1 - mSlope * targetLines[zLpCtr1].ocvX1;
				for (int zLpCtr2 = 16; zLpCtr2 < 22; zLpCtr2++) {
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
							foundCount++;
							//if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							//if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		if (foundCount > 0) {
			// Having found horizontal line data for 1HB, ignore the associated vertical line data which isn't likely to help
			yAtYfind[28] = 0;
			yAtYfind[29] = 0;
		}
		
		// Now get linefit coordinates for the "bottom" horizontal lines of target 2
		foundCount = 0;
		for (int zLpCtr1 = 0; zLpCtr1 < ocvLineCount; zLpCtr1++) {
			if (edgeID[zLpCtr1] == "2HB") {
				//if (TROUBLESHOOTING_MODE) System.out.println("Find linefit coordinates for line " + Integer.toString(zLpCtr1));
				mSlope = (targetLines[zLpCtr1].ocvY2 - targetLines[zLpCtr1].ocvY1) / (targetLines[zLpCtr1].ocvX2 - targetLines[zLpCtr1].ocvX1);
				yIntcpt = targetLines[zLpCtr1].ocvY1 - mSlope * targetLines[zLpCtr1].ocvX1;
				//if (TROUBLESHOOTING_MODE) System.out.println("Slope and Intercept as " + Double.toString(mSlope) + " / " + Double.toString(yIntcpt));
				for (int zLpCtr2 = 22; zLpCtr2 < 28; zLpCtr2++) {
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
							foundCount++;
							//if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						} else if (rand.nextInt(100) > 50) {
							yAtYfind[zLpCtr2] = mSlope * xAtYfind[zLpCtr2] + yIntcpt;
							//if (TROUBLESHOOTING_MODE) System.out.println("Fitting for x / y of " + Double.toString(xAtYfind[zLpCtr2]) + " / " + Double.toString(yAtYfind[zLpCtr2]));
						}
					}
				}
			} 
		}
		if (foundCount > 0) {
			// Having found horizontal line data for 2HB, ignore the associated vertical line data which isn't likely to help
			yAtYfind[30] = 0;
			yAtYfind[31] = 0;
		}
		
		if (TROUBLESHOOTING_MODE) System.out.println(" ");
		
		// Re-initialize the variables used to calculate slope / intercept
		sumx = 0;
		sumx2 = 0;
		sumy = 0;
		ptCount = 0;
		for (int zLpCtr1 = 16; zLpCtr1 < 32; zLpCtr1++) {
			if (yAtYfind[zLpCtr1] > 0) {
				ptCount ++;
				sumx += xAtYfind[zLpCtr1];
				sumx2 += xAtYfind[zLpCtr1] * xAtYfind[zLpCtr1];
				sumy += yAtYfind[zLpCtr1];
			}
		}
		
		percHLFPts += ptCount;
		percHLFPts = percHLFPts / 32;		// We're looking for a total of 32 points to linefit top and bottom
		
		// Again, avoid a 0 / 0 situation
		if (ptCount == 0) ptCount = 1;
		
		xbar = sumx / ptCount;
		ybar = sumy / ptCount;
		
        xxbar = 0.0;
        yybar = 0.0;
        xybar = 0.0;
        ptCount = 0;
        for (int zLpCtr1 = 16; zLpCtr1 < 32; zLpCtr1++) {
			if (yAtYfind[zLpCtr1] > 0) {
	            ptCount++;
	            xxbar += (xAtYfind[zLpCtr1]) * (xAtYfind[zLpCtr1]);
	            yybar += (yAtYfind[zLpCtr1]) * (yAtYfind[zLpCtr1]);
	            xybar += (xAtYfind[zLpCtr1]) * (yAtYfind[zLpCtr1]);
			}
        }
        
        if (ptCount > 1) {
	        // Finally, calculate the slope and y intercept of the best fit "bottom" line
	        bottomSlope = (ptCount * xybar) - (sumx * sumy);					// Numerator
	        bottomSlope = bottomSlope / ((ptCount * xxbar) - (sumx * sumx));	// Numerator / Denominator
	        bottomIntercept = (ybar - bottomSlope * xbar);
	
	        // At least while we want quality estimates, calculate error data from the line-fit
	        for (int i = 16; i < 32; i++) {
	        	if (yAtYfind[i] > 0) {
	        		yFit = bottomSlope * xAtYfind[i] + bottomIntercept;
	        		y1AtYfind[i] += (yFit - yAtYfind[i]) * (yFit - yAtYfind[i]);
	        		stdErrB = y1AtYfind[i] / ptCount;	// This will only be "correct" at the last point
	        	}
	        }
	        stdErrB += Math.sqrt(stdErrB);
	        
        } else {
        	bottomSlope = 0;
        	bottomIntercept = minYGoodLine;
        	stdErrB += 50;
        }
        
		if (TROUBLESHOOTING_MODE) System.out.println("Determined slope / intercept of " + Double.toString(bottomSlope) + " / " + Double.toString(bottomIntercept));
        yAtXFitBL = bottomSlope * nomXTgt1L + bottomIntercept;
        yAtXFitBR = bottomSlope * nomXTgt2R + bottomIntercept;
        Point ptTL = new Point(nomXTgt1L, yAtXFitTL);
        Point ptTR = new Point(nomXTgt2R, yAtXFitTR);
        Point ptBL = new Point(nomXTgt1L, yAtXFitBL);
        Point ptBR = new Point(nomXTgt2R, yAtXFitBR);
		//if (TROUBLESHOOTING_MODE) System.out.println("Top Left x / y of " + Double.toString(nomXTgt1L) + " / " + Double.toString(yAtXFitTL));
		//if (TROUBLESHOOTING_MODE) System.out.println("Top Right x / y of " + Double.toString(nomXTgt2R) + " / " + Double.toString(yAtXFitTR));
		//if (TROUBLESHOOTING_MODE) System.out.println("Bottom Left x / y of " + Double.toString(nomXTgt1L) + " / " + Double.toString(yAtXFitBL));
		//if (TROUBLESHOOTING_MODE) System.out.println("Bottom Right x / y of " + Double.toString(nomXTgt2R) + " / " + Double.toString(yAtXFitBR));
		if (TROUBLESHOOTING_MODE) System.out.println(" ");

		// Update our picture with the new determined lines / rectangle
		Imgproc.line(image, ptTL, ptTR, new Scalar(0,0,255), 1);
		Imgproc.line(image, ptBL, ptBR, new Scalar(0,0,255), 1);
		Imgproc.line(image, ptTL, ptBL, new Scalar(0,0,255), 1);
		Imgproc.line(image, ptBR, ptTR, new Scalar(0,0,255), 1);

		// Save a copy of the amended file with the identified lines
		if (JPGS_TO_C) Imgcodecs.imwrite("Image_with_Answer.jpg", image);
	}

	private static void calcTargetDistance() {
		// Note:  This will have to be corrected as it currently assumes a 90 degree angle of incidence
		obsTgtW = (nomXTgt2R - nomXTgt1L);
		if (obsTgtW > estTgtW ) {
			// Not a realistic solution so impose an upper limit
			obsTgtW = estTgtW;
		}
		
		// Calculate the magnitude of the angle, converting to degrees
		angOfIncT = Math.acos(obsTgtW / estTgtW) * RAD_TO_DEG;
		if ((yAtXFitTR - yAtXFitBR) > (yAtXFitTL - yAtXFitBL)) {
			// Because the right target is larger than the left, the angle must be negative
			angOfIncT = -angOfIncT;
		}
		
		// If we aren't aimed straight on, the calculation should be adjusted
		angOfIncT -= angOfIncR;
		
		// While a bit complicated, calculate an angle of trajectory for the robot
		double isectDist = 0;	// Intended intersection for straight line trajectory
		double effSinAngle = 0;
		double angleOffTgt = 0;
		effSinAngle = Math.sin(angOfIncT / RAD_TO_DEG);
		isectDist = (effSinAngle * ROBOT_TURN_RADIUS + DISTANCE_ON_COURSE) * effSinAngle;
		angleOffTgt = Math.atan(isectDist / dist2Target) * RAD_TO_DEG;
		angleTrajectory = angleOffTgt + angOfIncR;
		if (angleTrajectory > (HALF_FIELD_ANGLE_H/2)) angleTrajectory = (HALF_FIELD_ANGLE_H/2);
		System.out.println("The recommended change of course in degrees is " + Double.toString(angleTrajectory));

		
		
		System.out.println("The estimated angle of incidence (in degrees) for the target is " + Double.toString(angOfIncT));
		System.out.println("The estimated angle of incidence (in degrees) for the robot is " + Double.toString(angOfIncR));
		
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
		if (JPGS_TO_C) Imgcodecs.imwrite("HSLOutputFile.jpg", gp.hslThresholdOutput());
	}

	private static void findHorizontalLines(String[] edgeID, TargetLine[] targetLines, double[] nominalVerticalLineX,
		double[] yMinVerticalLineSet, double[] yMaxVerticalLineSet, double[] xMinHorizontalLine,
		double[] xMaxHorizontalLine, double[] xMinHorizontalLineSet, double[] xMaxHorizontalLineSet) {
		double nomYTgtTop = 0;
		double nomYTgtBtm = 0;
		double testX1 = 0;
		double testX2 = 0;
		double okHLGap = 12;
		double meanTop = 0;
		double meanBtm = 0;
		double stdevTop = 0;
		double stdevBtm = 0;
		int useCount = 0;

		// Initialize the arrays for the x max and min for the horizontal lines
		for (int x = 0; x <= vLineSet; x ++) {
			xMinHorizontalLineSet[x] = pxlWidth;
			xMaxHorizontalLineSet[x] = 0;
		}
		
 		// Note the nominal target top and bottom values (calculate a simple average from the vertical lines)
		for (int x = 0; x <= vLineSet; x ++) {
			if (x == tgt1LeftXPtr) {
				meanBtm += yMinVerticalLineSet[x];
				meanTop += yMaxVerticalLineSet[x];
			} else if (x == tgt1RightXPtr) {
				meanBtm += yMinVerticalLineSet[x];
				meanTop += yMaxVerticalLineSet[x];
			} else if (x == tgt2LeftXPtr) {
				meanBtm += yMinVerticalLineSet[x];
				meanTop += yMaxVerticalLineSet[x];
			} else if (x == tgt2RightXPtr) {
				meanBtm += yMinVerticalLineSet[x];
				meanTop += yMaxVerticalLineSet[x];
			}
		}
		meanBtm = meanBtm/4;
		meanTop = meanTop/4;
		
		// Calculate the standard deviation, top and bottom
		for (int x = 0; x <= vLineSet; x ++) {
			if (x == tgt1LeftXPtr) {
				stdevBtm += (yMinVerticalLineSet[x] - meanBtm) * (yMinVerticalLineSet[x] - meanBtm);
				stdevTop += (yMaxVerticalLineSet[x] - meanTop) * (yMaxVerticalLineSet[x] - meanTop);
			} else if (x == tgt1RightXPtr) {
				stdevBtm += (yMinVerticalLineSet[x] - meanBtm) * (yMinVerticalLineSet[x] - meanBtm);
				stdevTop += (yMaxVerticalLineSet[x] - meanTop) * (yMaxVerticalLineSet[x] - meanTop);
			} else if (x == tgt2LeftXPtr) {
				stdevBtm += (yMinVerticalLineSet[x] - meanBtm) * (yMinVerticalLineSet[x] - meanBtm);
				stdevTop += (yMaxVerticalLineSet[x] - meanTop) * (yMaxVerticalLineSet[x] - meanTop);
			} else if (x == tgt2RightXPtr) {
				stdevBtm += (yMinVerticalLineSet[x] - meanBtm) * (yMinVerticalLineSet[x] - meanBtm);
				stdevTop += (yMaxVerticalLineSet[x] - meanTop) * (yMaxVerticalLineSet[x] - meanTop);
			}
		}
		stdevBtm = Math.sqrt(stdevBtm/4);
		stdevTop = Math.sqrt(stdevTop/4);

		// Throwing out outliers, estimate the Y value for the top of the target
		useCount = 0;
		for (int x = 0; x <= vLineSet; x ++) {
			if (x == tgt1LeftXPtr) {
				if (((yMaxVerticalLineSet[x] - meanTop) * (yMaxVerticalLineSet[x] - meanTop)) < (stdevTop * stdevTop)) {
					nomYTgtTop += yMaxVerticalLineSet[x];
					useCount ++;
				}
			} else if (x == tgt1RightXPtr) {
				if (((yMaxVerticalLineSet[x] - meanTop) * (yMaxVerticalLineSet[x] - meanTop)) < (stdevTop * stdevTop)) {
					nomYTgtTop += yMaxVerticalLineSet[x];
					useCount ++;
				}
			} else if (x == tgt2LeftXPtr) {
				if (((yMaxVerticalLineSet[x] - meanTop) * (yMaxVerticalLineSet[x] - meanTop)) < (stdevTop * stdevTop)) {
					nomYTgtTop += yMaxVerticalLineSet[x];
					useCount ++;
				}
			} else if (x == tgt2RightXPtr) {
				if (((yMaxVerticalLineSet[x] - meanTop) * (yMaxVerticalLineSet[x] - meanTop)) < (stdevTop * stdevTop)) {
					nomYTgtTop += yMaxVerticalLineSet[x];
					useCount ++;
				}
			}
		}
		if (useCount > 0) {
			nomYTgtTop = nomYTgtTop / useCount;
		} else {
			nomYTgtTop = maxYGoodLine;
		}
		
		// Throwing out outliers, estimate the Y value for the bottom of the target
		useCount = 0;
		for (int x = 0; x <= vLineSet; x ++) {
			if (x == tgt1LeftXPtr) {
				if (((yMinVerticalLineSet[x] - meanBtm) * (yMinVerticalLineSet[x] - meanBtm)) < (stdevBtm * stdevBtm)) {
					nomYTgtBtm += yMinVerticalLineSet[x];
					useCount ++;
				}
			} else if (x == tgt1RightXPtr) {
				if (((yMinVerticalLineSet[x] - meanBtm) * (yMinVerticalLineSet[x] - meanBtm)) < (stdevBtm * stdevBtm)) {
					nomYTgtBtm += yMinVerticalLineSet[x];
					useCount ++;
				}
			} else if (x == tgt2LeftXPtr) {
				if (((yMinVerticalLineSet[x] - meanBtm) * (yMinVerticalLineSet[x] - meanBtm)) < (stdevBtm * stdevBtm)) {
					nomYTgtBtm += yMinVerticalLineSet[x];
					useCount ++;
				}
			} else if (x == tgt2RightXPtr) {
				if (((yMinVerticalLineSet[x] - meanBtm) * (yMinVerticalLineSet[x] - meanBtm)) < (stdevBtm * stdevBtm)) {
					nomYTgtBtm += yMinVerticalLineSet[x];
					useCount ++;
				}
			}
		}
		if (useCount > 0) {
			nomYTgtBtm = nomYTgtBtm / useCount;
		} else {
			nomYTgtBtm = minYGoodLine;
		}
		
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

		// Capture the estimated target properties
		for (int x = 0; x <= vLineSet; x++) {
			// Vertical ...
			if (x == tgt1LeftXPtr) {
				Imgproc.line(hslTO, new Point(nomXTgt1L,0), new Point(nomXTgt1L,pxlHeight), new Scalar(200,250,50), 1);	
				Imgproc.line(hslTO, new Point(nomXTgt1L,0), new Point(nomXTgt1L,80), new Scalar(200,250,50), 3);	
				Imgproc.line(hslTO, new Point(nomXTgt1L,pxlHeight-80), new Point(nomXTgt1L,pxlHeight), new Scalar(200,250,50), 3);	
			} else if (x == tgt1RightXPtr) {
				Imgproc.line(hslTO, new Point(nomXTgt1R,0), new Point(nomXTgt1R,pxlHeight), new Scalar(200,250,50), 1);	
				Imgproc.line(hslTO, new Point(nomXTgt1R,0), new Point(nomXTgt1R,80), new Scalar(200,250,50), 3);	
				Imgproc.line(hslTO, new Point(nomXTgt1R,pxlHeight-80), new Point(nomXTgt1R,pxlHeight), new Scalar(200,250,50), 3);	
			} else if (x == tgt2LeftXPtr) {
				Imgproc.line(hslTO, new Point(nomXTgt2L,0), new Point(nomXTgt2L,pxlHeight), new Scalar(200,250,50), 1);	
				Imgproc.line(hslTO, new Point(nomXTgt2L,0), new Point(nomXTgt2L,80), new Scalar(200,250,50), 3);	
				Imgproc.line(hslTO, new Point(nomXTgt2L,pxlHeight-80), new Point(nomXTgt2L,pxlHeight), new Scalar(200,250,50), 3);	
			} else if (x == tgt2RightXPtr) {
				Imgproc.line(hslTO, new Point(nomXTgt2R,0), new Point(nomXTgt2R,pxlHeight), new Scalar(200,250,50), 1);	
				Imgproc.line(hslTO, new Point(nomXTgt2R,0), new Point(nomXTgt2R,80), new Scalar(200,250,50), 3);	
				Imgproc.line(hslTO, new Point(nomXTgt2R,pxlHeight-80), new Point(nomXTgt2R,pxlHeight), new Scalar(200,250,50), 3);	
			}
		}
		// ... and horizontal
		Imgproc.line(hslTO, new Point(0,nomYTgtTop), new Point(pxlWidth,nomYTgtTop), new Scalar(200,250,50), 1);	
		Imgproc.line(hslTO, new Point(0,nomYTgtTop), new Point(50,nomYTgtTop), new Scalar(200,250,50), 3);	
		Imgproc.line(hslTO, new Point(pxlWidth-50,nomYTgtTop), new Point(pxlWidth,nomYTgtTop), new Scalar(200,250,50), 3);	

		Imgproc.line(hslTO, new Point(0,nomYTgtBtm), new Point(pxlWidth,nomYTgtBtm), new Scalar(200,250,50), 1);	
		Imgproc.line(hslTO, new Point(0,nomYTgtBtm), new Point(50,nomYTgtBtm), new Scalar(200,250,50), 3);	
		Imgproc.line(hslTO, new Point(pxlWidth-50,nomYTgtBtm), new Point(pxlWidth,nomYTgtBtm), new Scalar(200,250,50), 3);	
		
		if (JPGS_TO_C) Imgcodecs.imwrite("HSLout_with_lines_chosen.jpg", hslTO);

		

		// At this point we can capture 6 preferred points associated with each target's horizontal line fits
		incrX = (nomXTgt1R - nomXTgt1L) / 7;
		xAtYfind[0] = nomXTgt1L + incrX;
		yAtYfind[0] = 0;
		xAtYfind[16] = nomXTgt1L + incrX;
		yAtYfind[16] = 0;
		//if (TROUBLESHOOTING_MODE) System.out.println("Find Y at X = " + xAtYfind[0]);
		for (int zLpCtr1 = 1; zLpCtr1 < 6; zLpCtr1++) {
			xAtYfind[zLpCtr1] = xAtYfind[zLpCtr1 - 1] + incrX;
			yAtYfind[zLpCtr1] = 0;
			xAtYfind[zLpCtr1 + 16] = xAtYfind[zLpCtr1 - 1] + incrX;
			yAtYfind[zLpCtr1 + 16] = 0;
			//if (TROUBLESHOOTING_MODE) System.out.println("Find Y at X = " + xAtYfind[zLpCtr1]);
		}
		
		incrX = (nomXTgt2R - nomXTgt2L) / 7;
		xAtYfind[6] = nomXTgt2L + incrX;
		yAtYfind[6] = 0;
		xAtYfind[22] = nomXTgt2L + incrX;
		yAtYfind[22] = 0;
		//if (TROUBLESHOOTING_MODE) System.out.println("Find Y at X = " + xAtYfind[6]);
		for (int zLpCtr1 = 7; zLpCtr1 < 12; zLpCtr1++) {
			xAtYfind[zLpCtr1] = xAtYfind[zLpCtr1 - 1] + incrX;
			yAtYfind[zLpCtr1] = 0;
			xAtYfind[zLpCtr1 + 16] = xAtYfind[zLpCtr1 - 1] + incrX;
			yAtYfind[zLpCtr1 + 16] = 0;
			//if (TROUBLESHOOTING_MODE) System.out.println("Find Y at X = " + xAtYfind[zLpCtr1]);
		}
		if (TROUBLESHOOTING_MODE) System.out.println(" ");
		
		for (int x = 0; x < ocvLineCount; x++) {
			// No processing required for lines that aren't horizontal
			if (targetLines[x].isHorizontal()) {
				// For each of the grouped vertical line pairs, look for the target tops and bottoms
				
				//if (TROUBLESHOOTING_MODE) System.out.println("Evaluating horizontal line " + Integer.toString(x));

				// Having found an(other) line within the line group, get the min and max and see if we can validate the target's horizontal lines
				if (targetLines[x].ocvX1 < targetLines[x].ocvX2) {
					testX1 = targetLines[x].ocvX1;
					testX2 = targetLines[x].ocvX2;
				} else {	
					testX1 = targetLines[x].ocvX2;
					testX2 = targetLines[x].ocvX1;
				}	
				
				// Because the line is roughly horizontal, it doesn't matter much whether we choose Y1 or Y2
				if ((targetLines[x].ocvY1 < (nomYTgtTop + 2 * okHLGap)) && (targetLines[x].ocvY1 > (nomYTgtTop - okHLGap))) {
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
					
				} else if ((targetLines[x].ocvY1 < (nomYTgtBtm + okHLGap)) && (targetLines[x].ocvY1 > (nomYTgtBtm - 2 * okHLGap))) {
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
			
			// While the choice is arbitrary, choose a best weight
			bestWt = (nominalVerticalLineX[tgt2LeftXPtr] - nominalVerticalLineX[tgt1RightXPtr]);
			
			// Now verify that we have acceptable spacing to presume these to be our targets
			rectRatio = (nominalVerticalLineX[tgt2LeftXPtr] - nominalVerticalLineX[tgt1RightXPtr]);
			rectRatio = rectRatio / (nominalVerticalLineX[tgt1RightXPtr] - nominalVerticalLineX[tgt1LeftXPtr]);
			lTgtAccrW = rectRatio / refRatio;
			if (TROUBLESHOOTING_MODE) System.out.println("The left obs:act gap to strip target accuracy is 1 : " + Double.toString(lTgtAccrW));
			
			rectRatio = (nominalVerticalLineX[tgt2LeftXPtr] - nominalVerticalLineX[tgt1RightXPtr]);
			rectRatio = rectRatio / (nominalVerticalLineX[tgt2RightXPtr] - nominalVerticalLineX[tgt2LeftXPtr]);
			rTgtAccrW = rectRatio / refRatio;
			if (TROUBLESHOOTING_MODE) System.out.println("The right obs:act gap to strip target accuracy is 1 : " + Double.toString(rTgtAccrW));
			
		} else if (vLineSet < 3) {
			// We wouldn't expect to arrive here, but if we do we need to gracefully indicate that processing didn't go well
		} else {
			// Find the three sets of signals that yield the best 6.25 / 2 spacing ratio
			bestWt = 0;
			for (int x = 0; x <= (vLineSet-3); x++) {
				for (int y = 1; y <= (vLineSet-2); y++) {
					for (int z = 2; z <= (vLineSet-1); z++) {
						for (int w = 3; w <= vLineSet; w++) {
							if ((x < y) && (y < z) && (z < w)){
								//if (TROUBLESHOOTING_MODE) System.out.println("Assessing line set : " + Integer.toString(x) + ":" + Integer.toString(y) + ":" + Integer.toString(z) + ":" + Integer.toString(w));
								spacedOK = false;
								lineWt = 0;
								gap1 = (nominalVerticalLineX[y] - nominalVerticalLineX[x]);
								gap2 = (nominalVerticalLineX[z] - nominalVerticalLineX[y]);
								gap3 = (nominalVerticalLineX[w] - nominalVerticalLineX[z]);
								//if (TROUBLESHOOTING_MODE) System.out.println("Assessing gap of : " + Double.toString(gap1));
								//if (TROUBLESHOOTING_MODE) System.out.println("Assessing gap of : " + Double.toString(gap2));
								//if (TROUBLESHOOTING_MODE) System.out.println("Assessing gap of : " + Double.toString(gap3));
								if (((gap3 / gap1) < 1.5) && ((gap1 / gap3) < 1.5)) {
									if (((gap2 / gap1) > 3) && ((gap2 / gap3) > 3)) {
										if (((gap2 / gap1) < 10) && ((gap2 / gap3) < 10)) {
											spacedOK = true;
											lineWt = totalizedVerticalLen[w] + totalizedVerticalLen[x] + totalizedVerticalLen[y] + totalizedVerticalLen[z];
											if (lineWt >= bestWt) {
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
				System.out.println("No definitive target found.");
				//throw new Exception("vLineSet is perhaps greater than 4 and handling logic is required.");
			}
			
			if (TROUBLESHOOTING_MODE) System.out.println("Selecting verticals : " + Integer.toString(vertSel[0]) + ":" + Integer.toString(vertSel[1]) + ":" + Integer.toString(vertSel[2]) + ":" + Integer.toString(vertSel[3]));
			tgt1LeftXPtr = vertSel[0];
			tgt1RightXPtr = vertSel[1];
			tgt2LeftXPtr = vertSel[2];
			tgt2RightXPtr = vertSel[3];

			// Now verify that we have acceptable spacing to presume these to be our targets
			rectRatio = (nominalVerticalLineX[tgt2LeftXPtr] - nominalVerticalLineX[tgt1RightXPtr]);
			rectRatio = rectRatio / (nominalVerticalLineX[tgt1RightXPtr] - nominalVerticalLineX[tgt1LeftXPtr]);
			lTgtAccrW = rectRatio / refRatio;
			if (TROUBLESHOOTING_MODE) System.out.println("The left target accuracy is 1 : " + Double.toString(lTgtAccrW));
			
			rectRatio = (nominalVerticalLineX[tgt2LeftXPtr] - nominalVerticalLineX[tgt1RightXPtr]);
			rectRatio = rectRatio / (nominalVerticalLineX[tgt2RightXPtr] - nominalVerticalLineX[tgt2LeftXPtr]);
			rTgtAccrW = rectRatio / refRatio;
			if (TROUBLESHOOTING_MODE) System.out.println("The right target accuracy is 1 : " + Double.toString(rTgtAccrW));
			
		}
		if (TROUBLESHOOTING_MODE) System.out.println();
		
		
		if (vLineSet > 0) {
			// Save our weighted lines to a file if analysis is needed (false to create)
			try(PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter("weighted_line_segments.txt", false)))) {
			    out.println("NomX,WtdVLen,LineID,");
			}catch (IOException e) {
			    System.err.println(e);
			}			
			
			// (true to append)
			try(PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter("weighted_line_segments.txt", true)))) {
				for (int x = 0; x <= (vLineSet); x++) {
					String LineOut;
					LineOut = Double.toString(nominalVerticalLineX[x]);
					LineOut += "," + Double.toString(totalizedVerticalLen[x]);
					
					if (x == tgt1LeftXPtr) {
						LineOut += ",T1L";
					} else if (x == tgt1RightXPtr) {
						LineOut += ",T1R";
					} else if (x == tgt2LeftXPtr) {
						LineOut += ",T2L";
					} else if (x == tgt2RightXPtr) {
						LineOut += ",T2R";
					}
					
					out.println(LineOut);
				}
			}catch (IOException e) {
			    System.err.println(e);
			}			
		}
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
			// Update an image to include a vertical next line top to bottom at this nominal x
			Imgproc.line(hslTO, new Point(nominalVerticalLineX[x],0), new Point(nominalVerticalLineX[x],pxlHeight), new Scalar(250,0,255), 1);
			if (JPGS_TO_C) Imgcodecs.imwrite("HSLout_with_candidate_lines.jpg", hslTO);
			//if (TROUBLESHOOTING_MODE) System.out.println("Evaluating vertical line group " + Integer.toString(x));
			diffVLCount = 0;
			// For each original line relating to this line group, append as able
			for (int y = 0; y < ocvLineCount; y++) {
				//if (y == 114) {       // This is only for debugging to allow a logical break point
				//	testY1 = testY2;
				//	testY2 = testY1;
				//}
				if (targetLines[y].isVertical()) {
					if ((targetLines[y].xAvg >= (nominalVerticalLineX[x] - isSameLine)) && (targetLines[y].xAvg <= (nominalVerticalLineX[x] + isSameLine))) {
						// At least for now, confirm that we're evaluating all of the appropriate vertical lines
						//if (TROUBLESHOOTING_MODE) System.out.println("Evaluating for grouping vertical line " + Integer.toString(y));

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
							//if (TROUBLESHOOTING_MODE) System.out.println("Initializing resulting vertical line " + Integer.toString(x));
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
									// Situations 1, 2 or 3 from the documentation
									if (testY1 < (yMaxVerticalLine[z] + okVLGap)) {
										// Situations 2 or 3 from the documentation
										// Append the two lines
										yMaxVerticalLine[z] = testY2;
										
										if (testY1 < yMinVerticalLine[z]) {
											// Situation 3 from the documentation
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
										for (int w = 0; w < diffVLCount; w++) {
											if (w != z) {
												// Possibly append this new grouped segment with other segments 
												// before or after in the array, retaining lower array position
												if (yMaxVerticalLine[w] > yMaxVerticalLine[z]) {
													if ((yMinVerticalLine[w] > (yMinVerticalLine[z])) && (yMinVerticalLine[w] < (yMaxVerticalLine[z] + okVLGap))){
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
															yMaxVerticalLine[z] = yMaxVerticalLine[w];
															// Update at zLpCtr3 and eliminate at zLpCtr4
															if (yMinVerticalLine[w] < yMinVerticalLine[z]) {
																yMinVerticalLine[z] = yMinVerticalLine[w];
															}
															// Eliminate the obsolete line group
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
												}
											}
										}
										
										// Exit the loop having appended the segments
										z = diffVLCount + 1;
									}
								} else {
									// So (testY2 <= ymaxVlineEvl[z]), representing situations 4, 5 or 6 from the documentation
									// Similar logic but reversing the evaluation
									if ((testY2 > (yMinVerticalLine[z] - okVLGap)) && (testY1 < yMinVerticalLine[z])) {
										// Append the two lines (Situation 5)
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
										
									} else if (testY1 > (yMinVerticalLine[z] - okVLGap)) {
										// Because the line was ambiguous, simply ignore it here
										wasAppd = true;
									}
								}
							}
							
							if (!wasAppd) {
								// Record them separately for now (Situations 1 and 6 from the documentation)
								yMinVerticalLine[diffVLCount] = testY1;
								yMaxVerticalLine[diffVLCount] = testY2;
								
								// It's possible that this single segment is longer than the previous "group"
								// Assess whether we now have a new longest combined line segment at this x value
								if ((yMaxVerticalLine[diffVLCount] - yMinVerticalLine[diffVLCount]) > (yMaxVerticalLineSet[x] - yMinVerticalLineSet[x])) {
									yMinVerticalLineSet[x] = yMinVerticalLine[diffVLCount];
									yMaxVerticalLineSet[x] = yMaxVerticalLine[diffVLCount];
								}
								
								diffVLCount ++;
								
							}
						}
					}
				}
			}
			
			if (TROUBLESHOOTING_MODE) {
				System.out.println("Vertical line Group " + Double.toString(x) + ":");									
				for (int lgc = 0; lgc <= diffVLCount; lgc++) {
					System.out.println("Vertical line GroupX low and high " + Double.toString(yMinVerticalLine[lgc]) + " / " + yMaxVerticalLine[lgc]);									
				}
				System.out.println(" ");
			}
			
			// At least during development, display horizontal lines that we've just identified
			Imgproc.line(hslTO, new Point(0,yMinVerticalLineSet[x]), new Point(pxlWidth,yMinVerticalLineSet[x]), new Scalar(250,x,255), 1);
			Imgproc.line(hslTO, new Point(0,yMaxVerticalLineSet[x]), new Point(pxlWidth,yMaxVerticalLineSet[x]), new Scalar(250,x,255), 1);
			if (JPGS_TO_C) Imgcodecs.imwrite("HSLout_with_candidate_lines.jpg", hslTO);
			if (yMaxVerticalLineSet[x] > 250) {
				userStop = true;
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

			// Allow a troubleshooting break point
			//if (x == 11) {
			//	userStop = true;
			//}
			
			// Note that we do nothing for non-vertical lines
			if (xAvgDiff[x] > isSameLine) {
				// The line is vertical and assessed to be the first in the next group of vertical lines
				// Capture the cumulative assessment of the line length for the previous group of lines
				totalizedVerticalLen[vLineSet] = cumulLen;

				// Reset the cumulative determination to the length of the next line
				cumulLen = targetLines[x].length;
				
				// Capture the nominal x coordinate
				nominalVerticalLineX[vLineSet] = (lastAdjVerticalline + firstAdjVerticalline) / 2;
				
				if (TROUBLESHOOTING_MODE) System.out.println("This line of length " + Double.toString(totalizedVerticalLen[vLineSet]));
				if (TROUBLESHOOTING_MODE) System.out.println("Its position is roughly " + Double.toString(nominalVerticalLineX[vLineSet]));
				
				// Having closed out the last group of lines, if this is both the first and the last we wrap up
				if (x == (ocvLineCount - 1)) {
					// Capture the cumulative assessment of the line length
					vLineSet++;
					
					totalizedVerticalLen[vLineSet] = targetLines[x].length;

					// Capture the nominal x coordinate
					nominalVerticalLineX[vLineSet] = targetLines[x].xAvg;
					
					if (TROUBLESHOOTING_MODE) System.out.println("This line of length " + Double.toString(totalizedVerticalLen[vLineSet]));
					if (TROUBLESHOOTING_MODE) System.out.println("Its position is roughly " + Double.toString(nominalVerticalLineX[vLineSet]));
					
					
				} else {
					// Increment the count of grouped lines
					vLineSet += 1;
					
					// Capture the x coordinate for the first line in the new group
					firstAdjVerticalline = targetLines[x].xAvg;
					lastAdjVerticalline = targetLines[x].xAvg;
					
				}
				
			} else if (targetLines[x].isVertical())	{	
				// The line is vertical but is close enough in proximity to suggest it's the same line
				cumulLen += targetLines[x].length;
				//if (TROUBLESHOOTING_MODE) System.out.println("Expanded to " + Double.toString(cumulLen));
				lastAdjVerticalline = targetLines[x].xAvg;
				
				// If this happens to be the very first vertical line of the set, initialize that value as well
				if (firstAdjVerticalline == 0) {
					firstAdjVerticalline = lastAdjVerticalline;
				}
				
				// Consider whether any additional handling is required if this happens to be the last line
				if (x == (ocvLineCount - 1)) {
					totalizedVerticalLen[vLineSet] = cumulLen;
					
					// Capture the nominal x coordinate
					nominalVerticalLineX[vLineSet] = (lastAdjVerticalline + firstAdjVerticalline) / 2;
					
					if (TROUBLESHOOTING_MODE) System.out.println("The last line is of length " + Double.toString(totalizedVerticalLen[vLineSet]));
					if (TROUBLESHOOTING_MODE) System.out.println("Its position is roughly " + Double.toString(nominalVerticalLineX[vLineSet]));
					
				}
			} else if (x == (ocvLineCount - 1)) {
				// This line doesn't contribute, but we need to record the results of the last line group
				totalizedVerticalLen[vLineSet] = cumulLen;
				
				// Capture the nominal x coordinate
				nominalVerticalLineX[vLineSet] = (lastAdjVerticalline + firstAdjVerticalline) / 2;
				
				if (TROUBLESHOOTING_MODE) System.out.println("The last line is of length " + Double.toString(totalizedVerticalLen[vLineSet]));
				if (TROUBLESHOOTING_MODE) System.out.println("Its position is roughly " + Double.toString(nominalVerticalLineX[vLineSet]));
				
			}
			// Note:  In this loop, nothing happens for lines that aren't vertical
		}

		if (TROUBLESHOOTING_MODE) System.out.println("The number of line sets is " + Integer.toString(vLineSet + 1));
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
		if (JPGS_TO_C) Imgcodecs.imwrite("Image_with_lines.jpg", image);
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



