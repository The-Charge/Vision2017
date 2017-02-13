package com.thecharge;

public class CalibrationData {
	public String fileName;			// Analysis filename reference
	public double loHue;
	public double hiHue;
	public double loSat;
	public double hiSat;
	public double loLum;
	public double hiLum;
	public double linesFound;		// Number of lines found by GRIP
	public Integer vertLineGrps;	// Count of the vertical line groups
	public Integer horzLineGrps;	// Count of the horizontal line groups
	public double lfStdErr;			// Line fit std error
	public double ltTgtAccr;		// Left Tgt Accuracy
	public double rtTgtAccr;		// Right Tgt Accuracy
	
	public CalibrationData(String f, double lh, double hh, double ls, double hs){
		fileName = f;
		loHue = lh;
		hiHue = hh;
		loSat = ls;
		hiSat = hs;
	}
}
