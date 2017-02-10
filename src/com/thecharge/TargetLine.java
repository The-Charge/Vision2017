package com.thecharge;
import org.opencv.core.Point;

import com.thecharge.GripPipelineGym.Line;

public class TargetLine implements Comparable {
	
	public static double lenTotal=0;
	public double xAvg;
	public double ocvX1, ocvX2, ocvY1, ocvY2, angle, length;
	double maxAngleErr = 15;	//8;    // How far from vertical or horizontal will be included?

	public TargetLine(Line line){
		this.ocvX1 = line.x1;
		this.ocvX2 = line.x2;
		this.ocvY1 = line.y1;
		this.ocvY2 = line.y2;
		this.angle = line.angle();
		this.length = line.length();
		TargetLine.lenTotal += this.length;
		
		this.xAvg = (line.x1 + line.x2)/2;
	}
	public double lenPerc(){
		return this.length / TargetLine.lenTotal;
	}
	public boolean isVertical(){
		
		if ((this.angle > (90 - maxAngleErr)) && (this.angle < (90 + maxAngleErr))) {
			return true;
		} else if ((this.angle > (-90 - maxAngleErr)) && (this.angle < (-90 + maxAngleErr))) {
			return true;
		} else {
			return false;
		}

	}
	
	public Point point1(){
		return new Point(this.ocvX1, this.ocvY1);
	}

	public Point point2(){
		return new Point(this.ocvX2, this.ocvY2);
	}
	
	public boolean isHorizontal(){
		//double maxAngleErr = 8;
		if ((this.angle > (0 - maxAngleErr)) && (this.angle < (0 + maxAngleErr))) {
			return true;
		} else if ((this.angle > (180 - maxAngleErr)) && (this.angle < (180 + maxAngleErr))) {
			return true;
		} else if ((this.angle > (-180 - maxAngleErr)) && (this.angle < (-180 + maxAngleErr))) {
			return true;
		} else {
			return false;
		}

	}
	@Override
	public int compareTo(Object o) {
		TargetLine tl = (TargetLine)o;
		return Double.compare(this.xAvg, tl.xAvg);
	}
	
	@Override
	public String toString(){
		String lineOut = "";
		lineOut += xAvg;
		lineOut += "," + this.angle;
		lineOut += "," + this.length;
		lineOut += "," + this.ocvX1;
		lineOut += "," + this.ocvX2;
		lineOut += "," + this.ocvY1;
		lineOut += "," + this.ocvY2;
		lineOut += "," + this.lenPerc();
		lineOut += "," + this.isVertical();
		lineOut += "," + this.isHorizontal();
		return lineOut;
	}
}