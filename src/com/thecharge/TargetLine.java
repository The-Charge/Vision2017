package com.thecharge;
import com.thecharge.GripPipelineGym.Line;

public class TargetLine implements Comparable {
	public static double lenTotal=0;
			public double xAvg;
	public double ocvX1, ocvX2, ocvY1, ocvY2, angle, length;
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
	public boolean isVerticle(){
		double maxAngleErr = 5;
		if ((this.angle > (90 - maxAngleErr)) && (this.angle < (90 + maxAngleErr))) {
			return true;
		} else if ((this.angle > (-90 - maxAngleErr)) && (this.angle < (-90 + maxAngleErr))) {
			return true;
		} else {
			return false;
		}

	}
	public boolean isHorizontal(){
		double maxAngleErr = 5;
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
}
