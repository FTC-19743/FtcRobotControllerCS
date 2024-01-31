package org.firstinspires.ftc.teamcode.assemblies;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.libs.OpenCVProcesser;
import org.firstinspires.ftc.teamcode.libs.runningVoteCount;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class OpenCVPropFinder extends OpenCVProcesser {
    HardwareMap hardwareMap;
    Telemetry telemetry;


    Mat HSVMat = new Mat();
    public Rect rectLeft, rectMiddle, rectRight;
    private runningVoteCount election= new runningVoteCount(3000);  // track readings for the last 3 seconds

    int propPosition;
    double satRectLeft, satRectRight, satRectMiddle;

    // These constants should all be tuned at the same time
    public int propExposure = 50 ; //  frame exposure in ms (use TestDrive opMode to calibrate)
    public int propGain = 1; // Unknown--  DOESN'T WORK DUE TO FTC BUG
    double leftRedThreshold = 70; //
    double middleRedThreshold = 80; //
    double rightRedThreshold = 50; //
    double middleBlueThreshold = 100; //112
    double leftBlueThreshold = 100; //
    double rightBlueThreshold = 90; //

    public OpenCVPropFinder () {
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        teamUtil.log("Initializing OpenCVPropFinder processor");

        if (teamUtil.alliance== teamUtil.Alliance.RED&&teamUtil.SIDE==teamUtil.Side.SCORE) {
            rectLeft = new Rect(0, 0, 1, 1);
            rectMiddle = new Rect(210, 75 , 100, 100); // was Rect(200, 100, 100, 100)
            rectRight = new Rect(519, 100, 120, 120); // was Rect(500, 125, 110, 100);
        } else if(teamUtil.alliance== teamUtil.Alliance.RED&&teamUtil.SIDE==teamUtil.Side.WING) {
            rectLeft = new Rect(10, 90, 120, 120);
            rectMiddle = new Rect(370, 80, 100, 100); // was rectMiddle = new Rect(200, 80, 100, 100);
            rectRight = new Rect(0, 0, 1, 1); // was new Rect(520, 90, 100, 100)
        } else if(teamUtil.alliance== teamUtil.Alliance.BLUE&&teamUtil.SIDE==teamUtil.Side.SCORE) {
            rectLeft = new Rect(0, 120, 110, 120); // was Rect(0, 130, 110, 120)
            rectMiddle = new Rect(350, 75, 100, 100); // was Rect(350, 100, 100, 100)
            rectRight = new Rect(0, 0, 1, 1);
        }  else  {//blue wing
            rectLeft = new Rect(0, 0, 1, 1); // Rect(0, 130, 110, 120)
            rectMiddle = new Rect(175 , 75, 100, 100); // was Rect(360, 120, 100, 100)
            rectRight = new Rect(519, 100, 120, 120);
        }
        teamUtil.log("Initialized OpenCVPropFinder processor");
    }
    public void outputTelemetry () {
        telemetry.addData("Saturation L/M/R: ", "%.1f/%.1f/%.1f",satRectLeft, satRectMiddle, satRectRight);
        telemetry.addLine("Prop Location: " + getPropPosition());
    }

    public int getPropPosition() {
        return election.getWinner(1);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        propPosition = 2;
        Imgproc.cvtColor(frame, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV
        if (teamUtil.alliance == teamUtil.Alliance.RED && teamUtil.SIDE==teamUtil.Side.WING) {
            satRectLeft = getAvgSaturation(HSVMat, rectLeft);
            satRectMiddle = getAvgSaturation(HSVMat, rectMiddle);
            satRectRight = 0;
            if(satRectLeft> leftRedThreshold){ propPosition = 1;
            } else if(satRectMiddle>middleRedThreshold){ propPosition = 2;
            } else { propPosition = 3;}
        } else if (teamUtil.alliance == teamUtil.Alliance.RED && teamUtil.SIDE==teamUtil.Side.SCORE) {
            satRectLeft = 0;
            satRectMiddle = getAvgSaturation(HSVMat, rectMiddle);
            satRectRight = getAvgSaturation(HSVMat, rectRight);;
            if(satRectRight> rightRedThreshold){ propPosition = 3;
            } else if(satRectMiddle>middleRedThreshold){ propPosition = 2;
            } else { propPosition = 1;}
        }
        else if (teamUtil.alliance == teamUtil.Alliance.BLUE && teamUtil.SIDE==teamUtil.Side.WING) {
            satRectLeft = 0;
            satRectMiddle = getAvgSaturation(HSVMat, rectMiddle);
            satRectRight = getAvgSaturation(HSVMat, rectRight);;
            if(satRectRight> rightBlueThreshold){ propPosition = 3;
            } else if(satRectMiddle>middleBlueThreshold){ propPosition = 2;
            } else { propPosition = 1;}
        } else { // BLUE, SCORE
            satRectLeft = getAvgSaturation(HSVMat, rectLeft);
            satRectMiddle = getAvgSaturation(HSVMat, rectMiddle);
            satRectRight = 0;

            if(satRectLeft> leftBlueThreshold) {propPosition = 1;}
            else if(satRectMiddle>middleBlueThreshold){propPosition = 2;}
            else{ propPosition = 3; }
        }
        election.vote(propPosition);

        return null; // No need to pass data to OnDrawFrame

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        // draw a grid on the view
        Paint linePaint = new Paint();
        linePaint.setColor(Color.WHITE);
        //linePaint.setStyle(Paint.Style.STROKE);
        //linePaint.setStrokeWidth(scaleCanvasDensity * 4);
        for (int i=100;i<onscreenHeight;i=i+100){ canvas.drawLine(0,i,onscreenWidth ,i,linePaint);  }
        for (int i=100;i<onscreenWidth;i=i+100){ canvas.drawLine(i,0,i,onscreenHeight,linePaint);  }

        //Add the rectangles that we are sampling within
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx), rectPaint);
        if (teamUtil.alliance == teamUtil.Alliance.RED && teamUtil.SIDE== teamUtil.Side.WING) {
            canvas.drawRect(makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx), rectPaint);
        } if (teamUtil.alliance == teamUtil.Alliance.RED && teamUtil.SIDE== teamUtil.Side.SCORE) {
            canvas.drawRect(makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx), rectPaint);
        } else if (teamUtil.alliance == teamUtil.Alliance.BLUE && teamUtil.SIDE== teamUtil.Side.WING) {
            canvas.drawRect(makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx), rectPaint);
        } else { // BLUE SCORE
            canvas.drawRect(makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx), rectPaint);
        }
    }
}