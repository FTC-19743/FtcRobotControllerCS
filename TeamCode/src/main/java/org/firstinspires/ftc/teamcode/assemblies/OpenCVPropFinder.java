package org.firstinspires.ftc.teamcode.assemblies;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.libs.OpenCVProcesser;
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

    int propPosition;
    double satRectLeft, satRectRight, satRectMiddle;
    double middleRedThreshold = 70; //
    double rightRedThreshold = 50; //
    double middleBlueThreshold = 100; //112
    double leftBlueThreshold = 100; //
    double percentageError = 0.05;
    public OpenCVPropFinder () {
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        teamUtil.log("Initializing OpenCVPropFinder processor");
        if (teamUtil.alliance== teamUtil.Alliance.RED) {
            rectLeft = new Rect(0, 0, 1, 1);
            rectMiddle = new Rect(200, 100, 100, 100);
            rectRight = new Rect(500, 125, 110, 100);
        } else { // TODO: Need to set up the correct Rectangles for the Blue Side
            rectLeft = new Rect(50, 260, 100, 100);
            rectMiddle = new Rect(330, 210, 100, 80);
            rectRight = new Rect(0, 0, 1, 1);
        }
        teamUtil.log("Initialized OpenCVPropFinder processor");
    }
    public void outputTelemetry () {
        telemetry.addData("Saturation L/M/R: ", "%.1f/%.1f/%.1f",satRectLeft, satRectMiddle, satRectRight);
        telemetry.addLine("Prop Location: " + propPosition);
    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        propPosition = 2;
        Imgproc.cvtColor(frame, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV
        if (teamUtil.alliance == teamUtil.Alliance.RED) {
            satRectRight = getAvgSaturation(HSVMat, rectRight);
            satRectMiddle = getAvgSaturation(HSVMat, rectMiddle);
            satRectLeft = 0;

            if(satRectRight> rightRedThreshold){
                propPosition = 3;
            }
            else if(satRectMiddle>middleRedThreshold){
                propPosition = 2;
            }
            else{
                propPosition = 1;
            }



        } else {
            satRectLeft = getAvgSaturation(HSVMat, rectLeft);
            satRectMiddle = getAvgSaturation(HSVMat, rectMiddle);
            satRectRight = 0;

            if(satRectLeft> leftBlueThreshold){
                propPosition = 3;
            }
            else if(satRectMiddle>middleBlueThreshold){
                propPosition = 2;
            }
            else{
                propPosition = 1;
            }
        }

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

        if (teamUtil.alliance == teamUtil.Alliance.RED) {
            canvas.drawRect(makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx), rectPaint);
            canvas.drawRect(makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx), rectPaint);
        } else {
            canvas.drawRect(makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx), rectPaint);
            canvas.drawRect(makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx), rectPaint);
        }
    }
}