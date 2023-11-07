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

    double middleThreshold = 300; // tenative
    double leftThreshold = 300; // tenative
    double percentageError = 0.05;
    public OpenCVPropFinder () {
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        teamUtil.log("Initializing OpenCVPropFinder processor");

        teamUtil.log("Initialized OpenCVPropFinder processor");
    }
    public void outputTelemetry () {
        telemetry.addData("Saturation L/R/M: ", "%.1f/%.1f/%.1f",satRectLeft, satRectMiddle, satRectRight);
        telemetry.addLine("Prop Location: " + propPosition);
    }

    Mat HSVMat = new Mat();
    public Rect rectLeft = new Rect(90, 90, 110, 50); // TODO: These probably depend on alliance side, so move to init?
    public Rect rectMiddle = new Rect(400, 50, 110, 60);
    public Rect rectRight = new Rect(490, 90, 110, 50);

    List<MatOfPoint> contours = new ArrayList<>();
    double propPosition;
    double satRectLeft, satRectRight, satRectMiddle;

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        propPosition = 2;
        Imgproc.cvtColor(frame, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV
        if (teamUtil.alliance == teamUtil.Alliance.RED) {
            satRectRight = getAvgSaturation(HSVMat, rectRight);
            satRectMiddle = getAvgSaturation(HSVMat, rectMiddle);
            satRectLeft = 0;
            if (Math.abs(satRectMiddle/middleThreshold - satRectLeft/leftThreshold)>percentageError) {
                if(satRectMiddle/middleThreshold>satRectLeft/leftThreshold){
                    propPosition = 2;
                }else{
                    propPosition = 1;
                }
            } else {
                propPosition = 3;
            }
        } else {
            satRectLeft = getAvgSaturation(HSVMat, rectLeft);
            satRectMiddle = getAvgSaturation(HSVMat, rectMiddle);
            satRectRight = 0;
            propPosition = 2;
        }

        return null; // No need to pass data to OnDrawFrame

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
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