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
        telemetry.addLine("Prop Location: " + propPosition);
    }

    Mat HSVMat = new Mat();
    public Rect rectLeft = new Rect(90, 90, 110, 50);
    public Rect rectMiddle = new Rect(400, 50, 110, 60);


    List<MatOfPoint> contours = new ArrayList<>();
    double propPosition;

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        propPosition = 2;
        Imgproc.cvtColor(frame, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV
        double satRectLeft = getAvgSaturation(HSVMat, rectLeft);
        double satRectMiddle = getAvgSaturation(HSVMat, rectMiddle);

        telemetry.addLine("SatLeft: "+ satRectLeft+ " SatMid:"+ satRectMiddle);
        // TODO: logic goes here

        return satRectLeft; // maybe we don't need to return anything...

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (userContext != null) {
            Paint rectPaint = new Paint();
            rectPaint.setColor(Color.RED);
            rectPaint.setStyle(Paint.Style.STROKE);
            rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
            canvas.drawRect(makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx), rectPaint);
            canvas.drawRect(makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx), rectPaint);
        }
    }
}