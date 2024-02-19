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
import java.util.concurrent.atomic.AtomicBoolean;

public class OpenCVYellowPixelDetector extends OpenCVProcesser {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public AtomicBoolean foundPixel = new AtomicBoolean(false);
    public int pixelMidPoint = 0;
    public int minArea = 500;
    public int farthestRight = 0;
    public int farthestLeft = 0;
    public boolean viewingPipeline = false;
    enum Stage {
        RAW_IMAGE,
        BLURRED,
        HSV,
        THRESHOLD,
        EDGES
    }
    private Stage stageToRenderToViewport = Stage.RAW_IMAGE;
    private Stage[] stages = Stage.values();

    public OpenCVYellowPixelDetector () {
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        teamUtil.log("Initializing OpenCVYellowPixelDetector processor");

        teamUtil.log("Initializing OpenCVYellowPixelDetector processor - FINISHED");
    }
    public void outputTelemetry () {
        telemetry.addLine("Found One: " + foundPixel.get() + "MidPoint: " + pixelMidPoint + " Area:" + largestArea);
    }

    public void nextView() {

        int currentStageNum = stageToRenderToViewport.ordinal();
        int nextStageNum = currentStageNum + 1;
        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }
        stageToRenderToViewport = stages[nextStageNum];
    }

    public double getLargestArea() {
        return largestArea;
    }
    public double getMidpoint() {
        return pixelMidPoint;
    }

    Mat HSVMat = new Mat();
    Scalar lowHSV = new Scalar(15, 100, 100); // lower bound HSV for yellow
    Scalar highHSV = new Scalar(35, 255, 255); // higher bound HSV for yellow
    //    Scalar lowHSV = new Scalar(20, 100, 100); // lower bound HSV for yellow
    //Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow
    Mat blurredMat = new Mat();
    Size blurFactor = new Size(50, 50);
    Mat thresholdMat = new Mat();
    Mat edges = new Mat();
    Mat hierarchy = new Mat();

    List<MatOfPoint> contours = new ArrayList<>();
    double largestArea;

    public void reset() {
        foundPixel.set(false);
        pixelMidPoint = 0;
        largestArea = 0;
        farthestLeft = 640;
        farthestRight = 0;
    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        boolean details = false;
        //largestArea = 0;
        if (details) teamUtil.log("Yellow Pixel Detector: Process Frame");
        Imgproc.cvtColor(frame, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV
        Imgproc.blur(HSVMat, blurredMat, blurFactor); // get rid of noise
        Core.inRange(HSVMat, lowHSV, highHSV, thresholdMat);
        Imgproc.Canny(thresholdMat, edges, 100, 300);
        contours.clear(); // empty the list from last time
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        if (!contours.isEmpty()) {
            // find the bounding rectangles of those contours, compute midpoint, and prepare labeled mat
            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];

            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                double area = (boundRect[i].br().y - boundRect[i].tl().y) * (boundRect[i].br().x - boundRect[i].tl().x);

                if (area > minArea) { // Found one
                    foundPixel.set(true);
                    if (boundRect[i].tl().x < farthestLeft) farthestLeft = (int)boundRect[i].tl().x;
                    if (boundRect[i].br().x > farthestRight) farthestRight = (int)boundRect[i].br().x;
                    if (area > largestArea) {
                        largestArea = area;
                        pixelMidPoint = (int) ((boundRect[i].br().x - boundRect[i].tl().x)/2 + boundRect[i].tl().x);
                    }
                }
            }
            if (details) teamUtil.log("FoundOne:" + foundPixel.get()+ " Left: " + farthestLeft + " Right: " + farthestRight + " Area: " +largestArea+ " Mid: " +pixelMidPoint);

            return boundRect; // return the array of bounding rectangles we found
        }else{
            teamUtil.log("No Detections");
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // draw rectangles around the objects we found
        if (viewingPipeline) {
            Bitmap bmp = Bitmap.createBitmap(HSVMat.cols(), HSVMat.rows(), Bitmap.Config.ARGB_8888);
            switch (stageToRenderToViewport) {
                case BLURRED: { Utils.matToBitmap(blurredMat, bmp); break;}
                case HSV: { Utils.matToBitmap(HSVMat, bmp); break; }
                case THRESHOLD: { Utils.matToBitmap(thresholdMat, bmp); break;}
                case EDGES: { Utils.matToBitmap(edges, bmp); break;}
                default: {}
            }
            Bitmap resizedBitmap = Bitmap.createScaledBitmap(bmp, (int)(640*scaleBmpPxToCanvasPx), (int)(480*scaleBmpPxToCanvasPx), false);
            canvas.drawBitmap(resizedBitmap, 0,0,null);
        }

        if (userContext != null) {
            Rect[] boundRects = (Rect[]) userContext;
            Paint rectPaint = new Paint();
            rectPaint.setColor(Color.RED);
            rectPaint.setStyle(Paint.Style.STROKE);
            rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
            for (int i = 0; i < boundRects.length; i++) {
                canvas.drawRect(makeGraphicsRect(boundRects[i], scaleBmpPxToCanvasPx), rectPaint);
            }
        }
    }
}