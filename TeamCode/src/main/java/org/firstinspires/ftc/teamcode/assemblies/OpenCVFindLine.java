package org.firstinspires.ftc.teamcode.assemblies;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import java.util.concurrent.atomic.AtomicInteger;

public class OpenCVFindLine extends OpenCVProcesser {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public boolean viewingPipeline = false;

    enum Stage {
        RAW_IMAGE,
        HSV,
        BLURRED,
        THRESHOLD,
        EDGES
    }
    private Stage stageToRenderToViewport = Stage.RAW_IMAGE;
    private Stage[] stages = Stage.values();

    public OpenCVFindLine () {
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        teamUtil.log("Initializing OpenCVFindLine processor");

        teamUtil.log("Initializing OpenCVFindLine processor - FINISHED");
    }
    public void outputTelemetry () {
        telemetry.addLine("Lowest: " + lastValidBottom + "MidPoint: " + lastValidMidPoint);
        telemetry.addLine(" Area:" + largestArea + "Width: " + width + "thresh Diff:" + differenceFromAverageThreshold);
        if (viewingPipeline) {
            telemetry.addLine("Viewing PipeLine:" + stageToRenderToViewport);
        }
    }

    public void nextView() {

        int currentStageNum = stageToRenderToViewport.ordinal();
        int nextStageNum = currentStageNum + 1;
        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }
        stageToRenderToViewport = stages[nextStageNum];
    }

    public void reset() { // Call this when starting this processor to clear out previous results
        largestArea = 0;
        midpoint = 0;
        bottom = 0;
        width = 0;
        lastValidMidPoint.set(0);
        lastValidBottom.set(0);
    }
    public boolean sawLine () {
        return lastValidMidPoint.get() > 0;
    }
    public boolean details = false;

    public double getLargestArea() {
        return largestArea;
    }
    public double getMidpoint() {
        return midpoint;
    }

    static public int CAMWIDTH = 640; // Options: 320x240, 640x480, 1280x720
    static public int CAMHEIGHT = 480;
    static public int MIDPOINTTARGET = 334; // was (int) (CAMWIDTH * .47);
    static public int MINWIDTH = (int) (CAMWIDTH * .075);
    static public int MAXWIDTH = (int) (CAMWIDTH * .25);

    // These three need to be tuned together
    public int whiteThreshold = 250;

    public double differenceFromAverageThreshold; //can be between 30 and 90
    public int lineExposure = 10 ; // frame exposure in ms (use TestDrive opMode to calibrate)
    public int lineGain = 100; // Was 205
    public Rect cropRect = new  Rect(0,0,CAMWIDTH, (int)(CAMHEIGHT*.45)); // hide pixel stack
    public Rect viewRect = new  Rect(0, cropRect.height+1, CAMWIDTH, (int)(CAMHEIGHT*.53)); // hide pixel stack
    Scalar blackColor = new Scalar(0, 0, 0);

    Mat HSVMat = new Mat();
    Mat greyMat = new Mat();
     // lower bound HSV for yellow
    Scalar highHSV = new Scalar(255, 255, 255); // higher bound HSV for yellow
    Mat blurredMat = new Mat();
    Size blurFactor = new Size(10, 10);



    Mat thresholdMat = new Mat();
    Mat edges = new Mat();
    Mat hierarchy = new Mat();

    List<MatOfPoint> contours = new ArrayList<>();
    double midpoint, width, bottom, largestArea;
    AtomicInteger lastValidMidPoint = new AtomicInteger(0);
    AtomicInteger lastValidBottom = new AtomicInteger(0);

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        int largestArea = 0;
        if (details) teamUtil.log("Process Frame Start");
        Imgproc.rectangle(frame, cropRect, blackColor,-1); // black out the top portion of the frame to avoid seeing stuff off field
        Imgproc.cvtColor(frame, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV
        Imgproc.blur(HSVMat, blurredMat, blurFactor); // get rid of noise

        //double lowHSVValue = this.getAvgValue(frame,viewRect); // EGADS!  We were computing the average Value on a nonblurred RGB mat!
        double lowHSVValue = this.getAvgValue(blurredMat,viewRect); // Get average HSV "Value" for visible area
        differenceFromAverageThreshold = 35; //(255-lowHSVValue)*0.3
        if (details) teamUtil.log("Average V: "+ lowHSVValue + " Threshold: "+ (lowHSVValue+differenceFromAverageThreshold));
        if (details) teamUtil.log("Calculated Threshold Difference: "+ differenceFromAverageThreshold);
        Scalar lowHSV = new Scalar(0, 0, lowHSVValue+differenceFromAverageThreshold); // compute the low threshold

        Core.inRange(blurredMat, lowHSV, highHSV, thresholdMat); // find areas that we see as "white"

        //Imgproc.cvtColor(frame, greyMat, Imgproc.COLOR_RGB2GRAY); // convert to Greyscale
        //Imgproc.blur(greyMat, blurredMat, blurFactor); // get rid of noise
        //Imgproc.threshold(greyMat,thresholdMat,whiteThreshold,255,Imgproc.THRESH_BINARY);
        //Imgproc.adaptiveThreshold(greyMat,thresholdMat,255,Imgproc.ADAPTIVE_THRESH_MEAN_C,Imgproc.THRESH_BINARY,381,12);

        Imgproc.Canny(thresholdMat, edges, 100, 300); // find edges of white areas
        contours.clear(); // empty the list from last time
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); // find countours around white areas
        if (!contours.isEmpty()) {
            // find the bounding rectangles of those contours, compute midpoint, and prepare labeled mat
            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                /*
                if (boundRect[i].br().y > bottom) { // "lowest" one yet
                    bottom = boundRect[i].br().y;
                    midpoint = (boundRect[i].br().x - boundRect[i].tl().x)/2 + boundRect[i].tl().x;
                    largestArea = (boundRect[i].br().y - boundRect[i].tl().y) * (boundRect[i].br().x - boundRect[i].tl().x);
                }
                */
                double area = (boundRect[i].br().y - boundRect[i].tl().y) * (boundRect[i].br().x - boundRect[i].tl().x);
                if (area > largestArea) {
                    largestArea = (int) area; // biggest object seen so far in this frame
                    //midpoint = (boundRect[i].br().x - boundRect[i].tl().x)/2 + boundRect[i].tl().x;
                    //bottom = boundRect[i].br().y;
                    width = boundRect[i].br().x - boundRect[i].tl().x;
                    if (width > MINWIDTH && width < MAXWIDTH) { // Looks like white tape!
                        lastValidMidPoint.set((int) ((boundRect[i].br().x - boundRect[i].tl().x)/2 + boundRect[i].tl().x));
                        lastValidBottom.set((int)boundRect[i].br().y);
                        if (details) teamUtil.log("Tape Mid:" + lastValidMidPoint.get() + " Bot:" +lastValidBottom.get());
                    }
                }
            }
            if (details) teamUtil.log("Process Frame End");

            return boundRect; // return the array of bounding rectangles we found
        }
        if (details) teamUtil.log("Process Frame End - Nothing Found");

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // draw rectangles around the objects we found
        if (viewingPipeline) {
            //Bitmap bmp = Bitmap.createBitmap(greyMat.cols(), greyMat.rows(), Bitmap.Config.ARGB_8888);
            Bitmap bmp = Bitmap.createBitmap(HSVMat.cols(), HSVMat.rows(), Bitmap.Config.ARGB_8888);
            switch (stageToRenderToViewport) {
                case BLURRED: { Utils.matToBitmap(blurredMat, bmp); break;}
                //case HSV: { Utils.matToBitmap(greyMat, bmp); break; }
                case HSV: { Utils.matToBitmap(HSVMat, bmp); break; }
                case THRESHOLD: { Utils.matToBitmap(thresholdMat, bmp); break;}
                case EDGES: { Utils.matToBitmap(edges, bmp); break;}
                case RAW_IMAGE: {break;} // no bitmap needed
            }
            Bitmap resizedBitmap = Bitmap.createScaledBitmap(bmp, (int)(CAMWIDTH*scaleBmpPxToCanvasPx), (int)(CAMHEIGHT*scaleBmpPxToCanvasPx), false);
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