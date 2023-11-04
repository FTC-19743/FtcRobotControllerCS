package org.firstinspires.ftc.teamcode.assemblies;

import android.graphics.Canvas;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.CanvasAnnotator;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class OpenCVFindLine implements VisionProcessor {
    HardwareMap hardwareMap;
    OpenCvWebcam webcam;
    public LinePipeline LinePipe;
    Telemetry telemetry;

    public void init(int width, int height, CameraCalibration calibration) {
        LinePipe = new LinePipeline();
        teamUtil.log("Initializing OpenCVDetector");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(LinePipe);
        teamUtil.log("Finished Initializing OpenCVDetector");
    }

    public Object processFrame(Mat input, long captureTimeNanos) {
        return LinePipe.processFrame(input);
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        ((CanvasAnnotator) userContext).draw(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity);
    }

    public static class LinePipeline extends OpenCvPipeline {

        int areaThreshold = 150;
        Mat HSVMat = new Mat();


        Scalar lowHSV = new Scalar(0, 0, 215); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(255, 255, 255); // higher bound HSV for yellow ol di 33

        Mat bluredMat = new Mat();
        Size blurFactor = new Size(5, 5);

        Mat edges = new Mat();

        Mat thresholdMat = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();


        Mat hierarchy = new Mat();

        Mat labeled = new Mat();
        Scalar labelColor = new Scalar(0, 0, 255);


        int areaDetectionThreshold = 30;
        int largestArea = 0;
        int lastLargestArea = 0;


        public boolean foundLine = false;
        enum Stage {
            RAW_IMAGE,
            HSV,
            THRESHOLD,
            EDGES,
            LABELED
        }

        enum Color {
            WHITE,
            OTHER
        }

        public Color foundColor = Color.OTHER;

        private Stage stageToRenderToViewport = Stage.RAW_IMAGE;
        private Stage[] stages = Stage.values();

        public int getLargestArea() {
            return (int) lastLargestArea;
        }


        public void nextView() {

            int currentStageNum = stageToRenderToViewport.ordinal();
            int nextStageNum = currentStageNum + 1;
            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }
            stageToRenderToViewport = stages[nextStageNum];
        }


        @Override
        public Mat processFrame(Mat input) {

            // Convert to HSV
            Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV);
            // if something is wrong, bail out
            if (HSVMat.empty()) {
                return input;
            }
            largestArea = 0;
            //Imgproc.blur(HSVMat, bluredMat, blurFactor); // get rid of noise
            // We'll get  black and white images for each color range.
            // The white regions represent the color we are looking for.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range

            Core.inRange(HSVMat, lowHSV, highHSV, thresholdMat);

            // Core.inRange(HSVMat, pinkLowHSV, pinkHighHSV, thresholdMatPink);

            // Use Canny Edge Detection to find edges (might have to tune the thresholds for hysteresis)
            Imgproc.Canny(thresholdMat, edges, 100, 300);
            // Imgproc.Canny(thresholdMatPink, edgesPink, 100, 300);

            // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
            // Oftentimes the edges are disconnected. findContours connects these edges.
            contours.clear(); // empty the list from last time
            // contoursPink.clear(); // empty the list from last time
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            // Imgproc.findContours(edgesPink, contoursPink, hierarchyPink, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            input.copyTo(labeled); // make a copy of original to add labels to
            // input.copyTo(labeledPink); // make a copy of original to add labels to
            double maxY = 0;

            // if no contours, we didn't find anything
            if (!contours.isEmpty()) {
                // find the bounding rectangles of those contours, compute midpoint, and prepare labeled mat
                MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
                Rect[] boundRect = new Rect[contours.size()];
                for (int i = 0; i < contours.size(); i++) {
                    contoursPoly[i] = new MatOfPoint2f();
                    Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                    boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                    Imgproc.rectangle(labeled, boundRect[i], labelColor, 5);


                        double area = (boundRect[i].br().y - boundRect[i].tl().y) * (boundRect[i].br().x - boundRect[i].tl().x);
                        if (area > largestArea) {
                            largestArea = (int) area;
                        }

                }
            }

            if (largestArea > areaThreshold) {
                    foundColor = Color.WHITE;
            } else {
                    foundColor = Color.OTHER;
            }

            lastLargestArea = largestArea;
            /*
            if (contoursPink.isEmpty()) {
                foundPink = false;
            } else {
                foundPink = true;
                // find the bounding rectangles of those contours, compute midpoint, and prepare labeled mat
                int farLeft = 5000;
                int farRight = 0;
                MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contoursPink.size()];
                Rect[] boundRect = new Rect[contoursPink.size()];
                for (int i = 0; i < contoursPink.size(); i++) {
                    contoursPoly[i] = new MatOfPoint2f();
                    Imgproc.approxPolyDP(new MatOfPoint2f(contoursPink.get(i).toArray()), contoursPoly[i], 3, true);
                    boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                    Imgproc.rectangle(labeledPink, boundRect[i], labelColor, 5);
                }
            }

             */

            switch (stageToRenderToViewport) {
                case RAW_IMAGE: {
                    return input;
                }
                case HSV: {
                    return thresholdMat;
                    //return bluredMat;
                }
                case THRESHOLD: {
                    return HSVMat;
                }
                default: {
                    return input;
                }
            }
        }

    }
}