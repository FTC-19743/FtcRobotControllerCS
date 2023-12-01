package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.assemblies.OpenCVFindLine;
import org.firstinspires.ftc.teamcode.assemblies.OpenCVPropFinder;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name="Calibrate CV", group = "Calibration")
//@Disabled
public class calibrateCV extends LinearOpMode
{
    // Image Processors and Cameras
    private VisionPortal rearVisionPortal, sideVisionPortal, frontVisionPortal;

    public AprilTagProcessor aprilTag;
    private int     aprilTagExposure = 5 ; // frame exposure in ms (use TestDrive opMode to calibrate)
    private int     aprilTagGain = 230; // High gain to compensate for fast exposure  DOESN'T WORK DUE TO FTC BUG
    public OpenCVFindLine findLineProcesser;
    //public OpenCVYellowPixelDetector findPixelProcesser;

    public OpenCVPropFinder findTeamPropProcesser;

    public double noAprilTag = 999.0;
    public float TAG_CENTER_TO_CENTER = 15.2f;


    boolean aprilTagProcessorRunning = false;
    boolean findLineProcessorRunning = false;
    boolean findTeamPropProcessorRunning = false;
    public WebcamName frontCam;
    public WebcamName backCam;
    public WebcamName sideCam;
    public enum cvCam {REAR_APRILTAG, SIDE_PROP, FRONT_LINE};
    public double CMS_PER_INCH = 2.54;



    // Manually set the camera gain and exposure.  Camera must be actively streaming
    // Returns true if controls are set.
    private boolean updateCVManualExposure(VisionPortal portal, int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (portal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = portal.getCameraControl(GainControl.class);
            if (gainControl == null) { // FTC software has a bug where this isn't supported on switchableCameras!
            } else {
                gainControl.setGain(gain);
            }
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }
    public void initCV(boolean enableLiveView) { //should be false for comp code
        teamUtil.log("Initializing CV");

        // Setup a separate VisionPortal for each camera.  We will switch between cameras and processers as needed
        aprilTag = new AprilTagProcessor.Builder().build();
        //findPixelProcesser = new OpenCVYellowPixelDetector();
        findLineProcesser = new OpenCVFindLine();
        findTeamPropProcesser = new OpenCVPropFinder();

        backCam = hardwareMap.get(WebcamName.class, "Webcam Rear");
        frontCam = hardwareMap.get(WebcamName.class, "Webcam Front");
        if (teamUtil.alliance == teamUtil.Alliance.RED) {
            sideCam = hardwareMap.get(WebcamName.class, "Webcam Right");
        } else {
            sideCam = hardwareMap.get(WebcamName.class, "Webcam Left");
        }

        // In order to build multiple VisionPortals, you must first set up a MultiPortalView, even if you don't plan to run them at the same time
        int[] visionPortalViewIDs = VisionPortal.makeMultiPortalView(3, VisionPortal.MultiPortalLayout.HORIZONTAL);

        // Set up rear Camera
        VisionPortal.Builder rearBuilder = new VisionPortal.Builder();
        rearBuilder.setCamera(backCam);
        rearBuilder.setLiveViewContainerId(visionPortalViewIDs[0]);
        rearBuilder.enableLiveView(enableLiveView);
        rearBuilder.setAutoStopLiveView(true);
        // Can also set resolution and stream format if we want to optimize resource usage.        rearBuilder.addProcessor(aprilTag);
        rearVisionPortal = rearBuilder.build();

        // Set up side Camera
        VisionPortal.Builder sideBuilder = new VisionPortal.Builder();
        sideBuilder.setCamera(backCam);
        sideBuilder.setLiveViewContainerId(visionPortalViewIDs[1]);
        sideBuilder.enableLiveView(enableLiveView);
        sideBuilder.setAutoStopLiveView(true);
        // Can also set resolution and stream format if we want to optimize resource usage.
        sideBuilder.addProcessor(findTeamPropProcesser);
        sideVisionPortal = sideBuilder.build();

        // Set up front Camera
        VisionPortal.Builder frontBuilder = new VisionPortal.Builder();
        frontBuilder.setCamera(backCam);
        frontBuilder.setLiveViewContainerId(visionPortalViewIDs[2]);
        frontBuilder.enableLiveView(enableLiveView);
        frontBuilder.setAutoStopLiveView(true);
        // Can also set resolution and stream format if we want to optimize resource usage.        frontBuilder.addProcessor(findLineProcesser);
        frontVisionPortal = frontBuilder.build();

        teamUtil.log("Initializing Drive CV - FINISHED");
    }

    public void stopCV() {
        if (rearVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            rearVisionPortal.stopStreaming();
        }
        if (frontVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            frontVisionPortal.stopStreaming();
        }
        if (sideVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            frontVisionPortal.stopStreaming();
        }
        aprilTagProcessorRunning = false;
        findLineProcessorRunning = false;
        findTeamPropProcessorRunning = false;
    }
    public void switchCV(cvCam newCam) {
        stopCV();
        switch (newCam) {
            case REAR_APRILTAG:
                rearVisionPortal.resumeStreaming();
                aprilTagProcessorRunning = true;
                break;
            case FRONT_LINE:
                frontVisionPortal.resumeStreaming();
                findLineProcessorRunning = true;
                break;
            case SIDE_PROP:
                sideVisionPortal.resumeStreaming();
                findTeamPropProcessorRunning = true;
                break;
        }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Code specific to calibrateCV op mode
    int currentCam = 1;
    private int[]     myExposure  ;
    private int[]     minExposure ;
    private int[]     maxExposure ;
    private int[]     myGain      ;
    private int[]     minGain ;
    private int[]     maxGain ;
    public void toggleCV() {
        currentCam++;
        if (currentCam > 3) {
            currentCam = 1;
        }
        switch(currentCam) {
            case 1:
                switchCV(cvCam.REAR_APRILTAG);
                break;
            case 2:
                switchCV(cvCam.FRONT_LINE);
                break;
            case 3:
                switchCV(cvCam.SIDE_PROP);
                break;
            default:
        }
    }

    public void writeTelemetry () {
        telemetry.addData("Exposure","%d  (%d - %d)", myExposure[currentCam], minExposure[currentCam], maxExposure[currentCam]);
        telemetry.addData("Gain","%d  (%d - %d)", myGain[currentCam], minGain[currentCam], maxGain[currentCam]);
        if (aprilTagProcessorRunning) {
            //findPixelProcesser.outputTelemetry();
            //TODO: Consider adding backdrop offset results to this;
            telemetry.addLine("RearCam:" + rearVisionPortal.getCameraState()+ " FPS:" + rearVisionPortal.getFps());
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addLine("AprilTag: " + detection.id + " X:" + detection.ftcPose.x * CMS_PER_INCH + " Y:" + detection.ftcPose.y * CMS_PER_INCH);
            }
        } else if (findLineProcessorRunning) {
            telemetry.addLine("FrontCam:" + frontVisionPortal.getCameraState()+ " FPS:" + frontVisionPortal.getFps());
            findLineProcesser.outputTelemetry();
        }
        else if (findTeamPropProcessorRunning) {
            telemetry.addLine("SideCam:" + sideVisionPortal.getCameraState()+ " FPS:" + sideVisionPortal.getFps());
            findTeamPropProcesser.outputTelemetry();
        }
    }
    @Override public void runOpMode()
    {
        teamUtil.init(this);
        TeamGamepad gamepad = new TeamGamepad();
        gamepad.initilize(true);
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            if(gamepad.wasLeftPressed()){ teamUtil.alliance = teamUtil.Alliance.RED;}
            if(gamepad.wasRightPressed()){ teamUtil.alliance = teamUtil.Alliance.BLUE;}

            teamUtil.telemetry.addLine("RED or BLUE? (use Game Pad 1 DPad)");
            teamUtil.telemetry.addLine(teamUtil.alliance == teamUtil.Alliance.RED ? "RED Alliance" : "BLUE Alliance");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }

        initCV(true);
        // Establish Min and Max Gains and Exposure.  Then set a low exposure with high gain
        getAllCameraSettings();
        for (int cam = 1;cam < 4;cam++)
        {
            myExposure[cam] = Math.min(5, minExposure[cam]);
            myGain[cam] = maxGain[cam];
            updateCVManualExposure(cam==1 ? rearVisionPortal : cam==2 ? frontVisionPortal : sideVisionPortal, myExposure[cam], myGain[cam]);
        }

        // All 3 cams are running at this point
        switchCV(cvCam.REAR_APRILTAG);
        currentCam = 1;

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addLine("Use Left bump/trig to adjust Exposure.");
            telemetry.addLine("Use Right bump/trig to adjust Gain.\n");
            writeTelemetry();

            telemetry.update();

            if (gamepad.wasLeftBumperPressed()) {
                myExposure[currentCam] = Range.clip(myExposure[currentCam] + 1, minExposure[currentCam], maxExposure[currentCam]);
                updateCVManualExposure(currentCam==1 ? rearVisionPortal : currentCam==2 ? frontVisionPortal : sideVisionPortal, myExposure[currentCam], myGain[currentCam]);
            }
            if (gamepad.wasLeftTriggerPressed()) {
                myExposure[currentCam] = Range.clip(myExposure[currentCam] - 1, minExposure[currentCam], maxExposure[currentCam]);
                updateCVManualExposure(currentCam==1 ? rearVisionPortal : currentCam==2 ? frontVisionPortal : sideVisionPortal, myExposure[currentCam], myGain[currentCam]);
            }
            if (gamepad.wasRightBumperPressed()) {
                myGain[currentCam] = Range.clip(myGain[currentCam] + 1, minGain[currentCam], maxGain[currentCam]);
                updateCVManualExposure(currentCam==1 ? rearVisionPortal : currentCam==2 ? frontVisionPortal : sideVisionPortal, myExposure[currentCam], myGain[currentCam]);
            }
            if (gamepad.wasRightTriggerPressed()) {
                myGain[currentCam] = Range.clip(myGain[currentCam] - 1, minGain[currentCam], maxGain[currentCam]);
                updateCVManualExposure(currentCam==1 ? rearVisionPortal : currentCam==2 ? frontVisionPortal : sideVisionPortal, myExposure[currentCam], myGain[currentCam]);
            }
            if (gamepad.wasUpPressed()) {
                if (currentCam==1) {
                    // TODO: adjust saturation thresholds and maybe rectangles
                }
                if (currentCam==2) {
                    findLineProcesser.whiteThreshold--;
                }
            }
            if (gamepad.wasDownPressed()) {
                if (currentCam==1) {
                    // TODO: adjust saturation thresholds
                }
                if (currentCam==2) {
                    findLineProcesser.whiteThreshold++;
                }
            }

            sleep(20);
        }
    }

    public void getAllCameraSettings() {
        teamUtil.log("Getting Rear Camera Settings");
        getCameraSettings(1,rearVisionPortal);
        teamUtil.log("Getting Front Camera Settings");
        getCameraSettings(2,frontVisionPortal);
        teamUtil.log("Getting Side Camera Settings");
        getCameraSettings(3,sideVisionPortal);
    }

    private void getCameraSettings(int cam, VisionPortal portal) {
        // Ensure Vision Portal has been setup.
        if (portal == null) {
            return;
        }

        // Wait for the camera to be open
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            minExposure[cam] = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure[cam] = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = portal.getCameraControl(GainControl.class);
            if (gainControl == null) { // FTC software has a bug where this isn't supported on switchableCameras!
                minGain[cam] = 0;
                maxGain[cam] = 0;
            } else {
                minGain[cam] = gainControl.getMinGain();
                maxGain[cam] = gainControl.getMaxGain();
            }
        }
    }
}
