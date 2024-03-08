package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.assemblies.Drive;
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
    public enum cvCam {NONE, REAR_APRILTAG, SIDE_PROP, FRONT_LINE};
    public cvCam currentCam = cvCam.NONE;
    public double CMS_PER_INCH = 2.54;



    // Manually set the camera gain and exposure.  Camera must be actively streaming
    // Returns true if controls are set.
    private boolean updateCVManualExposure(VisionPortal portal, int exposureMS, int gain) {
        teamUtil.log("Setting Manual Exposure");
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
                teamUtil.log("Switching to Manual Mode");
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            teamUtil.log("Setting Exposure");

            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = portal.getCameraControl(GainControl.class);
            if (gainControl == null) { // FTC software has a bug where this isn't supported on switchableCameras!
            } else {
                teamUtil.log("Setting Gain");
                gainControl.setGain(gain);
            }
            sleep(20);
            teamUtil.log("Setting Manual - Finished");
            return (true);
        } else {
            teamUtil.log("Setting Manual - Finished");
            return (false);
        }
    }

    public void setAutoExposure(VisionPortal portal) {
        teamUtil.log("Set Auto Exposure");

        // Wait for the camera to be STREAMING to use CameraControls (FTC SDK requirement)
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                teamUtil.pause(20);
            }
        }
        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        if (exposureControl == null) {
            teamUtil.log("Failed to get ExposureControl object");
            return;
        }
        teamUtil.log("Current Mode: "+ exposureControl.getMode());

        if (exposureControl.getMode() != ExposureControl.Mode.Auto) {
            teamUtil.log("Switching to Auto Mode");
            exposureControl.setMode(ExposureControl.Mode.Auto);
            teamUtil.pause(50);
        }
        teamUtil.log("Set Auto Exposure - FINISHED");
    }

    // Assumes portal is either currently STREAMING or in process of starting STREAMING
    public void stopStreaming(VisionPortal portal) {
        teamUtil.log("Stop Streaming ");

        while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            teamUtil.log("Waiting for Camera to finish coming on line: " + portal.getCameraState());
            teamUtil.pause(100);
        }
        portal.stopStreaming();
        while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY)) {
            teamUtil.log("Waiting for Camera to stop: " + portal.getCameraState());
            teamUtil.pause(100);
        }
    }
    public void initCV(boolean enableLiveView) { //should be false for comp code
        teamUtil.log("Initializing CV: LiveView: "+enableLiveView);

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

        teamUtil.log("Getting Multiportal");

        // In order to build multiple VisionPortals, you must first set up a MultiPortalView, even if you don't plan to run them at the same time
        int[] visionPortalViewIDs = VisionPortal.makeMultiPortalView(3, VisionPortal.MultiPortalLayout.HORIZONTAL);
        //teamUtil.log("Multiportal: "+visionPortalViewIDs[0]+"/"+visionPortalViewIDs[1]+"/"+visionPortalViewIDs[2]);


        // Set up rear Camera
        teamUtil.log("Setting up rearVisionPortal");
        VisionPortal.Builder rearBuilder = new VisionPortal.Builder();
        rearBuilder.setCamera(backCam);
        rearBuilder.setLiveViewContainerId(visionPortalViewIDs[0]);
        if (!enableLiveView) {
            rearBuilder.enableLiveView(false);
        }
        rearBuilder.addProcessor(aprilTag);
        // Can also set resolution and stream format if we want to optimize resource usage.
        rearVisionPortal = rearBuilder.build();
        stopStreaming(rearVisionPortal);

        // Set up side Camera
        teamUtil.log("Setting up sideVisionPortal");
        VisionPortal.Builder sideBuilder = new VisionPortal.Builder();
        sideBuilder.setCamera(sideCam);
        sideBuilder.setLiveViewContainerId(visionPortalViewIDs[1]);
        if (!enableLiveView) {
            sideBuilder.enableLiveView(false);
        }
        sideBuilder.addProcessor(findTeamPropProcesser);
        // Can also set resolution and stream format if we want to optimize resource usage.
        sideVisionPortal = sideBuilder.build();
        stopStreaming(sideVisionPortal);

        // Set up front Camera
        teamUtil.log("Setting up frontVisionPortal");
        VisionPortal.Builder frontBuilder = new VisionPortal.Builder();
        frontBuilder.setCamera(frontCam);
        frontBuilder.setLiveViewContainerId(visionPortalViewIDs[2]);
        if (!enableLiveView) {
            frontBuilder.enableLiveView(false);
        }
        frontBuilder.addProcessor(findLineProcesser);
        // Can also set resolution and stream format if we want to optimize resource usage.
        frontVisionPortal = frontBuilder.build();
        stopStreaming(frontVisionPortal);

        currentCam = cvCam.NONE;

        teamUtil.log("Initializing Drive CV - FINISHED");
    }

    public void stopCV() {
        teamUtil.log("Stopping any running Cams");
        if (rearVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            teamUtil.log("Stopping Rear Cam");
            rearVisionPortal.stopStreaming();
        }
        if (frontVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            teamUtil.log("Stopping Front Cam");
            frontVisionPortal.stopStreaming();
        }
        if (sideVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            teamUtil.log("Stopping Side Cam");
            frontVisionPortal.stopStreaming();
        }
        aprilTagProcessorRunning = false;
        findLineProcessorRunning = false;
        findTeamPropProcessorRunning = false;
    }
    public void switchCV(cvCam newCam) {
        teamUtil.log("Switching CV to " + newCam);
        teamUtil.log("Stopping " + currentCam);
        switch (currentCam) {
            case REAR_APRILTAG:
                rearVisionPortal.stopStreaming();
                aprilTagProcessorRunning = false;
                break;



            case FRONT_LINE:

                teamUtil.log("Switching CV to " + newCam);
                frontVisionPortal.stopStreaming();
                findLineProcessorRunning = false;
                break;
            case SIDE_PROP:
                teamUtil.log("Switching CV to " + newCam);
                sideVisionPortal.stopStreaming();
                findTeamPropProcessorRunning = false;
                break;
        }
        teamUtil.log("Starting " + newCam);
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
        currentCam = newCam;
        teamUtil.log("switchCV - Finished ");
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Code specific to calibrateCV op mode
    int currentCamNum = 0;
    private int[]     myExposure = new int[4] ;
    private int[]     minExposure = new int[4] ;
    private int[]     maxExposure = new int[4] ;
    private int[]     myGain = new int[4]      ;
    private int[]     minGain = new int[4]  ;
    private int[]     maxGain = new int[4]  ;
    public void toggleCV() {
        currentCamNum++;
        if (currentCamNum > 3) {
            currentCamNum = 1;
        }
        switch(currentCamNum) {
            case 1:
                switchCV(cvCam.REAR_APRILTAG);
                updateCVManualExposure(rearVisionPortal, 3, 255);

                break;

            case 2:
                switchCV(cvCam.FRONT_LINE);
                updateCVManualExposure(frontVisionPortal, 10, 100);

                break;
            case 3:
                switchCV(cvCam.SIDE_PROP);
                updateCVManualExposure(sideVisionPortal, 10, 100);

                break;
            default:
        }
    }

    public void writeTelemetry () {
        telemetry.addData("Exposure","%d  (%d - %d)", myExposure[currentCamNum], minExposure[currentCamNum], maxExposure[currentCamNum]);
        telemetry.addData("Gain","%d  (%d - %d)", myGain[currentCamNum], minGain[currentCamNum], maxGain[currentCamNum]);
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
            telemetry.addLine("Threshold Difference From Average: " + findLineProcesser.differenceFromAverageThreshold);
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
        teamUtil.telemetry.addLine("Initializing CV. Please wait.");
        teamUtil.telemetry.update();
        initCV(true); // All portals and cameras are now ready, nothing is streaming

        teamUtil.telemetry.addLine("Setting up Exposure/Gain data. Please wait.");
        teamUtil.telemetry.update();
        // Establish Min and Max Gains and Exposure.  Then set a low exposure with high gain
        getAllCameraSettings();
        teamUtil.pause(1500);

        // All cams should be in READY state.
        switchCV(cvCam.REAR_APRILTAG);
        currentCamNum = 1;
        while (!isStopRequested() && (rearVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            teamUtil.log("Waiting for rear camera to come on line: " + rearVisionPortal.getCameraState());
            sleep(100);
        }

        telemetry.addLine("Ready to Start Op Mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            gamepad.loop();

            telemetry.addLine("Use Left bump/trig to adjust Exposure.");
            telemetry.addLine("Use Right bump/trig to adjust Gain.\n");
            writeTelemetry();

            telemetry.update();

            if (gamepad.wasLeftBumperPressed()) {
                myExposure[currentCamNum] = Range.clip(myExposure[currentCamNum] + 1, minExposure[currentCamNum], maxExposure[currentCamNum]);
                updateCVManualExposure(currentCamNum==1 ? rearVisionPortal : currentCamNum==2 ? frontVisionPortal : sideVisionPortal, myExposure[currentCamNum], myGain[currentCamNum]);
            }
            if (gamepad.wasLeftTriggerPressed()) {
                myExposure[currentCamNum] = Range.clip(myExposure[currentCamNum] - 1, minExposure[currentCamNum], maxExposure[currentCamNum]);
                updateCVManualExposure(currentCamNum==1 ? rearVisionPortal : currentCamNum==2 ? frontVisionPortal : sideVisionPortal, myExposure[currentCamNum], myGain[currentCamNum]);
            }
            if (gamepad.wasRightBumperPressed()) {
                myGain[currentCamNum] = Range.clip(myGain[currentCamNum] + 10, minGain[currentCamNum], maxGain[currentCamNum]);
                updateCVManualExposure(currentCamNum==1 ? rearVisionPortal : currentCamNum==2 ? frontVisionPortal : sideVisionPortal, myExposure[currentCamNum], myGain[currentCamNum]);
            }
            if (gamepad.wasRightTriggerPressed()) {
                myGain[currentCamNum] = Range.clip(myGain[currentCamNum] - 10, minGain[currentCamNum], maxGain[currentCamNum]);
                updateCVManualExposure(currentCamNum==1 ? rearVisionPortal : currentCamNum==2 ? frontVisionPortal : sideVisionPortal, myExposure[currentCamNum], myGain[currentCamNum]);
            }
            if (gamepad.wasUpPressed()) {
                if (currentCamNum==3) {
                    // TODO: adjust saturation thresholds and maybe rectangles
                }
                if (currentCamNum==2) {
                    findLineProcesser.differenceFromAverageThreshold++;
                }
            }
            if (gamepad.wasDownPressed()) {
                if (currentCamNum==3) {
                    // TODO: adjust saturation thresholds
                }
                if (currentCamNum==2) {
                    findLineProcesser.differenceFromAverageThreshold--;
                }
            }
            if (gamepad.wasYPressed()){
                toggleCV();
            }
            if (gamepad.wasAPressed()){
                switch (currentCam) {
                    case REAR_APRILTAG:
                        setAutoExposure(rearVisionPortal);

                        break;

                    case FRONT_LINE:
                        setAutoExposure(frontVisionPortal);

                        break;
                    case SIDE_PROP:
                        setAutoExposure(sideVisionPortal);

                        break;
                }
            }

            sleep(20);
        }
    }

    public void getAllCameraSettings() {
        teamUtil.log("Getting Rear Camera Settings");
        rearVisionPortal.resumeStreaming();
        getCameraSettings(1,rearVisionPortal, Drive.aprilTagExposure ,Drive.aprilTagGain);
        rearVisionPortal.stopStreaming();
        teamUtil.log("Getting Front Camera Settings");
        frontVisionPortal.resumeStreaming();
        getCameraSettings(2,frontVisionPortal, findLineProcesser.lineExposure, findLineProcesser.lineGain);
        frontVisionPortal.stopStreaming();
        teamUtil.log("Getting Side Camera Settings");
        sideVisionPortal.resumeStreaming();
        getCameraSettings(3,sideVisionPortal, findTeamPropProcesser.propExposure, findTeamPropProcesser.propGain);
        sideVisionPortal.stopStreaming();
    }

    private void getCameraSettings(int cam, VisionPortal portal, int startExposure, int startGain) {
        // Ensure Vision Portal has been setup.
        if (portal == null) {
            teamUtil.log("Null portal!");
            return;
        }

        // Wait for the camera to be open
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                teamUtil.log("Waiting for camera to come on line");
                sleep(100);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            teamUtil.log("getting exposure control limits");
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            minExposure[cam] = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure[cam] = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            teamUtil.log("getting gain control limits");
            GainControl gainControl = portal.getCameraControl(GainControl.class);
            if (gainControl == null) { // FTC software has a bug where this isn't supported on switchableCameras!
                minGain[cam] = 0;
                maxGain[cam] = 0;
            } else {
                minGain[cam] = gainControl.getMinGain();
                maxGain[cam] = gainControl.getMaxGain();
            }
        }
        myExposure[cam] = startExposure; //Math.min(5, minExposure[cam]);
        myGain[cam] = startGain; //maxGain[cam];

        updateCVManualExposure(portal, myExposure[cam], myGain[cam]);
    }
}
