package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class Drive {
////////////////////////////////////////////////////////////////////////////////////////////////
//
//    Drive class for mecanum wheels
//
////////////////////////////////////////////////////////////////////////////////////////////////

    // constants
    HardwareMap hardwareMap;
    Telemetry telemetry;
    //public bottomColorSensor tapeSensor1, tapeSensor2;
    public DigitalChannelController proxmityBackRight;


    public BNO055IMU imu; //This variable is the imu
    public static double HEADING_OFFSET; // offset between IMU heading and field
    public double lastVelocity;
    public boolean holdingHeading = false;
    public double heldHeading = 0;
    public AtomicBoolean movingAutonomously = new AtomicBoolean(false); // true under autonomous operation in teleop
    public AtomicBoolean manualInterrupt = new AtomicBoolean(true); // used to interrupt autonomous operations with manual driving control

    public DcMotorEx fl = null;
    public DcMotorEx fr = null;
    public DcMotorEx bl = null;
    public DcMotorEx br = null;
    public DcMotorEx strafeEncoder;

    public DcMotorEx forwardEncoder;
    public AnalogInput ultLeft = null;
    public DigitalChannel prxLeft = null, prxRight = null;

    // Image Processors and Cameras
    public VisionPortal rearVisionPortal, sideVisionPortal, frontVisionPortal;

    public AprilTagProcessor aprilTag;
    public static int aprilTagExposure = 3; // frame exposure in ms (use TestDrive opMode to calibrate)
    public static int aprilTagGain = 255; // High gain to compensate for fast exposure  DOESN'T WORK DUE TO FTC BUG
    public OpenCVFindLine findLineProcesser;
    public OpenCVYellowPixelDetector findPixelProcesser;

    public OpenCVFindWhitePixel findWhitePixelProcessor;

    public OpenCVPropFinder findTeamPropProcesser;



    public double noAprilTag = 999.0;
    public float TAG_CENTER_TO_CENTER = 15.2f;

    public enum cvCam {NONE, REAR_APRILTAG, REAR_YELLOW_APRILTAG, SIDE_PROP, FRONT_LINE, PIXEL_STACK};

    public Drive.cvCam currentCam = Drive.cvCam.NONE;




    public WebcamName frontCam;
    public WebcamName backCam;
    public WebcamName sideCam;

    public double COUNTS_PER_MOTOR_REV = 537.7;    // GoBilda 5202 312 RPM
    public double COUNTS_PER_CENTIMETER = 12.855; // Used with 435 RPM
    public double COUNTS_PER_CENTIMETER_312 = 17.923; // Used with 312 RPM
    public double TICS_PER_CM_STRAFE_ENCODER = 130;
    public double TICS_PER_CM_STRAIGHT_ENCODER = 735;

    public double TILE_CENTER_TO_CENTER = 60.325; // tile width in Cms

    public double MIN_START_VELOCITY = 300; //calibrated with 435s
    public double MIN_END_VELOCITY = 400; //calibrated with 435s
    public double MAX_ACCELERATION = 22; //calibrated with 435s
    public double MAX_DECELERATION = 1.0; //calibrated with 435s
    public double MAX_STRAIGHT_ACCELERATION = 1.0; //calibrated with 435s
    public double MAX_STRAIGHT_DECELERATION = 0.02; //calibrated with 435s
    public double MIN_STRAFE_START_VELOCITY = 800; //calibrated with 435s
    public double MIN_STRAFE_END_VELOCITY = 400; //calibrated with 435s
    public double MAX_STRAFE_DECELERATION = .15; //calibrated with 435s
    public double MAX_STRAFE_ACCELERATION = 2; // tentaive
    public double MAX_VELOCITY = 2450; // Was 2680
    public double MAX_VELOCITY_STRAFE = 2000; // Added with new motors
    public double ROTATION_ADJUST_FACTOR = 0.04;
    public double SIDE_VECTOR_COEFFICIENT = .92;
    public double FORWARD_VECTOR_COEFFICIENT = 1.08;
    public double SPIN_END_OF_MAX_VELOCITY = 60;
    public double DRIFT_SPINS = 1;
    public double CRAWL_SPEED = 200;
    public double CRAWL_DISTANCE_SPINS = 30;
    public boolean details = true;
    public double CMS_PER_INCH = 2.54;

    public Output output;

    public Drive() {
        teamUtil.log("Constructing Drive");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize() { // Output theOutput
        teamUtil.log("Initializing Drive");
        //Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //output=theOutput;
        fl = hardwareMap.get(DcMotorEx.class, "flm");
        fr = hardwareMap.get(DcMotorEx.class, "frm");
        bl = hardwareMap.get(DcMotorEx.class, "blm");
        br = hardwareMap.get(DcMotorEx.class, "brm");
//        strafeEncoder = hardwareMap.get(DcMotorEx.class, "liftAndStrafeEncoder");
//        forwardEncoder = hardwareMap.get(DcMotorEx.class, "sweeperAndForwardEncoder");
//        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ultLeft = hardwareMap.analogInput.get("ult");
//        prxLeft = hardwareMap.get(DigitalChannel.class, "prx_left");
//        prxRight = hardwareMap.get(DigitalChannel.class, "prx_right");
//        prxLeft.setMode(DigitalChannel.Mode.INPUT);
//        prxRight.setMode(DigitalChannel.Mode.INPUT);

        // colorSensor.calibrate();
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //tapeSensor1 = new bottomColorSensor(hardwareMap.get(RevColorSensorV3.class, "bottomColor1"));
        //tapeSensor2 = new bottomColorSensor(hardwareMap.get(RevColorSensorV3.class, "bottomColor2"));
        //tapeSensor1.calibrate();
        //tapeSensor2.calibrate();

        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        setMotorsBrake();
        teamUtil.log("Initializing Drive - FINISHED");
    }

    //////////////////////////
    public void initCV(boolean enableLiveView) { //should be false for comp code
        teamUtil.log("Initializing CV: LiveView: "+enableLiveView);

        // Setup a separate VisionPortal for each camera.  We will switch between cameras and processers as needed
        aprilTag = new AprilTagProcessor.Builder().build();
        findPixelProcesser = new OpenCVYellowPixelDetector();
        findLineProcesser = new OpenCVFindLine();
        findTeamPropProcesser = new OpenCVPropFinder();
        findWhitePixelProcessor = new OpenCVFindWhitePixel();

        backCam = hardwareMap.get(WebcamName.class, "Webcam Rear");
        frontCam = hardwareMap.get(WebcamName.class, "Webcam Front");
        if (teamUtil.alliance == RED) {
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
        rearBuilder.addProcessor(findPixelProcesser);
        // Can also set resolution and stream format if we want to optimize resource usage.
        rearVisionPortal = rearBuilder.build();
        stopStreaming(rearVisionPortal);
        rearVisionPortal.setProcessorEnabled(findPixelProcesser,false);

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

        frontBuilder.addProcessor(findWhitePixelProcessor);
        frontBuilder.addProcessor(findLineProcesser);

        // Can also set resolution and stream format if we want to optimize resource usage.
        frontVisionPortal = frontBuilder.build();
        stopStreaming(frontVisionPortal);

        // set up cam settings
        teamUtil.log("Updating Exposure Settings");
        switchCV(cvCam.REAR_APRILTAG);
        updateCVManualExposure(rearVisionPortal,aprilTagExposure,aprilTagGain);
        switchCV(cvCam.REAR_YELLOW_APRILTAG);
        switchCV(cvCam.FRONT_LINE);
        updateCVManualExposure(frontVisionPortal, findLineProcesser.lineExposure, findLineProcesser.lineGain);
        switchCV(cvCam.NONE);
        currentCam = Drive.cvCam.NONE;

        teamUtil.log("Initializing Drive CV - FINISHED");
    }






    private boolean updateCVManualExposure(VisionPortal portal, int exposureMS, int gain) {
        // update cam exposure and gain
        teamUtil.log("Setting Manual Exposure");
        // Ensure Vision Portal has been setup.
        if (portal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!teamUtil.theOpMode.isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                teamUtil.pause(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        // Set camera controls unless we are stopping the op mode
        if (!teamUtil.theOpMode.isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                teamUtil.log("Switching to Manual Mode");
                exposureControl.setMode(ExposureControl.Mode.Manual);
                teamUtil.pause(50);
            }
            teamUtil.log("Setting Exposure");

            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            teamUtil.pause(20);

            // Set Gain.
            GainControl gainControl = portal.getCameraControl(GainControl.class);
            if (gainControl == null) { // FTC software has a bug where this isn't supported on switchableCameras!
            } else {
                teamUtil.log("Setting Gain");
                gainControl.setGain(gain);
            }
            teamUtil.pause(20);
            teamUtil.log("Setting Manual - Finished");
            return (true);
        } else {
            teamUtil.log("Setting Manual - Finished");
            return (false);
        }
    }

    public void setAutoExposure(VisionPortal portal) {
        // set exposure for auto if it needs to be different
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
        boolean details = false;
        teamUtil.log("Stop Streaming ");

        while (!teamUtil.theOpMode.isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            if (details) {teamUtil.log("Waiting for Camera to finish coming on line: " + portal.getCameraState());}
            teamUtil.pause(100);
        }
        portal.stopStreaming();
        while (!teamUtil.theOpMode.isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY)) {
            if (details) {teamUtil.log("Waiting for Camera to stop: " + portal.getCameraState());}
            teamUtil.pause(100);
        }
    }

    public void stopCV() {
        // stop processors and cams
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
            sideVisionPortal.stopStreaming();
        }

    }
    public void switchCV(Drive.cvCam newCam) {
        // stop old cam processer and start a new one
        teamUtil.log("Switching CV to " + newCam);
        teamUtil.log("Stopping " + currentCam);
        switch (currentCam) {
            case REAR_APRILTAG:
                rearVisionPortal.stopStreaming();
                break;
            case REAR_YELLOW_APRILTAG:
                rearVisionPortal.stopStreaming();
                break;
            case FRONT_LINE:
                teamUtil.log("Switching CV to " + newCam);
                frontVisionPortal.stopStreaming();
                break;
            case SIDE_PROP:
                teamUtil.log("Switching CV to " + newCam);
                sideVisionPortal.stopStreaming();
                break;
            case PIXEL_STACK:
                teamUtil.log("Switching CV to " + newCam);
                frontVisionPortal.stopStreaming();
                break;
        }
        
        teamUtil.log("Starting " + newCam);

        switch (newCam) {
            case REAR_APRILTAG:
                rearVisionPortal.resumeStreaming();
                rearVisionPortal.setProcessorEnabled(findPixelProcesser,false);
                break;

            case REAR_YELLOW_APRILTAG:
                rearVisionPortal.resumeStreaming();
                rearVisionPortal.setProcessorEnabled(findPixelProcesser,true);
                break;

            case FRONT_LINE:
                frontVisionPortal.resumeStreaming();
                frontVisionPortal.setProcessorEnabled(findWhitePixelProcessor, false);
                frontVisionPortal.setProcessorEnabled(findLineProcesser, true);
                break;

            case SIDE_PROP:
                sideVisionPortal.resumeStreaming();
                break;

            case PIXEL_STACK:
                frontVisionPortal.resumeStreaming();
                frontVisionPortal.setProcessorEnabled(findWhitePixelProcessor, true);
                frontVisionPortal.setProcessorEnabled(findLineProcesser, false);
                break;
        }
        currentCam = newCam;
        teamUtil.log("switchCV - Finished ");
    }

//////////////////////



    public double getUltrasonicDistance() {
        // use formula to find ult distance
        double voltage = ultLeft.getVoltage();
        double distance = (260 / 3 * (voltage - .55) + 36) * 2.54; // based on real world distances measured
        return distance;
    }

    public boolean getProximity(boolean left) {
        // if the proximity is triggered
        if (!left) {
            return !prxRight.getState();
        } else {
            return !prxLeft.getState();
        }
    }

    public boolean getLeftProximity() {
        return !prxLeft.getState();
    }

    public boolean getRightProximity() {
        return !prxRight.getState();
    }

    public void backgroundProximityCheck(){
        // contantly continue moving until both left and right prx have been triggered
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(!getRightProximity()&&!getLeftProximity()){
                }
                teamUtil.log("Object detected within proximity distance");
                stopMotors();
                teamUtil.log("Motors Stopped because of Proximity");
            }
        });
        thread.start();

    }

    public void driveMotorTelemetry() {
        telemetry.addData("Drive ", "flm:%d frm:%d blm:%d brm:%d strafe:%d forward:%d heading:%f ",
                fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition(), strafeEncoder.getCurrentPosition(), forwardEncoder.getCurrentPosition(),getHeading());
    }

    public void sensorTelemetry() {
        telemetry.addData("UltrasonicLeft Distance: ", "%.1f", getUltrasonicDistance());
        telemetry.addLine("Right proximity sensor: " + getProximity(false));
        telemetry.addLine("Left proximity sensor: " + getProximity(true));

        //telemetry.addData("On Line:", " %b/%b", tapeSensor1.isOnTape(), tapeSensor2.isOnTape());
        //telemetry.addData("Red Value: ", "%d/%d", tapeSensor1.redValue(), tapeSensor2.redValue());
        //telemetry.addData("Blue Value: ", "%d/%d", tapeSensor1.blueValue(), tapeSensor2.blueValue());
    }

    public void visionTelemetry() {
        // states and info about the cams and processors
        if (currentCam==cvCam.REAR_APRILTAG) {
            //findPixelProcesser.outputTelemetry();
            //telemetry.addLine("BackDrop Offset: " + getRobotBackdropXOffset());
            telemetry.addLine("RearCam:" + rearVisionPortal.getCameraState() + " FPS:" + rearVisionPortal.getFps());
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addLine("AprilTag: " + detection.id + " X:" + detection.ftcPose.x * CMS_PER_INCH + " Y:" + detection.ftcPose.y * CMS_PER_INCH);
            }
        }else if(currentCam==cvCam.REAR_YELLOW_APRILTAG){
            telemetry.addLine("FrontCam:" + frontVisionPortal.getCameraState() + " FPS:" + frontVisionPortal.getFps());
            findPixelProcesser.outputTelemetry();
        }
        else if (currentCam==cvCam.FRONT_LINE) {
            telemetry.addLine("FrontCam:" + frontVisionPortal.getCameraState() + " FPS:" + frontVisionPortal.getFps());
            findLineProcesser.outputTelemetry();
        } else if (currentCam==cvCam.SIDE_PROP) {
            telemetry.addLine("SideCam:" + sideVisionPortal.getCameraState() + " FPS:" + sideVisionPortal.getFps());
            findTeamPropProcesser.outputTelemetry();
        } else if (currentCam==cvCam.PIXEL_STACK) {
            telemetry.addLine("FrontCam:" + frontVisionPortal.getCameraState() + " FPS:" + frontVisionPortal.getFps());
            findWhitePixelProcessor.outputTelemetry();
        }
    }

    public void runMotors(double velocity) {
        lastVelocity = velocity;
        fl.setVelocity(velocity);
        fr.setVelocity(velocity);
        bl.setVelocity(velocity);
        br.setVelocity(velocity);
    }

    public void setMotorsBrake() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorsFloat() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    public void setMotorPower(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

    }

    public void setMotorsActiveBrake() {
        // hold a position using setPosition
        int flPosition = fl.getCurrentPosition();
        int frPosition = fr.getCurrentPosition();
        int blPosition = bl.getCurrentPosition();
        int brPosition = br.getCurrentPosition();
        fl.setTargetPosition(flPosition);
        fr.setTargetPosition(frPosition);
        bl.setTargetPosition(blPosition);
        br.setTargetPosition(brPosition);

        setMotorsRunToPosition();
        setMotorPower(0.5);
    }

    public void stopMotors() {
        teamUtil.log("Stopping Motors");
        lastVelocity = 0;
        fl.setVelocity(0);
        fr.setVelocity(0);
        bl.setVelocity(0);
        br.setVelocity(0);
    }

    public void setMotorVelocities(double flV, double frV, double blV, double brV) {
        fl.setVelocity(flV);
        fr.setVelocity(frV);
        bl.setVelocity(blV);
        br.setVelocity(brV);
    }

    public void setMotorsRunToPosition() {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void setMotorsRunWithoutEncoder() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void resetAllDriveEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMotorsWithEncoder() {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void findMaxVelocity(int cmDistance) {
        // set motors to 3000 (theoretical max) then see how far it actually has traveled
        long startTime = System.currentTimeMillis();
        teamUtil.log("Finding Forward Max Velocities...");
        resetAllDriveEncoders();
        double travelTics = COUNTS_PER_CENTIMETER * cmDistance;
        setMotorVelocities(3000, 3000, 3000, 3000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0, v;
        double ticStartPosition = fr.getCurrentPosition();
        while (fr.getCurrentPosition() < travelTics) {
            flmax = (v = fl.getVelocity()) > flmax ? v : flmax;
            frmax = (v = fr.getVelocity()) > frmax ? v : frmax;
            blmax = (v = bl.getVelocity()) > blmax ? v : blmax;
            brmax = (v = br.getVelocity()) > brmax ? v : brmax;
            teamUtil.log("Looping FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);
        }
        stopMotors();
        long elapsedTime = System.currentTimeMillis()-startTime;
        double ticsTraveled = fr.getCurrentPosition()-ticStartPosition;
        double cmsTraveled = ticsTraveled/COUNTS_PER_CENTIMETER;
        double timeS = elapsedTime/1000.0;
        teamUtil.log("Elapsed Time: " + elapsedTime);
        teamUtil.log("Tics Traveled: " + ticsTraveled);
        teamUtil.log("Cms Traveled: " + cmsTraveled);
        teamUtil.log("Cms Per Second: " + cmsTraveled/timeS);



        teamUtil.log("Forward Max Velocities FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);
    }

    public void findMaxStrafeVelocity(double distance){
        // same as above
        setHeading(180);
        long startTime = System.currentTimeMillis();
        teamUtil.log("Finding Strafing Max Velocities...");
        resetAllDriveEncoders();
        double travelTics = distance*COUNTS_PER_CENTIMETER;
        teamUtil.log("Travel Tics: " + travelTics);
        double ticStartPosition = fr.getCurrentPosition();
        driveMotorsHeadingsFR(270,180,3000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0, v;

        while(fr.getCurrentPosition()<travelTics){

            flmax = (v = fl.getVelocity()) > flmax ? v : flmax;
            frmax = (v = fr.getVelocity()) > frmax ? v : frmax;
            blmax = (v = bl.getVelocity()) > blmax ? v : blmax;
            brmax = (v = br.getVelocity()) > brmax ? v : brmax;
            teamUtil.log("Looping FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);


        }
        stopMotors();
        long elapsedTime = System.currentTimeMillis()-startTime;
        double ticsTraveled = fr.getCurrentPosition()-ticStartPosition;
        double cmsTraveled = ticsTraveled/COUNTS_PER_CENTIMETER;
        double timeS = elapsedTime/1000.0;
        teamUtil.log("Elapsed Time: " + elapsedTime);
        teamUtil.log("Tics Traveled: " + ticsTraveled);

        teamUtil.log("Cms Per Second: " + cmsTraveled/timeS);
        teamUtil.log("Cms Traveled: " + cmsTraveled);

        teamUtil.log("Tics Per Second: " + ticsTraveled/(elapsedTime/1000));

        teamUtil.log("Strafing Max Velocities FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);
    }

    public void driveMotorsHeadings(double driveHeading, double robotHeading, double velocity) {
        // move robot based on a heading to face and a heading to drive to
        double flV, frV, blV, brV;
        double x, y, scale;

        // Determine how much adjustment for rotational drift
        double headingError = getHeadingError(robotHeading); // Difference between desired and actual robot heading
        //double headingError = Math.max(-45.0, Math.min(getHeadingError(robotHeading), 45.0)); // clip this to 45 degrees in either direction to control rate of spin
        double rotationAdjust = ROTATION_ADJUST_FACTOR * velocity * headingError; // scale based on velocity AND amount of rotational error

        // Covert heading to cartesian on the unit circle and scale so largest value is 1
        // This is essentially creating joystick values from the heading
        // driveHeading is relative to robot at this point since the wheels are relative to robot!
        x = Math.cos(Math.toRadians(driveHeading + 90)); // + 90 cause forward is 0...
        y = Math.sin(Math.toRadians(driveHeading + 90));
        scale = 1 / Math.max(Math.abs(x), Math.abs(y));
        x = x * scale;
        y = y * scale;

        // Clip to motor power range
        flV = Math.max(-1.0, Math.min(x + y, 1.0)) * velocity;
        brV = flV;
        frV = Math.max(-1.0, Math.min(y - x, 1.0)) * velocity;
        blV = frV;

        // Adjust for rotational drift
        flV = flV - rotationAdjust;
        brV = brV + rotationAdjust;
        frV = frV + rotationAdjust;
        blV = blV - rotationAdjust;

        // Update the motors
        setMotorVelocities(flV, frV, blV, brV);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set the velocity of all 4 motors based on a driveHeading RELATIVE TO FIELD and provided velocity
    // Will rotate robot as needed to achieve and hold robotHeading RELATIVE TO FIELD by moving to a set target
    public void driveMotorsHeadingsFR(double driveHeading, double robotHeading, double velocity) {
        double RRDriveHeading = getHeadingError(driveHeading);
        driveMotorsHeadings(RRDriveHeading, robotHeading, velocity);
    }

    public double getHeadingError(double targetAngle) {
        // distance from target
        double robotError;

        // calculate heading error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getRawHeading() {
        Orientation anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (anglesCurrent.firstAngle);
    }

    public double getHeading() {
        // stows an offset to change the range and set the heading
        return adjustAngle(getRawHeading() - HEADING_OFFSET);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Make the current heading 0.
    public void resetHeading() {
        HEADING_OFFSET = getRawHeading();
        heldHeading = getHeading();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Make the current heading to specified number
    public void setHeading(int heading) {
        HEADING_OFFSET = getRawHeading() - heading;
        heldHeading = getHeading();
    }

    public void logMotorPositions() {
        teamUtil.log("fr: " + fr.getCurrentPosition());
        teamUtil.log("fl: " + fl.getCurrentPosition());
        teamUtil.log("br: " + br.getCurrentPosition());
        teamUtil.log("bl: " + bl.getCurrentPosition());
    }

    public double adjustAngle(double angle) {
        //assuming imu runs from [0, 360] and angle is added/substracted, adjust it to expected reading
        while (angle >= 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    class MotorData { // a helper class to allow for faster access to hub data
        int eFL, eFR, eBL, eBR; // encoder values of each motor
    }

    public void getDriveMotorData(MotorData data) {
        // update current motor positions
        data.eFL = fl.getCurrentPosition();
        data.eFR = fr.getCurrentPosition();
        data.eBL = bl.getCurrentPosition();
        data.eBR = br.getCurrentPosition();
    }

    /************************************************************************************************************
     /************************************************************************************************************
     /************************************************************************************************************
     *
     * Methods to drive based on encoders
     *
     /************************************************************************************************************
     /************************************************************************************************************/
    public int getEncoderDistance(MotorData initialPositions) {
        // use trig to find the hypotenuse of the side and front distances
        MotorData currentPositions = new MotorData();
        getDriveMotorData(currentPositions);

        // Calculate the vector along the forward/backward axis
        int ForwardVector = (currentPositions.eFL - initialPositions.eFL)
                + (currentPositions.eFR - initialPositions.eFR)
                + (currentPositions.eBL - initialPositions.eBL)
                + (currentPositions.eBR - initialPositions.eBR);
        // Calculate the vector along the left/right axis
        int SideVector = (currentPositions.eFL - initialPositions.eFL)
                + (currentPositions.eBR - initialPositions.eBR)
                - (currentPositions.eFR - initialPositions.eFR)
                - (currentPositions.eBL - initialPositions.eBL);

        // Return the hypotenuse of the two vectors
        // divide by 4 to account for the math that adds all 4 motor encoders
        return (int) (Math.sqrt(Math.pow((ForwardVector * FORWARD_VECTOR_COEFFICIENT), 2) + (Math.pow((SideVector * SIDE_VECTOR_COEFFICIENT), 2))) / 4);

    }
    
    public void moveCm(double centimeters, double driveHeading) {
        // simplified parameters of moveCm
        moveCm(MAX_VELOCITY, centimeters, driveHeading, getHeading(), MIN_END_VELOCITY);
    }

    public void moveCm(double centimeters, double driveHeading, double endVelocity) {
        // simplified parameters of moveCm
        moveCm(MAX_VELOCITY, centimeters, driveHeading, getHeading(), endVelocity);
    }

    public void moveCm(double maxVelocity, double centimeters, double driveHeading, double endVelocity) {
        // simplified parameters of moveCm
        moveCm(maxVelocity, centimeters, driveHeading, getHeading(), endVelocity);
    }

    public void moveCm(double maxVelocity, double centimeters, double driveHeading, double robotHeading, double endVelocity) {
        // move based on a cruise, end, and max velocity, distance, and headings
        teamUtil.log("MoveCM cms:" + centimeters + " driveH:" + driveHeading + " robotH:" + robotHeading + " MaxV:" + maxVelocity + " EndV:" + endVelocity);

        details = false;
        MotorData data = new MotorData();
        getDriveMotorData(data);

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY; // simplify by setting min end to 0
        }
        // tics^2/s
        if (lastVelocity == 0) { // at a stop
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else { // already moving
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        setMotorsWithEncoder();
        // all are measured in tics
        double totalTics = centimeters * COUNTS_PER_CENTIMETER;
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_DECELERATION);
        double postCruiseTargetDistance = totalTics - decelerationDistance;
        if (postCruiseTargetDistance < 0) { // need to cut off the curve
            double percentageToRemoveAccel = accelerationDistance / (accelerationDistance + decelerationDistance);
            accelerationDistance += postCruiseTargetDistance * percentageToRemoveAccel;
            decelerationDistance += postCruiseTargetDistance * percentageToRemoveAccel;
            postCruiseTargetDistance = 0;
        }
        double distance = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
            teamUtil.log("Post Cruise distance: " + postCruiseTargetDistance);
        }
//acceleration
        while (distance < accelerationDistance) {
            distance = getEncoderDistance(data);
            if (lastVelocity == 0) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_ACCELERATION * distance + MIN_START_VELOCITY); // velocity moves by distance
            } else {
                driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_ACCELERATION * distance + lastVelocity);
            }
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distance);
        }
//cruise
        while (distance < postCruiseTargetDistance) {

            distance = getEncoderDistance(data);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity); // constant
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distance);
        }


//deceleration
        double startDecelerationDistance = distance;
        double ticsUntilEnd = totalTics - distance;
        while (distance < totalTics) {
            distance = getEncoderDistance(data);
            ticsUntilEnd = totalTics - distance;
            driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_DECELERATION * ticsUntilEnd + endVelocity); // lowers through tics to end

        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distance);
        }
        if (endVelocity <= MIN_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was min end velocity");
            }
        }
        lastVelocity = endVelocity;
        teamUtil.log("MoveCM--Finished");

    }



    public void strafeToTarget(double maxVelocity, double strafeTarget, double driveHeading, double robotHeading, double endVelocity, long timeout) {
        teamUtil.log("strafeToTarget target: " + strafeTarget + " driveH: " + driveHeading + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = false;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;

        if (details) teamUtil.log("Starting Strafe Encoder: "+ strafeEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_STRAFE_END_VELOCITY) {
            endVelocity = MIN_STRAFE_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) { // at stop
            velocityChangeNeededAccel = maxVelocity - MIN_STRAFE_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else { // moving
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        // all are measured in tics
        double startEncoder = strafeEncoder.getCurrentPosition();
        if ((driveHeading< 180 && strafeTarget-startEncoder <=0) || (driveHeading > 180 && startEncoder-strafeTarget <=0))
        {
            teamUtil.log("ALREADY PAST TARGET--Not Strafing");
            stopMotors();
            return;
        }
        double totalTics = Math.abs(startEncoder-strafeTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAFE_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAFE_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;

        setBulkReadAuto();



//acceleration
        double currentVelocity = 0;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + MIN_STRAFE_START_VELOCITY; // increases velocity
            } else {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + lastVelocity;
            }
            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);

        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return;


        }
//cruise
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity); // constant
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return;


        }


//deceleration
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            currentVelocity = MAX_STRAFE_DECELERATION * distanceRemaining + endVelocity;
            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining); 
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);// decreases
        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (endVelocity <= MIN_STRAFE_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was min end velocity");
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("strafeToTarget--Finished.  Current Strafe Encoder:" + strafeEncoder.getCurrentPosition());

    }

    public boolean strafeToTargetWithProximity(double maxVelocity, double strafeTarget, double driveHeading, double robotHeading, double endVelocity, long timeout) {
        // same as above but aborts if the prx is triggered
        teamUtil.log("strafeToTarget target: " + strafeTarget + " driveH: " + driveHeading + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = false;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;

        if (details) teamUtil.log("Starting Strafe Encoder: "+ strafeEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_STRAFE_END_VELOCITY) {
            endVelocity = MIN_STRAFE_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_STRAFE_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        // all are measured in tics
        double startEncoder = strafeEncoder.getCurrentPosition();
        if ((driveHeading< 180 && strafeTarget-startEncoder <=0) || (driveHeading > 180 && startEncoder-strafeTarget <=0))
        {
            teamUtil.log("ALREADY PAST TARGET--Not Strafing");
            stopMotors();
            return true;
        }
        double totalTics = Math.abs(startEncoder-strafeTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAFE_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAFE_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;

        setBulkReadAuto();



//acceleration
        double currentVelocity = 0;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + MIN_STRAFE_START_VELOCITY;
            } else {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + lastVelocity;
            }
            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            if(teamUtil.alliance == RED){
                if(getLeftProximity()){
                    stopMotors();
                    teamUtil.pause(500);
                    teamUtil.log("Proximity Sensors Triggered in Acceleration Phase");
                    return false;
                }
            }else{
                if(getRightProximity()){
                    stopMotors();
                    teamUtil.pause(500);
                    teamUtil.log("Proximity Sensors Triggered in Acceleration Phase");
                    return false;
                }
            }

            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);

        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            teamUtil.log("Tics Left + " + distanceRemaining);

            stopMotors();
            return false;


        }
//cruise
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity);
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(teamUtil.alliance == RED){
            if(getLeftProximity()){
                stopMotors();
                teamUtil.pause(500);
                teamUtil.log("Proximity Sensors Triggered in Cruise Phase");
                return false;
            }
        }else{
            if(getRightProximity()){
                stopMotors();
                teamUtil.pause(500);
                teamUtil.log("Proximity Sensors Triggered in Cruise Phase");
                return false;
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return false;


        }


//deceleration
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            currentVelocity = MAX_STRAFE_DECELERATION * distanceRemaining + endVelocity;
            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);
        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (endVelocity <= MIN_STRAFE_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was min end velocity");
            }
        }
        if(teamUtil.alliance == RED){
            if(getLeftProximity()){
                stopMotors();
                teamUtil.pause(500);
                teamUtil.log("Proximity Sensors Triggered in Deceleration Phase");
                return false;
            }
        }else{
            if(getRightProximity()){
                stopMotors();
                teamUtil.pause(500);
                teamUtil.log("Proximity Sensors Triggered in Deceleration Phase");
                return false;
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("strafeToTarget--Finished.  Current Strafe Encoder:" + strafeEncoder.getCurrentPosition());
        return true;

    }

    public void driveStraightToTarget(double maxVelocity, double forwardTarget, double driveHeading, double robotHeading, double endVelocity, long timeout) {
        // same as movecm but reads distance from dead wheels
        teamUtil.log("driveToTarget target: " + forwardTarget + " driveH: " + driveHeading + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = false;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;



        if (details) teamUtil.log("Starting Forward Encoder: "+ forwardEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        // all are measured in tics
        double startEncoder = forwardEncoder.getCurrentPosition();
        if (((driveHeading< 90 || driveHeading>270)&&forwardTarget-startEncoder >=0) || ((driveHeading> 90 && driveHeading<270) && startEncoder-forwardTarget >=0)){

            teamUtil.log("ALREADY PAST TARGET--Not Moving");
            stopMotors();
            return;
        }
        double totalTics = Math.abs(startEncoder-forwardTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAIGHT_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAIGHT_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();

        setBulkReadAuto();


//acceleration
        double currentVelocity = 0;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (totalTics-distanceRemaining) + MIN_START_VELOCITY;
            } else {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (totalTics-distanceRemaining) + lastVelocity;
            }
            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);

        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return;


        }
//cruise
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity);
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return;


        }


//deceleration
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();
            currentVelocity = MAX_STRAIGHT_DECELERATION * distanceRemaining + endVelocity;
            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);
        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (endVelocity <= MIN_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was min end velocity");
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("driveToTarget--Finished.  Current Forward Encoder:" + forwardEncoder.getCurrentPosition());

    }

    public boolean driveStraightToTargetWithStrafeEncoderValue(double maxVelocity, double forwardTarget, double strafeTarget, double driveHeading, double robotHeading, double endVelocity, long timeout) {
        // same as above but also includes the strafe encoder and the forwards encoder
        teamUtil.log("driveStraightToTargetWithStrafeEncoderValue target: " + forwardTarget + " driveH: " + driveHeading + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = false;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;

        if (details) teamUtil.log("Starting Forward Encoder: "+ forwardEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;

        float strafeFactor = .09f; // convert strafe encoder error into heading declination
        float maxHeadingDeclination = 27f; // don't veer off of straight more than this number of degrees
        float headingFactor = Math.abs(driveHeading-180)<.01 ? 1 : -1; // reverse correction for going backwards

        double requestedEndVelocity = endVelocity;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        // all are measured in tics
        double startEncoder = forwardEncoder.getCurrentPosition();
        if (((driveHeading< 90 || driveHeading>270)&&forwardTarget-startEncoder >=0) || ((driveHeading> 90 && driveHeading<270) && startEncoder-forwardTarget >=0)){

            teamUtil.log("ALREADY PAST TARGET--Not Strafing");
            stopMotors();
            return false;
        }
        double totalTics = Math.abs(startEncoder-forwardTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAIGHT_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAIGHT_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();

        setBulkReadAuto();

        int currentPos;
//acceleration
        double currentVelocity;
        double adjustedDriveHeading;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? currentPos-forwardTarget : forwardTarget - currentPos;
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + MIN_START_VELOCITY;
            } else {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + lastVelocity;
            }
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);

        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return false;


        }
//cruise
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? currentPos-forwardTarget : forwardTarget - currentPos;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);

            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, maxVelocity);
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return false;


        }


//deceleration
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? currentPos-forwardTarget : forwardTarget - currentPos;
            currentVelocity = MAX_STRAIGHT_DECELERATION * distanceRemaining + endVelocity;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (requestedEndVelocity < 1) {
            stopMotors();
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("driveStraightToTargetWithStrafeEncoderValue--Finished.  Current Forward Encoder:" + forwardEncoder.getCurrentPosition());
        return true;
    }
    //potential
    public void strafeToTargetWithForwardEncoderValue(double maxVelocity, double forwardTarget, double strafeTarget, double driveHeading, double robotHeading, double endVelocity, long timeout) {
    // same but strafes 
        teamUtil.log("driveStraightToTargetWithStrafeEncoderValue target: " + forwardTarget + " driveH: " + driveHeading + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = false;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;

        if (details) teamUtil.log("Starting Forward Encoder: "+ forwardEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;

        float strafeFactor = .09f; // convert strafe encoder error into heading declination
        float maxHeadingDeclination = 27f; // don't veer off of straight more than this number of degrees
        float headingFactor = Math.abs(driveHeading-180)<.01 ? 1 : -1; // reverse correction for going backwards

        double requestedEndVelocity = endVelocity;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        // all are measured in tics
        double startEncoder = forwardEncoder.getCurrentPosition();
        if (((driveHeading< 90 || driveHeading>270)&&forwardTarget-startEncoder >=0) || ((driveHeading> 90 && driveHeading<270) && startEncoder-forwardTarget >=0)){

            teamUtil.log("ALREADY PAST TARGET--Not Strafing");
            stopMotors();
            return;
        }
        double totalTics = Math.abs(startEncoder-forwardTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAIGHT_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAIGHT_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();

        setBulkReadAuto();

        int currentPos;
//acceleration
        double currentVelocity;
        double adjustedDriveHeading;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? currentPos-forwardTarget : forwardTarget - currentPos;
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + MIN_START_VELOCITY;
            } else {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + lastVelocity;
            }
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);

        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return;


        }
//cruise
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? currentPos-forwardTarget : forwardTarget - currentPos;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);

            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, maxVelocity);
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return;


        }


//deceleration
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? currentPos-forwardTarget : forwardTarget - currentPos;
            currentVelocity = MAX_STRAIGHT_DECELERATION * distanceRemaining + endVelocity;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (requestedEndVelocity < 1) {
            stopMotors();
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("driveStraightToTargetWithStrafeEncoderValue--Finished.  Current Forward Encoder:" + forwardEncoder.getCurrentPosition());

    }

    public void driveStraightToTargetWithStrafeEncoderAndGoToScore(double maxVelocity, double forwardTarget, double strafeTarget, double driveHeading, double robotHeading, double endVelocity, double goToScoreTarget, double rotatorPos, double straferPos, float level, long timeout, boolean operateArms) {
        teamUtil.log("driveStraightToTargetWithStrafeEncoderValueAndGoToScore target: " + forwardTarget + " driveH: " + driveHeading + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = false;
        boolean wentToScore = false;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;

        if (details) teamUtil.log("Starting Forward Encoder: "+ forwardEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;

        float strafeFactor = .09f; // convert strafe encoder error into heading declination
        float maxHeadingDeclination = 27f; // don't veer off of straight more than this number of degrees
        float headingFactor = Math.abs(driveHeading-180)<.01 ? 1 : -1; // reverse correction for going backwards

        double requestedEndVelocity = endVelocity;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        // all are measured in tics
        double startEncoder = forwardEncoder.getCurrentPosition();
        if (((driveHeading< 90 || driveHeading>270)&&forwardTarget-startEncoder >=0) || ((driveHeading> 90 && driveHeading<270) && startEncoder-forwardTarget >=0)){

            teamUtil.log("ALREADY PAST TARGET--Not Strafing");
            stopMotors();
            return;
        }
        double totalTics = Math.abs(startEncoder-forwardTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAIGHT_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAIGHT_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();

        setBulkReadAuto();

        int currentPos;
//acceleration
        double currentVelocity;
        double adjustedDriveHeading;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? currentPos-forwardTarget : forwardTarget - currentPos;
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + MIN_START_VELOCITY;
            } else {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + lastVelocity;
            }
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);

        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return;


        }
//cruise
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? currentPos-forwardTarget : forwardTarget - currentPos;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);
            if(operateArms){
                if (!wentToScore && goToScoreTarget < -1 && forwardEncoder.getCurrentPosition()<goToScoreTarget) {
                    wentToScore = true;
                    output.goToScoreNoWait(level, rotatorPos, straferPos);
                }
            }


            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, maxVelocity);
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return;


        }


//deceleration
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? currentPos-forwardTarget : forwardTarget - currentPos;
            currentVelocity = MAX_STRAIGHT_DECELERATION * distanceRemaining + endVelocity;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }
            if(operateArms){
                if (!wentToScore && goToScoreTarget < -1 && forwardEncoder.getCurrentPosition()<goToScoreTarget) {
                    wentToScore = true;
                    output.goToScoreNoWait(level, rotatorPos, straferPos);
                }
            }

            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (requestedEndVelocity < 1) {
            stopMotors();
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("driveStraightToTargetWithStrafeEncoderValueAndGoToScore--Finished.  Current Forward Encoder:" + forwardEncoder.getCurrentPosition());

    }

    public void backToPoint(double robotHeading, double x, double y, double endVelocity) { // assumes robot heading is 180
        // x positive means to the robots right
        // y positive means robot move backwards (not tested for anything else!)
        double heading, distance;
        teamUtil.log("Move to Point: x/y " + x + "/"+ y);
        distance = Math.sqrt(x*x+y*y);
        if (y == 0) {
            heading = x < 0 ? 270 : 90;
        } else if (y > 0) { // Using vertical (y-axis) to compute reference angles since 0 is at top
            heading = adjustAngle(Math.toDegrees(Math.atan(x / y)));
        } else {
            heading = 180 + Math.toDegrees(Math.atan(x / y));
        }
        moveCm(MAX_VELOCITY,distance,heading,180,endVelocity);
    }
    public void moveStraightCmWithStrafeEncoder(double maxVelocity, double centimeters, int strafeTarget, double driveHeading, double robotHeading, double endVelocity) {
        // movecm with strafe encoder for side vector
        teamUtil.log("Strafe Target" + strafeTarget);
        teamUtil.log("Strafe Start Value" + strafeEncoder.getCurrentPosition());

        teamUtil.log("MoveStraightCMwStrafeEnc cms:" + centimeters + " strafe:" + strafeTarget + " driveH:" + driveHeading + " robotH:" + robotHeading + " MaxV:" + maxVelocity + " EndV:" + endVelocity);

        float strafeFactor = .04f; // convert strafe encoder error into heading declination
        float maxHeadingDeclination = 20f; // don't veer off of straight more than this number of degrees
        float headingFactor = Math.abs(driveHeading-180)<.01 ? 1 : -1; // reverse correction for going backwards

        details = false;
        MotorData data = new MotorData();
        getDriveMotorData(data);

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        setMotorsWithEncoder();
        // all are measured in tics
        double totalTics = centimeters * COUNTS_PER_CENTIMETER;
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_DECELERATION);
        double postCruiseDistance = totalTics - decelerationDistance;
        if (postCruiseDistance < 0) {
            double percentageToRemoveAccel = accelerationDistance / (accelerationDistance + decelerationDistance);
            accelerationDistance += postCruiseDistance * percentageToRemoveAccel;
            decelerationDistance += postCruiseDistance * percentageToRemoveAccel;
            postCruiseDistance = 0;
        }
        double distance = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
            teamUtil.log("Post Cruise distance: " + postCruiseDistance);
        }

        //acceleration
        while (distance < accelerationDistance) {
            if (details) {
                teamUtil.log("dh: " + (driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination)));
            }
            distance = getEncoderDistance(data);
            if (lastVelocity == 0) {
                driveMotorsHeadingsFR(driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor, robotHeading, MAX_ACCELERATION * distance + MIN_START_VELOCITY);
            } else {
                driveMotorsHeadingsFR(driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor, robotHeading, MAX_ACCELERATION * distance + lastVelocity);
            }
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distance);
        }

        //cruise
        while (distance < postCruiseDistance) {
            if (details) {
                teamUtil.log("dh: " + (driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination)));
            }
            distance = getEncoderDistance(data);
            driveMotorsHeadingsFR(driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor, robotHeading, maxVelocity);
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distance);
        }

        //deceleration
        double startDecelerationDistance = distance;
        double ticsUntilEnd = totalTics - distance;
        while (distance < totalTics) {
            if (details) {
                teamUtil.log("dh: " + (driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination)));
            }
            distance = getEncoderDistance(data);
            ticsUntilEnd = totalTics - distance;
            driveMotorsHeadingsFR(driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor, robotHeading, MAX_DECELERATION * ticsUntilEnd + endVelocity);

        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distance);
        }
        if (endVelocity <= MIN_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was at min end velocity");
            }
        }
        lastVelocity = endVelocity;
        teamUtil.log("Strafe Encoder End Value" + strafeEncoder.getCurrentPosition());

        teamUtil.log("MoveStraightCMwStrafeEnc--Finished");

    }

    public void moveStraightCmWithStrafeEncoderWithGoToScore(double maxVelocity, double centimeters, int strafeTarget,double cmsForLift, double driveHeading, double robotHeading, double endVelocity) {
        // same but score
        teamUtil.log("MoveStraightCMwStrafeEnc cms:" + centimeters + " strafe:" + strafeTarget + " driveH:" + driveHeading + " robotH:" + robotHeading + " MaxV:" + maxVelocity + " EndV:" + endVelocity);

        float strafeFactor = .02f; // convert strafe encoder error into heading declination
        float maxHeadingDeclination = 20f; // don't veer off of straight more than this number of degrees
        float headingFactor = Math.abs(driveHeading-180)<.01 ? 1 : -1; // reverse correction for going backwards

        details = false;
        MotorData data = new MotorData();
        getDriveMotorData(data);

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        setMotorsWithEncoder();
        // all are measured in tics
        double totalTics = centimeters * COUNTS_PER_CENTIMETER;
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_DECELERATION);
        double postCruiseDistance = totalTics - decelerationDistance;
        if (postCruiseDistance < 0) {
            double percentageToRemoveAccel = accelerationDistance / (accelerationDistance + decelerationDistance);
            accelerationDistance += postCruiseDistance * percentageToRemoveAccel;
            decelerationDistance += postCruiseDistance * percentageToRemoveAccel;
            postCruiseDistance = 0;
        }
        double distance = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
            teamUtil.log("Post Cruise distance: " + postCruiseDistance);
        }

        //acceleration
        while (distance < accelerationDistance) {
            if (details) {
                teamUtil.log("dh: " + (driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination)));
            }
            distance = getEncoderDistance(data);
            if (lastVelocity == 0) {
                driveMotorsHeadingsFR(driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor, robotHeading, MAX_ACCELERATION * distance + MIN_START_VELOCITY);
            } else {
                driveMotorsHeadingsFR(driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor, robotHeading, MAX_ACCELERATION * distance + lastVelocity);
            }
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distance);
        }

        //cruise
        while (distance < postCruiseDistance) {
            if (details) {
                teamUtil.log("dh: " + (driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination)));
            }
            distance = getEncoderDistance(data);
            if(distance>cmsForLift*COUNTS_PER_CENTIMETER){
                output.goToScoreNoWait(4,output.GrabberRotatorHorizontal2,output.StraferLoad);
            }
            driveMotorsHeadingsFR(driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor, robotHeading, maxVelocity);
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distance);
        }

        //deceleration
        double startDecelerationDistance = distance;
        double ticsUntilEnd = totalTics - distance;
        while (distance < totalTics) {
            if (details) {
                teamUtil.log("dh: " + (driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination)));
            }
            distance = getEncoderDistance(data);
            ticsUntilEnd = totalTics - distance;
            driveMotorsHeadingsFR(driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)*strafeFactor, -maxHeadingDeclination, maxHeadingDeclination) * headingFactor, robotHeading, MAX_DECELERATION * ticsUntilEnd + endVelocity);

        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distance);
        }
        if (endVelocity <= MIN_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was at min end velocity");
            }
        }
        lastVelocity = endVelocity;
        teamUtil.log("MoveStraightCMwStrafeEnc--Finished");

    }

    public boolean waitForStall(long timeout){
        // wait for the robot to slow down on the wall
        // expects setPower
        boolean details = false;
        teamUtil.log("Waiting For Stall");
        long timeoutTime = System.currentTimeMillis()+timeout;
        int lastEncoder = forwardEncoder.getCurrentPosition();
        double startEncoderVelocity = forwardEncoder.getVelocity();
        while(teamUtil.keepGoing(timeoutTime)){
            teamUtil.pause(25);
            if(details){teamUtil.log("Forward Encoder Velocity: " + forwardEncoder.getVelocity());}
            int currentEncoder = forwardEncoder.getCurrentPosition();
            if (details) teamUtil.log("last: " + lastEncoder + " current: "+ currentEncoder);
            if(forwardEncoder.getVelocity()<startEncoderVelocity*0.5){
                teamUtil.log("Stalled");
                return true;
            }
            lastEncoder = currentEncoder;
        }
        teamUtil.log("Didn't Stall");
        return false; // didn't get a stall
    }

    /************************************************************************************************************
     /************************************************************************************************************
     /************************************************************************************************************
     *
     * Methods to turn the robot in place
     *
     /************************************************************************************************************
     /************************************************************************************************************/
    public void spinToHeading(double heading) {
        // moves at full speed then decelerates to spin
        double velocity = MAX_VELOCITY;
        boolean turningLeft;
        double startHeading = getHeading();
        double currentHeading = getHeading();
        double leftCoefficient = 1;
        double rightCoefficient = 1;
        setMotorsWithEncoder();
        if (heading > currentHeading) { // fix direction
            if (heading - currentHeading < 180) {
                leftCoefficient = -1;
            } else {
                rightCoefficient = -1;
            }
        } else {
            if (currentHeading - heading < 180) {
                rightCoefficient = -1;
            } else {
                leftCoefficient = -1;
            }
        }
        if (details) {
            teamUtil.log("turning left: " + rightCoefficient);
            teamUtil.log("current heading: " + currentHeading);
            teamUtil.log("heading goal: " + (heading + DRIFT_SPINS));
        }
        if (details) {
            teamUtil.log("crossing 0/360 barrier");
        }
        while (Math.abs(currentHeading - heading) > SPIN_END_OF_MAX_VELOCITY) {
            setMotorVelocities(leftCoefficient * velocity, rightCoefficient * velocity, leftCoefficient * velocity, rightCoefficient * velocity);
            currentHeading = getHeading();
        }
        if (details) {
            teamUtil.log("current heading: " + currentHeading);
            teamUtil.log("heading cutoff (greater): " + adjustAngle(heading - CRAWL_DISTANCE_SPINS));
            teamUtil.log("done with max velocity phase");
            teamUtil.log("heading: " + currentHeading);
        }
        while (Math.abs(currentHeading - heading) > CRAWL_DISTANCE_SPINS) {
            currentHeading = getHeading();
            velocity = ((MAX_VELOCITY - CRAWL_SPEED) / (SPIN_END_OF_MAX_VELOCITY - CRAWL_DISTANCE_SPINS)) * (Math.abs(currentHeading - heading) - SPIN_END_OF_MAX_VELOCITY) + MAX_VELOCITY; // wrote an equasion
            if (velocity < CRAWL_SPEED) {
                velocity = CRAWL_SPEED;
            }
            setMotorVelocities(leftCoefficient * velocity, rightCoefficient * velocity, leftCoefficient * velocity, rightCoefficient * velocity);
        }

        if (details) {
            teamUtil.log("done with deceleration phase");
            teamUtil.log("heading: " + currentHeading);
        }
        while (Math.abs(currentHeading - heading) > DRIFT_SPINS) {
            currentHeading = getHeading();
            velocity = CRAWL_SPEED;
            setMotorVelocities(leftCoefficient * velocity, rightCoefficient * velocity, leftCoefficient * velocity, rightCoefficient * velocity);
        }

        if (details) {
            teamUtil.log("done with crawl phase");
            teamUtil.log("heading: " + currentHeading);
        }

        setMotorsBrake();
        setMotorPower(0);
    }


    /************************************************************************************************************
     /************************************************************************************************************
     /************************************************************************************************************
     *
     * Methods to drive based on sensors and CV
     *
     /************************************************************************************************************
     /************************************************************************************************************/

    public void stopAtUltDistance(double distanceFromWall, double robotHeading, double driveHeading) {
        boolean details = true;
        MotorData data = new MotorData();
        getDriveMotorData(data);
        setMotorsWithEncoder();
        distanceFromWall = distanceFromWall + 15; // adjust for moving average computed by sensor

        double distance = getUltrasonicDistance();

        if (details) {
            teamUtil.log("starting distance: " + distance);
            teamUtil.log("goal distance: " + distanceFromWall);
        }
        if (distance <= distanceFromWall) {
            if (details) {
                teamUtil.log("already within distance");
            }
            return;
        }
        while (distanceFromWall < distance) {
            //log("in while loop");
            distance = getUltrasonicDistance();
            teamUtil.log("distance: " + distance);
            //double velocity = MIN_END_VELOCITY+Math.abs(distance-distanceFromWall)*VELOCITY_DECREASE_PER_CM;
            //log("Velocity: "+velocity);
            driveMotorsHeadingsFR(driveHeading, robotHeading, 500); // TODO: Tested at 500, could be faster?
        }
        if (details) {
            teamUtil.log("distance before pause: " + distance);
        }
        stopMotors();
        setMotorsBrake();

        teamUtil.pause(1000);

        distance = getUltrasonicDistance();
        if (details) {
            teamUtil.log("distance at end: " + distance);
        }
    }

    


    // drive until either rear proximity sensor is triggered, OR we are interrupted, OR we time out.
    // Returns true if it was successful, false if it timed out
    // Does NOT stop motors at end!
    public boolean driveToProximity(double driveHeading, double robotHeading, double velocity, long timeout) {
        teamUtil.log("Drive To Proximity");
        long timeOutTime = System.currentTimeMillis() + timeout;
        while ((teamUtil.keepGoing(timeOutTime) && !manualInterrupt.get()) && !getLeftProximity() && !getRightProximity()) {
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            teamUtil.log("Closing: " + manualInterrupt.get() + " " + getLeftProximity() + "/" + getRightProximity());
        }
        movingAutonomously.set(false);
        return System.currentTimeMillis() < timeOutTime;
    }

    public void driveToProximityNoWait(double driveHeading, double robotHeading, double velocity, long timeout) {
        if (movingAutonomously.get()) { // Already in an autonomous operation
            teamUtil.log("WARNING: Attempt to driveToProximity while drive system is in autonomous operation--ignored");
            return;
        } else {
            movingAutonomously.set(true); // signal that we are in an auto operation
            manualInterrupt.set(false); // reset interrupt flag
            teamUtil.log("Launching Thread to driveToProximity");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    driveToProximity(driveHeading, robotHeading, velocity, timeout);
                }
            });
            thread.start();
        }
    }

    // Drives forward until robot sees white tape, then strafes to line up on it and then goes to wall
    // returns true if it succeeded, false if things went horribly wrong.
    public boolean driveToStack(double driveHeading, double robotHeading, double velocity, long timeout) {
        //start with a minimum of 40 cms from the wall
        boolean details = false;
        findLineProcesser.details = details;
        teamUtil.log("Drive To Stack");
        int driftPixels = (int) (findLineProcesser.CAMWIDTH * .075);
        long timeOutTime = System.currentTimeMillis() + timeout;
        findLineProcesser.reset();

        // Drive until we see the line

        double startEncoderValue = fl.getCurrentPosition();
        while (!findLineProcesser.sawLine() && teamUtil.keepGoing(timeOutTime)) {
            if(fl.getCurrentPosition()-startEncoderValue>=25*COUNTS_PER_CENTIMETER){
                stopMotors();
                teamUtil.log("Drive To Stack Failed to See Line");
                return false;
            }
            if (details) {
                teamUtil.log("Looking for Line. ");
            }
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            teamUtil.pause(50); // give CPU time to image processing
        }

        // kill forward momentum to preserve camera field of view
        // TODO: Might not be needed
        stopMotors();
        teamUtil.pause(250);

        teamUtil.log("Strafing to Line");
        if (findLineProcesser.lastValidMidPoint.get() > findLineProcesser.MIDPOINTTARGET) {
            // TODO: Need another failsafe here to make sure we don't strafe off of the tile
            while (findLineProcesser.lastValidMidPoint.get() > findLineProcesser.MIDPOINTTARGET + driftPixels && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(90, 180, velocity);
                teamUtil.pause(50); // give CPU time to image processing
            }
        } else {
            // TODO: And here!
            while (findLineProcesser.lastValidMidPoint.get() < findLineProcesser.MIDPOINTTARGET - driftPixels && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(270, 180, velocity);
                teamUtil.pause(50); // give CPU time to image processing
            }
        }
        findLineProcesser.details = false; // Don't continue to clutter up log

        // kill strafe momentum
        stopMotors();
        teamUtil.pause(250);

        // Use proximity to end of tape to adjust distance to wall
        int visiblePixels = findLineProcesser.CAMHEIGHT - findLineProcesser.cropRect.height;
        int pixelsToClose = findLineProcesser.CAMHEIGHT - findLineProcesser.lastValidBottom.get();
        double cmsToStack = ((float) pixelsToClose / (float) visiblePixels) * (26.5 - 18.5) + 18.5; // Assume Camera view is linear.  Might not be
        teamUtil.log("Driving to wall: " + cmsToStack);
        moveCm(MAX_VELOCITY, cmsToStack, driveHeading, robotHeading, 0); // maxVelocity was 400
        stopMotors();

        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("Drive To Stack --TIMED OUT!");
        } else {
            teamUtil.log("Drive To Stack - Finished");
        }
        return true;
    }

    public boolean driveToStackNoStop(double driveHeading, double robotHeading, double velocity, double endVelocity, long timeout) {
        //start with a minimum of 40 cms from the wall
        boolean details = false;
        findLineProcesser.details = details;
        teamUtil.log("Drive To Stack");
        int driftPixels = (int) (findLineProcesser.CAMWIDTH * .075);
        long timeOutTime = System.currentTimeMillis() + timeout;
        findLineProcesser.reset();

        // Drive until we see the line

        double startEncoderValue = fl.getCurrentPosition();
        while (!findLineProcesser.sawLine() && teamUtil.keepGoing(timeOutTime)) {
            if(fl.getCurrentPosition()-startEncoderValue>=25*COUNTS_PER_CENTIMETER){
                stopMotors();
                teamUtil.log("Drive To Stack Failed to See Line");
                return false;
            }
            if (details) {
                teamUtil.log("Looking for Line. ");
            }
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            teamUtil.pause(50); // give CPU time to image processing
        }




        double cmsDiagonally;

        //vertical data
        int visiblePixels = findLineProcesser.CAMHEIGHT - findLineProcesser.cropRect.height;
        int pixelsToClose = findLineProcesser.CAMHEIGHT - findLineProcesser.lastValidBottom.get();
        double cmsToStack = ((float) pixelsToClose / (float) visiblePixels) * (26.5 - 18.5) + 18.5;
        teamUtil.log("Cms to Stack (vertical): " + cmsToStack);

        //horizontal data
        //leftmidpoint = 48 (robots right)
        //rightmidpoint = 581 (robots left) (8cm away)
        //distance in pixels to middle divided by distance in cm to middle
        float horizontalCmsPerPixel = 8f/(581f-findLineProcesser.MIDPOINTTARGET);
        float horizontalDistanceToStackPixels =Math.abs(findLineProcesser.lastValidMidPoint.get() - findLineProcesser.MIDPOINTTARGET);
        teamUtil.log("Horizontal Last Valid Midpoint: " + findLineProcesser.lastValidMidPoint.get());

        float horizontalDistanceToStackCms =horizontalDistanceToStackPixels*horizontalCmsPerPixel;
        teamUtil.log("Horizontal Distance To Stack Pixels: " + horizontalDistanceToStackPixels);

        teamUtil.log("Horizontal Distance To Stack Cms: " + horizontalDistanceToStackCms);


        cmsDiagonally=Math.sqrt(Math.pow(horizontalDistanceToStackCms,2)*2);
        teamUtil.log("Cms to travel diagonally " + cmsDiagonally);

        moveCm(velocity,cmsDiagonally,findLineProcesser.lastValidMidPoint.get() > findLineProcesser.MIDPOINTTARGET? 135 : 225,180,endVelocity);


            /*
            // TODO: Need another failsafe here to make sure we don't strafe off of the tile
            while (findLineProcesser.lastValidMidPoint.get() > findLineProcesser.MIDPOINTTARGET + driftPixels && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(90, 180, velocity);
                teamUtil.pause(50); // give CPU time to image processing
            }

             */


            /*
            // TODO: And here!
            while (findLineProcesser.lastValidMidPoint.get() < findLineProcesser.MIDPOINTTARGET - driftPixels && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(270, 180,velocity);
                teamUtil.pause(50); // give CPU time to image processing
            }

             */

        findLineProcesser.details = false; // Don't continue to clutter up log
        /*
        // kill strafe momentum
        stopMotors();
        teamUtil.pause(250);

         */

        // Use proximity to end of tape to adjust distance to wall
         // Assume Camera view is linear.  Might not be
        double cmsLeftToStack = cmsToStack-horizontalDistanceToStackCms;
        teamUtil.log("Driving to wall: " + cmsLeftToStack);
        moveCm(500, cmsToStack-horizontalDistanceToStackCms, driveHeading, robotHeading, 0); // maxVelocity was 400
        stopMotors();

        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("Drive To Stack --TIMED OUT!");
        } else {
            teamUtil.log("Drive To Stack - Finished");
        }
        return true;
    }

    public boolean driveToStackNoStopWithStrafe(double driveHeading, double robotHeading, double velocity, long timeout) {
        //start with a minimum of 40 cms from the wall
        int driftCms;
        boolean details = false;
        findLineProcesser.details = details;
        teamUtil.log("Drive To Stack No Stop With Strafe");
        int driftPixels = (int) (findLineProcesser.CAMWIDTH * .075);
        long timeOutTime = System.currentTimeMillis() + timeout;
        findLineProcesser.reset();

        // Drive until we see the line

        double startEncoderValue = fl.getCurrentPosition();
        while (!findLineProcesser.sawLine() && teamUtil.keepGoing(timeOutTime)) {
            if(fl.getCurrentPosition()-startEncoderValue>=25*COUNTS_PER_CENTIMETER){
                stopMotors();
                teamUtil.log("Drive To Stack Failed to See Line");
                return false;
            }
            if (details) {
                teamUtil.log("Looking for Line. ");
            }
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            teamUtil.pause(50); // give CPU time to image processing
        }


        float horizontalCmsPerPixel = 20f/(581f-findLineProcesser.MIDPOINTTARGET);
        float horizontalDistanceToStackPixels =Math.abs(findLineProcesser.lastValidMidPoint.get() - findLineProcesser.MIDPOINTTARGET);
        teamUtil.log("Horizontal Last Valid Midpoint: " + findLineProcesser.lastValidMidPoint.get());

        float horizontalDistanceToStackCms =horizontalDistanceToStackPixels*horizontalCmsPerPixel;
        teamUtil.log("Horizontal Distance To Stack Pixels: " + horizontalDistanceToStackPixels);

        teamUtil.log("Horizontal Distance To Stack Cms: " + horizontalDistanceToStackCms);

        teamUtil.log("Strafing to Line");
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double strafeEncoderTarget = (horizontalDistanceToStackCms-1)*TICS_PER_CM_STRAFE_ENCODER;
        if(horizontalDistanceToStackCms<1){
            teamUtil.log("No strafe needed; within 1 cm of stack line");

        }

        else if (findLineProcesser.lastValidMidPoint.get() > findLineProcesser.MIDPOINTTARGET) {
            teamUtil.log("To The Left Of the Line");
            if(horizontalDistanceToStackCms>5){
                teamUtil.log("Has to correct strafe distance because too far away");

                strafeEncoderTarget=strafeEncoderTarget-2*TICS_PER_CM_STRAFE_ENCODER;
            }
            // TODO: Need another failsafe here to make sure we don't strafe off of the tile
            //moveCm(350, horizontalDistanceToStackCms, 90, 180, 500); // maxVelocity was 400
            strafeToEncoder(90,180,400,strafeEncoderTarget,2000);

            /*
            while (findLineProcesser.lastValidMidPoint.get() > findLineProcesser.MIDPOINTTARGET + driftPixels && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(90, 180,350);
                //teamUtil.pause(50); // give CPU time to image processing
            }

             */
        } else {
            teamUtil.log("To The Right Of the Line");

            // TODO: And here!
            //moveCm(350, horizontalDistanceToStackCms, 270, 180, 500); // maxVelocity was 400
            strafeToEncoder(270,180,400,-strafeEncoderTarget,2000);
            /*
            while (findLineProcesser.lastValidMidPoint.get() < findLineProcesser.MIDPOINTTARGET - driftPixels && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(270, 180, 350);
                //teamUtil.pause(50); // give CPU time to image processing
            }

             */
        }
        findLineProcesser.details = false; // Don't continue to clutter up log



        // Use proximity to end of tape to adjust distance to wall
        int visiblePixels = findLineProcesser.CAMHEIGHT - findLineProcesser.cropRect.height;
        int pixelsToClose = findLineProcesser.CAMHEIGHT - findLineProcesser.lastValidBottom.get();
        //double cmsToStack = ((float) pixelsToClose / (float) visiblePixels) * (26.5 - 18.5) + 18.5; // Assume Camera view is linear.  Might not be
        //double cmsToStack = ((float) pixelsToClose / (float) visiblePixels) * (40.5 - 32.5) + 32.5; // Assume Camera view is linear.  Might not be
        double cmsToStack = Math.log10((findLineProcesser.lastValidBottom.get()-160)/4)/Math.log10(0.9)+70;
        teamUtil.log("Driving to wall: " + cmsToStack);
        //moveCm(MAX_VELOCITY, cmsToStack-3.5, driveHeading, robotHeading, 0); // maxVelocity was 400
        //moveCm(350, .5, 0, robotHeading, 0); // little back up
        //stopMotors();
        moveCm(MAX_VELOCITY, cmsToStack-8.5, 180, 180,400); // was 81
        setMotorsFloat(); // coast to wall

        stopMotors();
        teamUtil.pause(250);
        setMotorsBrake();
        //moveCm(350, .5, 0, robotHeading, 0); // little back up


        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("Drive To Stack --TIMED OUT!");
        } else {
            teamUtil.log("Drive To Stack - Finished");
        }
        return true;
    }
    //42 231 lowest
    //29 480
    //35.5 320
    public boolean driveToStackNoStopWithStrafeV2(double driveHeading, double robotHeading, double velocity, long timeout) {
        //start with a minimum of 40 cms from the wall
        int driftCms;
        boolean details = false;
        findLineProcesser.details = details;
        teamUtil.log("Drive To Stack No Stop With Strafe");
        int driftPixels = (int) (findLineProcesser.CAMWIDTH * .075);
        long timeOutTime = System.currentTimeMillis() + timeout;
        findLineProcesser.reset();

        // Drive until we see the line

        double startEncoderValue = fl.getCurrentPosition();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.HEARTBEAT_WHITE);

        while (!findLineProcesser.sawLine() && teamUtil.keepGoing(timeOutTime)) {
            if(fl.getCurrentPosition()-startEncoderValue>=25*COUNTS_PER_CENTIMETER){
                stopMotors();
                teamUtil.log("Drive To Stack Failed to See Line");
                return false;
            }
            if (details) {
                teamUtil.log("Looking for Line. ");
            }
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            teamUtil.pause(50); // give CPU time to image processing
        }
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);



        float horizontalCmsPerPixel = 10f/(581f-findLineProcesser.MIDPOINTTARGET);
        float horizontalDistanceToStackPixels =Math.abs(findLineProcesser.lastValidMidPoint.get() - findLineProcesser.MIDPOINTTARGET);
        teamUtil.log("Horizontal Last Valid Midpoint: " + findLineProcesser.lastValidMidPoint.get());

        float horizontalDistanceToStackCms =horizontalDistanceToStackPixels*horizontalCmsPerPixel;
        teamUtil.log("Horizontal Distance To Stack Pixels: " + horizontalDistanceToStackPixels);

        teamUtil.log("Horizontal Distance To Stack Cms: " + horizontalDistanceToStackCms);

        teamUtil.log("Strafing to Line");
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double strafeEncoderTarget = (horizontalDistanceToStackCms-1)*TICS_PER_CM_STRAFE_ENCODER;
        if(horizontalDistanceToStackCms<1){
            teamUtil.log("No strafe needed; within 1 cm of stack line");

        }

        else if (findLineProcesser.lastValidMidPoint.get() > findLineProcesser.MIDPOINTTARGET) {
            teamUtil.log("To The Left Of the Line");
            if(horizontalDistanceToStackCms>5){
                teamUtil.log("Has to correct strafe distance because too far away");

                strafeEncoderTarget=strafeEncoderTarget-2*TICS_PER_CM_STRAFE_ENCODER;
            }
            // TODO: Need another failsafe here to make sure we don't strafe off of the tile
            //moveCm(350, horizontalDistanceToStackCms, 90, 180, 500); // maxVelocity was 400
            strafeToEncoder(90,180,400,strafeEncoderTarget,2000);

            /*
            while (findLineProcesser.lastValidMidPoint.get() > findLineProcesser.MIDPOINTTARGET + driftPixels && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(90, 180,350);
                //teamUtil.pause(50); // give CPU time to image processing
            }

             */
        } else {
            teamUtil.log("To The Right Of the Line");

            // TODO: And here!
            //moveCm(350, horizontalDistanceToStackCms, 270, 180, 500); // maxVelocity was 400
            strafeToEncoder(270,180,400,-strafeEncoderTarget,2000);
            /*
            while (findLineProcesser.lastValidMidPoint.get() < findLineProcesser.MIDPOINTTARGET - driftPixels && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(270, 180, 350);
                //teamUtil.pause(50); // give CPU time to image processing
            }

             */
        }
        findLineProcesser.details = false; // Don't continue to clutter up log



        // Use proximity to end of tape to adjust distance to wall
        double cmsToStack = Math.log10((findLineProcesser.lastValidBottom.get()-160)/4)/Math.log10(0.9)+70;
        stopCV(); // Don't need line processor any more so turn it off to allow time for next CV to start
        teamUtil.log("Driving to wall: " + cmsToStack);

        moveCm(MAX_VELOCITY, cmsToStack-5, 180, 180,400); // was 81
        /*
        setMotorsFloat(); // coast to wall

        stopMotors();
        teamUtil.pause(250);
        setMotorsBrake();
        //moveCm(350, .5, 0, robotHeading, 0); // little back up

         */


        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("Drive To Stack --TIMED OUT!");
        } else {
            teamUtil.log("Drive To Stack - Finished");
        }
        return true;
    }

    public double computeLineXOffset (double midpoint, double bottom) { // return cms
        return (-0.36*midpoint+120)/10; // TODO This could be optimzed a tiny bit more by adjusting based on bottom parameter
    }
    public boolean driveToStackNoStopWithStrafeV3(double driveHeading, double robotHeading, double velocity, double tileMidpoint, boolean average, long timeout) {
        //start with a minimum of 40 cms from the wall
        boolean details = false;
        double driftReductionFactor = .1; // drifts quite a bit coming off the 1000 velocity during the white line seek!
        //findLineProcesser.details = details;
        teamUtil.log("Drive To Stack No Stop With Strafe");
        long timeOutTime = System.currentTimeMillis() + timeout;
        findLineProcesser.reset();

        // Drive until we see the line
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.HEARTBEAT_WHITE);
        double startEncoderValue = forwardEncoder.getCurrentPosition();
        double maxEncoder = startEncoderValue+25*TICS_PER_CM_STRAIGHT_ENCODER;
        boolean sawLine = findLineProcesser.sawLine(); // Maybe we can already see it
        while (!sawLine && teamUtil.keepGoing(timeOutTime)) {
            if(forwardEncoder.getCurrentPosition()>=maxEncoder){ // ran out of room to look
                stopMotors();
                teamUtil.log("FAILED: Drive To Stack Ran out space to see Line");
                break;
            }
            if (details) teamUtil.log("Looking for Line. ");
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            teamUtil.pause(50); // give CPU time for image processing
            sawLine = findLineProcesser.sawLine();
        }
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        double lineMidpoint = tileMidpoint;
        double strafeEncoderTarget = tileMidpoint;
        double strafeHeading = 0;
        if(sawLine) {
            teamUtil.log ("Saw Line");
            double detectedMidpoint = findLineProcesser.lastValidMidPoint.get();
            double detectedBottom = findLineProcesser.lastValidBottom.get();
            double xOffset = computeLineXOffset(detectedMidpoint, detectedBottom);
            strafeEncoderTarget = strafeEncoder.getCurrentPosition()-TICS_PER_CM_STRAFE_ENCODER*xOffset;
            if (average) {
                lineMidpoint = (tileMidpoint+strafeEncoderTarget)/2; // Common Filters can be really simple!
            } else {
                lineMidpoint = strafeEncoderTarget;
            }
            strafeHeading =findLineProcesser.lastValidMidPoint.get() > findLineProcesser.MIDPOINTTARGET ? 90 : 270;
            if (details) teamUtil.log ("Mid: " + detectedMidpoint + " xOffset: "+ xOffset + " strafeTarget: "+ strafeEncoderTarget +  " h: "+ strafeHeading);
         }else {
            strafeEncoderTarget = tileMidpoint; // This is all we have to go on
            strafeHeading = strafeEncoder.getCurrentPosition()<tileMidpoint ? 90 : 270;
            // Tap the breaks here or there may not be enough room to strafe before we hit the stack!
            long timeoutTime2 = System.currentTimeMillis() + 250 + (int) teamUtil.robot.b;
            while (teamUtil.keepGoing(timeoutTime2)) {
                driveMotorsHeadingsFR(0, 180, 1000); // A desperate act to stop the robots forward momentum
            }
            stopMotors();
            teamUtil.pause(250);
            driftReductionFactor = .3; // But tapping the breaks will alter the drift on the strafe further below...
            if (details) teamUtil.log ("No Line, working with strafe Encoder. strafeTarget: "+ strafeEncoderTarget +  " h: "+ strafeHeading);
        }
        stopCV(); // Don't need line processor any more so turn it off to allow time for next CV to start
        double currentStrafe = strafeEncoder.getCurrentPosition();
        double ticsToStrafe = Math.abs(currentStrafe-strafeEncoderTarget);
        if (ticsToStrafe<2*TICS_PER_CM_STRAFE_ENCODER) {
            teamUtil.log("No strafe needed; within 2 cm of stack line");
        }else {
            teamUtil.log("Strafing to Target");
            // adjust for drift
            double driftReduction = ticsToStrafe * (driftReductionFactor);
            strafeEncoderTarget = strafeEncoderTarget > currentStrafe ? currentStrafe+driftReduction : currentStrafe-driftReduction;
            if (details) teamUtil.log ("Adjusted strafe Target: "+ strafeEncoderTarget);
            strafeToTarget(401, strafeEncoderTarget, strafeHeading, 180, 401, 1000);
        }

        findLineProcesser.details = false; // Don't continue to clutter up log
        teamUtil.log("Driving to wall: ");
        driveStraightToTargetWithStrafeEncoderValue(velocity, 0-4*TICS_PER_CM_STRAIGHT_ENCODER, lineMidpoint, 180,180, 400, 1500);
        driveMotorsHeadingsFR(180, 180, 400); // try to avoid extra drift during the wait for stall
        boolean stallResult = waitForStall(1500);

        if(!stallResult){
            stopMotors();
            return false;
        }


        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("Drive To Stack --TIMED OUT!");
        } else {
            teamUtil.log("Drive To Stack - Finished");
        }
        return true;
    }

    public boolean strafeToStackDetection(double driveHeading, double robotHeading, double velocity, long timeout, int failsafeCms, Point location){
        // strafe until there is a valid detection of the stack
        teamUtil.log("strafeToStackDetection Starting");
        long timeOutTime = System.currentTimeMillis() + timeout;
        details = false;

        frontVisionPortal.setProcessorEnabled(findWhitePixelProcessor, true);
        findWhitePixelProcessor.reset();

        teamUtil.theBlinkin.setSignal(Blinkin.Signals.HEARTBEAT_WHITE);
        double startEncoderValue = strafeEncoder.getCurrentPosition();
        double maxEncoder = startEncoderValue+failsafeCms*TICS_PER_CM_STRAFE_ENCODER;
        boolean sawStack = findWhitePixelProcessor.getDetectionLastFrame(location); // Maybe we can already see it
        if(sawStack) teamUtil.log("Already Seeing Stack");

        while (!sawStack && teamUtil.keepGoing(timeOutTime)) {
            if(Math.abs(strafeEncoder.getCurrentPosition() - startEncoderValue)>=failsafeCms*TICS_PER_CM_STRAFE_ENCODER){ // ran out of room to look
                teamUtil.log("Strafe encoder: " + strafeEncoder.getCurrentPosition() + "Forward Encoder: " + forwardEncoder.getCurrentPosition());
                teamUtil.log("FAILED: Drive To Stack Ran out space to see Line");
                return false;
            }
            if (details) teamUtil.log("Looking for Stack. Strafe Encoder: " +strafeEncoder.getCurrentPosition());

            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            teamUtil.pause(50); // give CPU time for image processing
            sawStack = findWhitePixelProcessor.getDetectionLastFrame(location);
        }
        if(sawStack){
            teamUtil.log("Saw Stack");
            teamUtil.log("Pipline FPS: " + frontVisionPortal.getFps());
            teamUtil.log("Strafe encoder: " + strafeEncoder.getCurrentPosition() + "Forward Encoder: " + forwardEncoder.getCurrentPosition());
            teamUtil.log("Left: " + location.x + "Right: " + location.y);
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
            frontVisionPortal.setProcessorEnabled(findWhitePixelProcessor, false);
            return true;
        }
        else{
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
            frontVisionPortal.setProcessorEnabled(findWhitePixelProcessor, false);
            teamUtil.log("Pipline FPS: " + frontVisionPortal.getFps());
            teamUtil.log("FAILED: Drive To Stack Ran out Time");
            return false;
        }
    }


    public void frontLineCameraDimensionTelemetry(){
        // compute line distances
        boolean details = false;
        if (currentCam==cvCam.FRONT_LINE) {
            double xOffset = computeLineXOffset(findLineProcesser.lastValidMidPoint.get(), findLineProcesser.lastValidBottom.get());
            telemetry.addLine("LINE Last Valid Midpoint: " + findLineProcesser.lastValidMidPoint.get());
            //telemetry.addLine("Horizontal Distance To Stack Pixels: " + horizontalDistanceToStackPixels);
            telemetry.addLine("LINE Horizontal Distance To Stack Cms: " + xOffset);

            //int visiblePixels = findLineProcesser.CAMHEIGHT - findLineProcesser.cropRect.height;
            //int pixelsToClose = findLineProcesser.CAMHEIGHT - findLineProcesser.lastValidBottom.get();
            //double cmsToStack = Math.log10((findLineProcesser.lastValidBottom.get()-160)/4)/Math.log10(0.9)+70;
            //double cmsToStack = ((float) pixelsToClose / (float) visiblePixels) * (40.5 - 32.5) + 32.5; // Assume Camera view is linear.  Might not be
            //telemetry.addLine("Vertical Data");
            //telemetry.addLine("Cms to Stack: " + cmsToStack);
        } else{
            if(details){
                teamUtil.log("Front camera not currently running");
            }
        }
    }

    public double[] calculateAngle(double rightDist, double forwardsDist, double xOffset, double yOffset) {
        // helper method to find the direction needed for original apriltag
        int quadrant; // the quad the goal point would be in if the current spot was the origin
        if (rightDist < xOffset && forwardsDist > yOffset) {
            quadrant = 1;
        } else if (rightDist > xOffset && forwardsDist > yOffset) {
            quadrant = 2;
        } else if (rightDist > xOffset) {
            quadrant = 3;
        } else {
            quadrant = 4;
        }
        double angle = Math.toDegrees(Math.atan(Math.abs(yOffset - rightDist) / Math.abs(xOffset - forwardsDist)));
        double[] list = {quadrant, angle};
        return list;
    }


    public double returnAprilTagIDOffset(int id, long timeout) {
    // get distances from an apriltag
        long timeOutTime = System.currentTimeMillis() + timeout;
        while (teamUtil.keepGoing(timeOutTime)) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.id == id) {
                    teamUtil.log("April Tag Detected");

                    return detection.ftcPose.x * CMS_PER_INCH;
                }
            } // TODO: do more tries
        }
        teamUtil.log("No April Tag Seen");
        return noAprilTag;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // returns an x,y offset of the robot relative to the center of the backdrop.  Negative means robot is  left of center
    // Returns false if it fails or times out.
    public enum YellowPixelPosition {NONE, LEFT, RIGHT,FAILED};

    public YellowPixelPosition findYellowPixel(int path, long timeout){
        // detects and changes the state of the alliance parter's yellow
        boolean details = true;
        teamUtil.log("YellowPixelPosition");
        long timeoutTime = System.currentTimeMillis()+ timeout;
        while (teamUtil.keepGoing(timeoutTime)) {
            if(findPixelProcesser.foundPixel.get()){
                if (details) teamUtil.log("Found One R: "+ findPixelProcesser.farthestRight + " L: "+findPixelProcesser.farthestLeft);
                if (teamUtil.alliance==RED) {
                    if (findPixelProcesser.farthestRight > 500) { // Depends on distance to backdrop, drift, etc.!
                        teamUtil.log("YellowPixelPosition--FINISHED");
                        return YellowPixelPosition.RIGHT;
                    } else {
                        teamUtil.log("YellowPixelPosition--FINISHED");
                        return YellowPixelPosition.LEFT;
                    }
                } else {
                    if (findPixelProcesser.farthestLeft < 140) {
                        teamUtil.log("YellowPixelPosition--FINISHED");
                        return YellowPixelPosition.LEFT;
                    } else {
                        teamUtil.log("YellowPixelPosition--FINISHED");
                        return YellowPixelPosition.RIGHT;
                    }
                }
            }
        }
        teamUtil.log("YellowPixelPosition--FINISHED");
        return YellowPixelPosition.NONE;

        
    }
    public boolean getRobotBackdropOffset(org.opencv.core.Point p,boolean freshDetection) {
        // distance to backdrop
        boolean details = false;
        if (details) teamUtil.log("getRobotBackdropOffset");
        if (currentCam!=cvCam.REAR_APRILTAG) {
            teamUtil.log("ERROR: getRobotBackdropOffset called without April Tag Processor Running");
            return false;
        } else {
            List<AprilTagDetection> detections;
            if(freshDetection){
                 detections = aprilTag.getFreshDetections();

            }else{
                detections = aprilTag.getDetections();

            }
            if(detections==null){
                if (details) teamUtil.log("no fresh detections");
                return false;
            }
            float xOffsetTotal = 0, yOffsetTotal = 0;
            int numTags = 0;
            if (details) teamUtil.log("Detections: " + detections.size());

            for (AprilTagDetection detection : detections) { // Average whatever readings we have
                if (detection.ftcPose == null) {
                    // We shouldn't be here, but there might be a bug in the FTC code which allows this to happen
                    if (details) teamUtil.log("NULL ftcPose in Detection List!!");
                } else {
                    if (detection.id == 1 || detection.id == 4) { // left April Tags
                        numTags++;
                        xOffsetTotal = xOffsetTotal + (float) (detection.ftcPose.x * CMS_PER_INCH + TAG_CENTER_TO_CENTER);
                    } else if (detection.id == 3 || detection.id == 6) { // right April Tags
                        numTags++;
                        xOffsetTotal = xOffsetTotal + (float) (detection.ftcPose.x * CMS_PER_INCH - TAG_CENTER_TO_CENTER);
                    } else { // Center tags
                        numTags++;
                        xOffsetTotal = xOffsetTotal + (float) (detection.ftcPose.x * CMS_PER_INCH);
                    }
                    yOffsetTotal = yOffsetTotal + (float) (detection.ftcPose.y * CMS_PER_INCH);
                }
            }
            if (numTags > 0) {
                p.x = -1 * xOffsetTotal / numTags;
                p.y = yOffsetTotal / numTags;
                if (details) teamUtil.log("getRobotBackdropOffset - FINISHED");
                return true;
            } else {
                if (details) teamUtil.log("getRobotBackdropOffset - FINISHED");
                return false;
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // returns an x offset of the robot relative to the center of the backdrop.  Negative means robot is  left of center
    public float getRobotBackdropXOffset() {
        if (currentCam!=cvCam.REAR_APRILTAG) {
            teamUtil.log("ERROR: getRobotBackdropXOffset called without April Tag Processor Running");
            return (float) noAprilTag;
        } else {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            float offsetTotal = 0;
            int numTags = 0;
            for (AprilTagDetection detection : detections) { // Average whatever readings we have
                if (detection.id == 1 || detection.id == 4) { // left April Tags
                    numTags++;
                    offsetTotal = offsetTotal + (float) (detection.ftcPose.x * CMS_PER_INCH + TAG_CENTER_TO_CENTER);
                } else if (detection.id == 3 || detection.id == 6) { // right April Tags
                    numTags++;
                    offsetTotal = offsetTotal + (float) (detection.ftcPose.x * CMS_PER_INCH - TAG_CENTER_TO_CENTER);
                } else { // Center tags
                    numTags++;
                    offsetTotal = offsetTotal + (float) (detection.ftcPose.x * CMS_PER_INCH);
                }
            }
            if (numTags > 0) {
                return -1 * offsetTotal / numTags;
            } else {
                return (float) noAprilTag;
            }
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Drive robot to a location relative to backdrop using April Tags
    // ASSUMPTIONS:
    // Robot will still be able to see at least one April Tag at end
    // Robot is moving at 'velocity" when this method is called
    // X is cms from the middle of the backdrop (- is left, + is right)
    // Y is cms from the April tags
    // TODO: SPEED UP IDEA: enhance this to take an end velocity so you don't need to stop and waste time
    public boolean driveToAprilTagOffset(double initialVelocity, double initialDriveHeading, double robotHeading, double xOffset, double yOffset, long timeout) {
        teamUtil.log("Drive to April Tag Offset X: " + xOffset + " Y: "+ yOffset);
        boolean details = false;
        long timeOutTime = System.currentTimeMillis() + timeout;
        long aprilTagTimeoutTime = 0;
        float driftCms = 2;
        org.opencv.core.Point tagOffset = new org.opencv.core.Point();
        teamUtil.log("Continue on Initial Heading");
        aprilTag.getFreshDetections();
        while (!getRobotBackdropOffset(tagOffset,true) && teamUtil.keepGoing(timeOutTime)) {
            driveMotorsHeadingsFR(initialDriveHeading, robotHeading, initialVelocity); // continue on initial heading until we see a tag
        }
        teamUtil.log("Driving based on tags");
        while (teamUtil.keepGoing(timeOutTime)) { // Use April Tags to go the rest of the way
            double cmsToStrafe = tagOffset.x - xOffset;
            double cmsToBackup = tagOffset.y - yOffset;
            double cmsToTravel = Math.sqrt(cmsToStrafe * cmsToStrafe + cmsToBackup * cmsToBackup);
            if (Math.abs(cmsToTravel) < driftCms) {
                break;
            }
            double heading;
            if (cmsToBackup == 0) {
                heading = cmsToStrafe < 0 ? 270 : 90;
            } else if (cmsToBackup > 0) { // Using vertical (y-axis) to compute reference angles since 0 is at top
                heading = adjustAngle(Math.toDegrees(Math.atan(cmsToStrafe / cmsToBackup)));
            } else {
                heading = 180 + Math.toDegrees(Math.atan(cmsToStrafe / cmsToBackup));
            }
            double velocity = Math.min(initialVelocity, MIN_END_VELOCITY + MAX_DECELERATION * COUNTS_PER_CENTIMETER * cmsToTravel);
            if (details)
                teamUtil.log("strafe: " + cmsToStrafe + " back: " + cmsToBackup + " travel: " + cmsToTravel + " heading: " + heading + " v: " + velocity+ " y from tag: " +tagOffset.y);
            driveMotorsHeadingsFR(heading, robotHeading, velocity);
            aprilTagTimeoutTime = System.currentTimeMillis() + 1000;
            while (!getRobotBackdropOffset(tagOffset,false) && teamUtil.keepGoing(aprilTagTimeoutTime)) {
                if (details) teamUtil.log("WARNING: Lost sight of tags!");
            }

        }
        stopMotors();
        if (System.currentTimeMillis() > timeOutTime || System.currentTimeMillis() > aprilTagTimeoutTime) {
            teamUtil.log("driveToAprilTagOffset - TIMED OUT!");
            return false;
        } else {
            teamUtil.log("Drive to April Tag Offset - FINISHED");
            return true;
        }
    }

    public org.opencv.core.Point lastAprilTagOffset = new Point();
    public boolean driveToAprilTagOffsetV2(double initialVelocity, double initialDriveHeading, double robotHeading, double xOffset, double yOffset, long timeout) {
        teamUtil.log("Drive to April Tag Offset X: " + xOffset + " Y: "+ yOffset);
        boolean details = true;
        long timeOutTime = System.currentTimeMillis() + timeout;
        long aprilTagTimeoutTime = 0;
        float xDriftCms = 2.54f;
        float yDriftCms = 2.54f;
        double kAprilTagX = 2;
        double aprilTagMaxVelocity = initialVelocity;
        org.opencv.core.Point tagOffset = new org.opencv.core.Point();
        teamUtil.log("Continue on Initial Heading");
        aprilTag.getFreshDetections();
        while (!getRobotBackdropOffset(tagOffset,true) && teamUtil.keepGoing(timeOutTime)) {
            driveMotorsHeadingsFR(initialDriveHeading, robotHeading, initialVelocity); // continue on initial heading until we see a tag
        }
        teamUtil.log("Driving based on tags");
        while (teamUtil.keepGoing(timeOutTime)) { // Use April Tags to go the rest of the way
            double cmsToStrafe = tagOffset.x - xOffset;
            double cmsToBackup = tagOffset.y - yOffset;
            double cmsToTravel = Math.sqrt(cmsToStrafe * cmsToStrafe + cmsToBackup * cmsToBackup);
            if (Math.abs(cmsToStrafe) < xDriftCms && Math.abs(cmsToBackup)< yDriftCms) {
                lastAprilTagOffset.x = cmsToStrafe;
                lastAprilTagOffset.y = cmsToBackup;
                break;
            }
            double heading;
            if (cmsToBackup == 0) {
                heading = cmsToStrafe < 0 ? 270 : 90;
            } else if (cmsToBackup > 0) { // Using vertical (y-axis) to compute reference angles since 0 is at top
                heading = adjustAngle(Math.toDegrees(Math.atan(cmsToStrafe / cmsToBackup)));
            } else {
                heading = 180 + Math.toDegrees(Math.atan(cmsToStrafe / cmsToBackup));
            }
            if (heading < 90) {
                heading = Math.min(90, heading*(kAprilTagX));
            } else if (heading > 270) {
                heading = 360 - (Math.min(90, (360-heading) * (kAprilTagX)));
            } else if (heading >90 && heading < 180) {
                heading = 180 - (Math.min(90, (180-heading) * (kAprilTagX)));
            } else if (heading > 180 && heading < 270) {
                heading = 270 - (Math.min(90, (270-heading) * (kAprilTagX)));
            }
            double velocity = Math.min(initialVelocity, MIN_END_VELOCITY + MAX_DECELERATION * COUNTS_PER_CENTIMETER * cmsToTravel);
            velocity = Math.min(aprilTagMaxVelocity+teamUtil.robot.d, velocity);
            if (details)
                teamUtil.log("strafe: " + cmsToStrafe + " back: " + cmsToBackup + " travel: " + cmsToTravel + " heading: " + heading + " v: " + velocity);
            driveMotorsHeadingsFR(heading, robotHeading, velocity);
            aprilTagTimeoutTime = System.currentTimeMillis() + 1000;
            while (!getRobotBackdropOffset(tagOffset,false) && teamUtil.keepGoing(aprilTagTimeoutTime)) {
                if (details) teamUtil.log("WARNING: Lost sight of tags!");
            }

        }
        stopMotors();
        if (System.currentTimeMillis() > timeOutTime || System.currentTimeMillis() > aprilTagTimeoutTime) {
            teamUtil.log("driveToAprilTagOffset - TIMED OUT!");
            return false;
        } else {
            teamUtil.log("Drive to April Tag Offset - FINISHED");
            return true;
        }
    }

    public boolean strafeToAprilTagOffsetV2(double velocity, double driveHeading, double robotHeading, double xOffset, double yOffset, long timeout) {
        teamUtil.log("Strafe to April Tag Offset X: " + xOffset );
        boolean details = true;
        long timeOutTime = System.currentTimeMillis() + timeout;
        long aprilTagTimeoutTime = 0;
        double xDriftCms = 6.0f;
        org.opencv.core.Point tagOffset = new org.opencv.core.Point();
        teamUtil.log("Waiting to see tags");
        aprilTag.getFreshDetections();
        while (!getRobotBackdropOffset(tagOffset,true) && teamUtil.keepGoing(timeOutTime)) {
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity); // continue on initial heading until we see a tag
        }
        teamUtil.log("Driving based on tags");
        while (teamUtil.keepGoing(timeOutTime)) { // Use April Tags to go the rest of the way
            double cmsToStrafe = tagOffset.x - xOffset;
            double cmsToBackup = tagOffset.y - yOffset;

            //if ((driveHeading > 180 && cmsToStrafe < xDriftCms) || (driveHeading < 180 && cmsToStrafe > xDriftCms)) {
            if (Math.abs(cmsToStrafe) < xDriftCms) {
                lastAprilTagOffset.x = cmsToStrafe;
                lastAprilTagOffset.y = cmsToBackup;
                break;
            }

            if (details)
                teamUtil.log("strafe: " + cmsToStrafe + " back: " + cmsToBackup );
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            aprilTagTimeoutTime = System.currentTimeMillis() + 1000;
            while (!getRobotBackdropOffset(tagOffset,false) && teamUtil.keepGoing(aprilTagTimeoutTime)) {
                if (details) teamUtil.log("WARNING: Lost sight of tags!");
            }
            if (System.currentTimeMillis()>=aprilTagTimeoutTime){
                teamUtil.log("TIMED OUT after losing aprilTags" );
                stopMotors();
                return false;
            }


        }
        stopMotors();
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("StrafeToAprilTagOffset - TIMED OUT!");
            return false;
        } else {
            teamUtil.log("Strafe to April Tag Offset - FINISHED");
            return true;
        }
    }

    public void setBulkReadOff() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
    }
    public void setBulkReadAuto() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    public boolean strafeToAprilTagOffsetV3(double velocity, double xDriftCms, double driveHeading, double robotHeading, double xOffset, double yOffset, long timeout) {
        teamUtil.log("Strafe to April Tag V3 Offset X: " + xOffset );
        setBulkReadAuto();
        boolean details = false;
        long timeOutTime = System.currentTimeMillis() + timeout;
        long aprilTagTimeoutTime = 0;
        org.opencv.core.Point lastDetectedOffset = new org.opencv.core.Point();
        org.opencv.core.Point previousDetectedOffset = new org.opencv.core.Point();
        org.opencv.core.Point lastDistance = new org.opencv.core.Point();
        org.opencv.core.Point lastComputedOffset = new org.opencv.core.Point();
        long lastCycleDuration = 0; // initialize to signal first time through
        long now;

        teamUtil.log("Waiting to see tags");
        while (!getRobotBackdropOffset(previousDetectedOffset,true) && teamUtil.keepGoing(timeOutTime)) {
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity); // continue on initial heading until we see a tag
        }
        teamUtil.log("Driving based on tags");
        long lastDetectionTime = System.currentTimeMillis();
        lastComputedOffset.x = previousDetectedOffset.x;
        lastComputedOffset.y = previousDetectedOffset.y;

        // Continue until we get a second fresh detection so we can establish robot's velocity for future projections
        aprilTagTimeoutTime = System.currentTimeMillis() + 1000;
        while (!getRobotBackdropOffset(lastDetectedOffset,true) && teamUtil.keepGoing(aprilTagTimeoutTime)) {
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            if (details) teamUtil.log("Waiting for 2nd Fresh Detection");
        }
        if (System.currentTimeMillis()>=aprilTagTimeoutTime){
            teamUtil.log("TIMED OUT looking for 2nd Fresh Detection" );
            stopMotors();
            return false;
        }
        // set up the history using the first two detections
        now = System.currentTimeMillis();
        lastCycleDuration = Math.max(now-lastDetectionTime,1);
        lastDetectionTime = now;
        lastDistance.x = lastDetectedOffset.x-previousDetectedOffset.x;
        lastDistance.y = lastDetectedOffset.y-previousDetectedOffset.y;
        lastComputedOffset.x = lastDetectedOffset.x;
        lastComputedOffset.y = lastDetectedOffset.y;

        int loops = 0;
        long startTime = System.currentTimeMillis();
        while (teamUtil.keepGoing(timeOutTime)) { // Use April Tags to go the rest of the way
            double cmsToStrafe = lastComputedOffset.x - xOffset;
            double cmsToBackup = lastComputedOffset.y - yOffset;

            if ((driveHeading > 180 && cmsToStrafe > -xDriftCms) || (driveHeading < 180 && cmsToStrafe < xDriftCms)) {
            //if (Math.abs(cmsToStrafe) < xDriftCms) {
                lastAprilTagOffset.x = cmsToStrafe;
                if (loops > 2) {
                    lastAprilTagOffset.y = cmsToBackup;
                } else  {
                    lastAprilTagOffset.y = lastDetectedOffset.y - yOffset;
                }
                break;
            }
            if (details)  teamUtil.log("strafe: " + cmsToStrafe + " back: " + cmsToBackup );
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity); // Update the motors

            if (getRobotBackdropOffset(lastDetectedOffset,true)) { // If we have a fresh April Tag detection
                if (details) teamUtil.log("Updating with fresh AprilTag Detection");
                now = System.currentTimeMillis();
                // Update position with the latest and greatest
                lastComputedOffset.x = lastDetectedOffset.x;
                lastComputedOffset.y = lastDetectedOffset.y;

                // Update history for next time
                lastCycleDuration = now-lastDetectionTime;
                lastDetectionTime = now;
                lastDistance.x = lastDetectedOffset.x-previousDetectedOffset.x;
                lastDistance.y = lastDetectedOffset.y-previousDetectedOffset.y;
                previousDetectedOffset.x = lastDetectedOffset.x;
                previousDetectedOffset.y = lastDetectedOffset.y;

            } else { // No fresh detection so continue to update position and motors based on history
                if (details) teamUtil.log("Navigating without fresh AprilTag");
                // Compute new offsets assuming everything is changing linearly with time but loop time is unpredictable
                now = System.currentTimeMillis();

                double timeRatio = (now - lastDetectionTime) / lastCycleDuration;

                lastComputedOffset.x = lastDetectedOffset.x + lastDistance.x * timeRatio;
                lastComputedOffset.y = lastDetectedOffset.y + lastDistance.y * timeRatio;
            }
            loops++;
        }

        stopMotors();
        setBulkReadOff();

        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("StrafeToAprilTagOffsetV3 - TIMED OUT!");
            return false;
        } else {
            teamUtil.log("Loops: "+ loops + " Avg Loop Time: "+ (loops>0? (System.currentTimeMillis()-startTime)/loops : 0));
            teamUtil.log("Strafe to April Tag Offset V3 - FINISHED");
            return true;
        }
    }

    public boolean strafeToEncoder(double driveHeading, double robotHeading, double velocity, double targetEncoderValue, long timeout) {
        // strafe to a strafe encoder value
        long timeOutTime = System.currentTimeMillis() + timeout;
        teamUtil.log("strafeToEncoder: Current: " + strafeEncoder.getCurrentPosition() + " Target: " + targetEncoderValue);
        float driftCms = 1;
        double realTarget = targetEncoderValue + (driveHeading < 180? -1:1)*driftCms*TICS_PER_CM_STRAFE_ENCODER;
        if (driveHeading<180) {
            while (strafeEncoder.getCurrentPosition() < realTarget && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        }
        else{
            while (strafeEncoder.getCurrentPosition() > realTarget && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        }
        lastVelocity=velocity;
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("strafeToEncoder - TIMED OUT!");
            return false;
        } else {
            teamUtil.log("strafeToEncoder - FINISHED : Current: " + strafeEncoder.getCurrentPosition());
            return true;
        }
    }


    public boolean strafeToEncoderWithDecel(double driveHeading, double robotHeading, double velocity, double targetEncoderValue, double endVelocity, double decelK, long timeout) {
        long timeOutTime = System.currentTimeMillis() + timeout;
        teamUtil.log("strafeToEncoder: Current: " + strafeEncoder.getCurrentPosition() + " Target: " + targetEncoderValue);
        float driftCms = 1;
        double realTarget = targetEncoderValue + (driveHeading < 180? -1:1)*driftCms*TICS_PER_CM_STRAFE_ENCODER;
        double strafeCmsToGo;
        double liveVelocity;
        if(endVelocity<MIN_END_VELOCITY){
            endVelocity = MIN_END_VELOCITY;
        }

        if (driveHeading<180) {
            while (strafeEncoder.getCurrentPosition() < realTarget && teamUtil.keepGoing(timeOutTime)) {
                strafeCmsToGo = Math.abs(targetEncoderValue-strafeEncoder.getCurrentPosition())/TICS_PER_CM_STRAFE_ENCODER;
                liveVelocity = Math.min(velocity, endVelocity+decelK* COUNTS_PER_CENTIMETER * strafeCmsToGo);
                driveMotorsHeadingsFR(driveHeading, robotHeading, liveVelocity);
            }
        }
        else{
            while (strafeEncoder.getCurrentPosition() > realTarget && teamUtil.keepGoing(timeOutTime)) {
                strafeCmsToGo = Math.abs(targetEncoderValue-strafeEncoder.getCurrentPosition())/TICS_PER_CM_STRAFE_ENCODER;
                liveVelocity = Math.min(velocity, endVelocity+decelK* COUNTS_PER_CENTIMETER * strafeCmsToGo);
                driveMotorsHeadingsFR(driveHeading, robotHeading, liveVelocity);
            }
        }
        lastVelocity=velocity;
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("strafeToEncoder - TIMED OUT!");
            return false;
        } else {
            teamUtil.log("strafeToEncoder - FINISHED : Current: " + strafeEncoder.getCurrentPosition());
            return true;
        }
    }



    /************************************************************************************************************
     /************************************************************************************************************
     /************************************************************************************************************
     *
     * Methods to drive based on joystick values
     *
     /************************************************************************************************************
     /************************************************************************************************************/
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Drive robot based on two joystick values
    // Implements a deadband where joystick position will be ignored (translation and rotation)
    // Uses a linear scale that starts at the edge of the dead band
    // Attempts to hold the last heading that was commanded via a turn
    public void driveJoyStick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast) {
        // returns values to drive to the main loop
        boolean details = true;

        float DEADBAND = 0.1f;
        float SLOPE = 1.6f;
        float FASTSLOPE = 3.6f;
        float SLOWSPEED = .1f;
        float STRAFESLOWSPEED = 0.25f;
        //float SLOPE = 1 / (1 - DEADBAND); // linear from edge of dead band to full power  TODO: Should this be a curve?

        float POWERFACTOR = 1; // MAKE LESS THAN 0 to cap upperlimit of power
        float leftX;
        float leftY;
        float rotationAdjustment;

        float scaleAmount = isFast ? 1f : 0.5f; // full power is "fast", half power is "slow"

        float frontLeft, frontRight, backRight, backLeft;

        // Ensure nothing happens if we are inside the deadband
        if (Math.abs(leftJoyStickX) < DEADBAND) {
            leftJoyStickX = 0;
        }
        if (Math.abs(leftJoyStickY) < DEADBAND) {
            leftJoyStickY = 0;
        }
        if (Math.abs(rightJoyStickX) < DEADBAND) {
            rightJoyStickX = 0;
        }

        if (movingAutonomously.get() && (leftJoyStickX != 0 || rightJoyStickX != 0 || leftJoyStickY != 0)) { // Do we need to interrupt an autonomous operation?
            manualInterrupt.set(true);
        }
        if(leftJoyStickX == 0){ // NEW Power Curves
            leftX = 0;
        } else if(Math.abs(leftJoyStickX)<.75){
            leftX = leftJoyStickX>0? STRAFESLOWSPEED:-STRAFESLOWSPEED;
        } else{
            if (isFast) {
                leftX = leftJoyStickX * FASTSLOPE + (leftJoyStickX>0? -2.6f:2.6f);
            }
            else{
                leftX = leftJoyStickX * SLOPE + (leftJoyStickX>0? -1.1f:1.1f);
            }
        }

        if(leftJoyStickY == 0){
            leftY = 0;
        } else if(Math.abs(leftJoyStickY)<.75){
            leftY = leftJoyStickY>0? SLOWSPEED:-SLOWSPEED;
        } else{
            if (isFast) {
                leftY = leftJoyStickY * FASTSLOPE + (leftJoyStickY>0? -2.6f:2.6f);
            }
            else{
                leftY = leftJoyStickY *SLOPE + (leftJoyStickY>0? -1.1f:1.1f);
            }
        }

        final float MAXROTATIONFACTOR = 0.8f;
        if (Math.abs(rightJoyStickX) > DEADBAND) { // driver is turning the robot
            rotationAdjustment = (float) (rightJoyStickX * 0.525 * scaleAmount);
            holdingHeading = false;
        } else { // Need to automatically hold the current heading
            if (!holdingHeading) { // start to hold current heading
                heldHeading = getHeading();
                holdingHeading = true;
            }
            rotationAdjustment = (float) getHeadingError(heldHeading) * -1f * .05f; // auto rotate to held heading
            rotationAdjustment = rotationAdjustment * Math.min(Math.max(Math.abs(leftX), Math.abs(leftY)), 0.7f); // make it proportional to speed
            rotationAdjustment = MathUtils.clamp(rotationAdjustment, -MAXROTATIONFACTOR,MAXROTATIONFACTOR ); // clip rotation so it doesn't obliterate translation
        }

        frontLeft = -(leftY - leftX - rotationAdjustment);
        frontRight = (-leftY - leftX - rotationAdjustment);
        backRight = (-leftY + leftX - rotationAdjustment);
        backLeft = -(leftY + leftX - rotationAdjustment);

        if (details) {

            teamUtil.telemetry.addLine("Joy X/Y: "+ leftJoyStickX+ "/"+ leftJoyStickY+ " X/Y: "+ leftX+ "/"+leftY);
            if (holdingHeading) {
                teamUtil.telemetry.addData("HOLDING:", heldHeading);
            }
        }

        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        br.setPower(backRight);
        bl.setPower(backLeft);

        //TODO: take out soon just for testing purposes
        telemetry.addLine("Left Joystick Y: " + leftJoyStickY);
        telemetry.addLine("Left Joystick X: " + leftJoyStickX);

        telemetry.addLine("fl power: " + frontLeft);
        telemetry.addLine("fr power: " + frontRight);
        telemetry.addLine("bl power: " + backLeft);
        telemetry.addLine("br power: " + backRight);


    }

    public void driveJoyStickV2(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast, boolean isSlow) {
        boolean details = true;

        float DEADBAND = 0.1f;
        float SLOWSLOPE =0.22f;
        float SLOWSLOPESTRAFE =0.35f;
        float SLOPE = 0.55f;
        float FASTSLOPE = 1f;
        float SLOWSPEED = .1f;
        //float STRAFESLOWSPEED = 0.25f;
        //float SLOPE = 1 / (1 - DEADBAND); // linear from edge of dead band to full power  TODO: Should this be a curve?

        float POWERFACTOR = 1; // MAKE LESS THAN 0 to cap upperlimit of power
        float leftX;
        float leftY;
        float rotationAdjustment;

        float scaleAmount = isFast ? 1f : 0.5f; // full power is "fast", half power is "slow"

        float frontLeft, frontRight, backRight, backLeft;

        // Ensure nothing happens if we are inside the deadband
        if (Math.abs(leftJoyStickX) < DEADBAND) {
            leftJoyStickX = 0;
        }
        if (Math.abs(leftJoyStickY) < DEADBAND) {
            leftJoyStickY = 0;
        }
        if (Math.abs(rightJoyStickX) < DEADBAND) {
            rightJoyStickX = 0;
        }

        if (movingAutonomously.get() && (leftJoyStickX != 0 || rightJoyStickX != 0 || leftJoyStickY != 0)) { // Do we need to interrupt an autonomous operation?
            manualInterrupt.set(true);
        }
        if(leftJoyStickX == 0){ // NEW Power Curves
            leftX = 0;
        } else if(isSlow){
            leftX = leftJoyStickX*SLOWSLOPESTRAFE+(leftJoyStickX>0? -0.078f:0.078f);
        } else if(isFast){
            leftX = leftJoyStickX * FASTSLOPE ;
        }
        //medium speed
        else{
            leftX = leftJoyStickX*SLOPE+(leftJoyStickX>0? -0.055f:0.055f);

        }

        if(leftJoyStickY == 0){
            leftY = 0;
        } else if(isSlow){
            leftY = leftJoyStickY*SLOWSLOPE+(leftJoyStickY>0? -0.078f:0.078f);
        } else if(isFast){
            leftY = leftJoyStickY * FASTSLOPE ;
        }
        //medium speed
        else{
            leftY = leftJoyStickY*SLOPE+(leftJoyStickY>0? -0.055f:0.055f);

        }
//
//        if(leftJoyStickX>0){ // apply power curve to x value
//            if(Math.abs(leftJoyStickX)<.5){
//                leftX = SLOWSPEED;
//            }
//            else{
//                if (isFast) {
//                    leftX = leftJoyStickX * FASTSLOPE + (-.8f);
//                }
//                else{
//                    leftX = leftJoyStickX *SLOPE + (-.3f);
//                }
//            }
//
//
//        }
//        else if (leftJoyStickX<0) {
//            if(Math.abs(leftJoyStickX)<.5){
//                leftX = -SLOWSPEED;
//            }
//            else{
//                if (isFast) {
//                    leftX = leftJoyStickX * FASTSLOPE + .8f;
//                }
//                else{
//                    leftX = leftJoyStickX *SLOPE + .3f;
//                }
//            }
//        }
//        else{
//            leftX = 0;
//        }
//
//        if(leftJoyStickY>0){
//            if(Math.abs(leftJoyStickY)<.5){
//                leftY = SLOWSPEED;
//            }
//            else{
//                if (isFast) {
//                    leftY = leftJoyStickY * FASTSLOPE + (-.8f);
//                }
//                else{
//                    leftY = leftJoyStickY *SLOPE + (-.3f);
//                }
//            }
//
//
//        }
//        else if (leftJoyStickY<0) {
//            if(Math.abs(leftJoyStickY)<.5){
//                leftY = -SLOWSPEED;
//            }
//            else{
//                if (isFast) {
//                    leftY = leftJoyStickY * FASTSLOPE + .8f;
//                }
//                else{
//                    leftY = leftJoyStickY * SLOPE + .3f;
//                }
//            }
//        }
//        else{
//            leftY = 0;
//        }

//        if (leftJoyStickX > 0) {
//            leftX = (leftJoyStickX - DEADBAND) * SLOPE * scaleAmount * POWERFACTOR;
//        } else if (leftJoyStickX < 0) {
//            leftX = (leftJoyStickX + DEADBAND) * SLOPE * scaleAmount * POWERFACTOR;
//        } else {
//            leftX = 0;
//        }
//        if (leftJoyStickY > 0) {
//            leftY = (leftJoyStickY - DEADBAND) * SLOPE * scaleAmount * POWERFACTOR;
//        } else if (leftJoyStickY < 0) {
//            leftY = (leftJoyStickY + DEADBAND) * SLOPE * scaleAmount * POWERFACTOR;
//        } else {
//            leftY = 0;
//        }

        final float MAXROTATIONFACTOR = 0.8f;
        if (Math.abs(rightJoyStickX) > DEADBAND) { // driver is turning the robot
            rotationAdjustment = (float) (rightJoyStickX * 0.525 * scaleAmount);
            holdingHeading = false;
        } else { // Need to automatically hold the current heading
            if (!holdingHeading) { // start to hold current heading
                heldHeading = getHeading();
                holdingHeading = true;
            }
            rotationAdjustment = (float) getHeadingError(heldHeading) * -1f * .05f; // auto rotate to held heading
            rotationAdjustment = rotationAdjustment * Math.min(Math.max(Math.abs(leftX), Math.abs(leftY)), 0.7f); // make it proportional to speed
            rotationAdjustment = MathUtils.clamp(rotationAdjustment, -MAXROTATIONFACTOR,MAXROTATIONFACTOR ); // clip rotation so it doesn't obliterate translation
        }

        frontLeft = -(leftY - leftX - rotationAdjustment);
        frontRight = (-leftY - leftX - rotationAdjustment);
        backRight = (-leftY + leftX - rotationAdjustment);
        backLeft = -(leftY + leftX - rotationAdjustment);

        if (details) {

            teamUtil.telemetry.addLine("Joy X/Y: "+ leftJoyStickX+ "/"+ leftJoyStickY+ " X/Y: "+ leftX+ "/"+leftY);
            if (holdingHeading) {
                teamUtil.telemetry.addData("HOLDING:", heldHeading);
            }
        }

        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        br.setPower(backRight);
        bl.setPower(backLeft);

        String currentSpeedState;
        if(isSlow){
            currentSpeedState="Is Slow";
        }else if(isFast){
            currentSpeedState="Is Fast";
        }else{
            currentSpeedState="Is Medium";
        }
        telemetry.addLine("Current Speed State " + currentSpeedState);

        //TODO: take out soon just for testing purposes
        telemetry.addLine("Left Joystick Y: " + leftJoyStickY);
        telemetry.addLine("Left Joystick X: " + leftJoyStickX);

        telemetry.addLine("fl power: " + frontLeft);
        telemetry.addLine("fr power: " + frontRight);
        telemetry.addLine("bl power: " + backLeft);
        telemetry.addLine("br power: " + backRight);


    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Adds Field Relative driving to driveJoyStick
    public void universalDriveJoystick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast, double robotHeading) {
        double angleInRadians = robotHeading * Math.PI / 180;
        float leftX = leftJoyStickX;
        float leftY = leftJoyStickY;
        float rightX = rightJoyStickX;

        //rotate to obtain new coordinates
        float rotatedLeftX = (float) (Math.cos(angleInRadians) * leftX - Math.sin(angleInRadians) * leftY);
        float rotatedLeftY = (float) (Math.sin(angleInRadians) * leftX + Math.cos(angleInRadians) * leftY);

        driveJoyStick(rotatedLeftX, rotatedLeftY, rightX, isFast);
    }

    public void universalDriveJoystickV2(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast,boolean isSlow, double robotHeading) {
        double angleInRadians = robotHeading * Math.PI / 180;
        float leftX = leftJoyStickX;
        float leftY = leftJoyStickY;
        float rightX = rightJoyStickX;

        //rotate to obtain new coordinates
        float rotatedLeftX = (float) (Math.cos(angleInRadians) * leftX - Math.sin(angleInRadians) * leftY);
        float rotatedLeftY = (float) (Math.sin(angleInRadians) * leftX + Math.cos(angleInRadians) * leftY);

        driveJoyStickV2(rotatedLeftX, rotatedLeftY, rightX, isFast,isSlow);
    }

    public void geoFenceDriveJoystick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast, double robotHeading) {
        // check if proximity sensors are triggered
        if (Math.abs(robotHeading - 180) < 10) { // TODO: Also maybe don't do this unless output is in score position?
            if ((getLeftProximity() || getRightProximity()) && leftJoyStickX > 0) {
                leftJoyStickX = 0;
            }
        }
        universalDriveJoystick(leftJoyStickX, leftJoyStickY, rightJoyStickX, isFast, robotHeading);
    }

    public void setHeldHeading(double heading){
        holdingHeading = true;
        heldHeading = heading;
    }

}

/***************************************************************************************************************************
 *
 * OLDER CODE

public void decelerationPhase(double robotHeading, double driveHeading){
    double cruiseVelocity = lastVelocity;
    MotorData data = new MotorData();
    getDriveMotorData(data);
    setMotorsWithEncoder();
    double velocityChangeNeededDecel = cruiseVelocity - MIN_END_VELOCITY;
    double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_DECELERATION); // divide by counts per cm for distance
    double distance = getEncoderDistance(data);
    double startDecelerationDistance = distance;
    while (distance < decelerationDistance) {
        distance = getEncoderDistance(data);
        double ticsUntilEnd = decelerationDistance - distance;
        driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_DECELERATION * ticsUntilEnd + MIN_END_VELOCITY);
    }
    runMotors(0);
}
    public void oldMoveCm(double cruiseVelocity, double centimeters) {
        double startEncoderPosition = fl.getCurrentPosition();


        double velocityChangeNeededAccel = cruiseVelocity - MIN_START_VELOCITY;
        double velocityChangeNeededDecel = cruiseVelocity - MIN_END_VELOCITY;

        setMotorsWithEncoder();

        double totalTics = centimeters * COUNTS_PER_CENTIMETER;
        double ticsDuringAcceleration = velocityChangeNeededAccel / MAX_ACCELERATION;
        double ticsDuringDeceleration = velocityChangeNeededDecel / MAX_DECELERATION;
        double cruiseTics = totalTics - ticsDuringDeceleration - ticsDuringAcceleration;
        if (cruiseTics<0){
            double percentageToRemoveAccel = ticsDuringAcceleration/(ticsDuringAcceleration+ticsDuringDeceleration);
            ticsDuringDeceleration += (ticsDuringDeceleration+cruiseTics)*percentageToRemoveAccel;
            ticsDuringDeceleration += (ticsDuringAcceleration+cruiseTics)*percentageToRemoveAccel;
            cruiseTics = 0;
        }
        if (details) {
            log("Total tics: " + totalTics);
            log("Acceleration distance: " + ticsDuringAcceleration);
            log("Deceleration distance: " + ticsDuringDeceleration);
            log("Cruise length: " + cruiseTics);
        }
//acceleration
        while (fl.getCurrentPosition() < startEncoderPosition + ticsDuringAcceleration) {
            double ticsSinceStart = fl.getCurrentPosition() - startEncoderPosition;

            runMotors(MAX_ACCELERATION * ticsSinceStart + MIN_START_VELOCITY);
        }
        if (details) {

            log("distance after acceleration: " + (fl.getCurrentPosition() - startEncoderPosition));
        }
//cruise
        while (fl.getCurrentPosition() < cruiseTics + startEncoderPosition) {
            runMotors(cruiseVelocity);
        }
        if (details) {
            log("distance after cruise: " + (fl.getCurrentPosition() - startEncoderPosition));
        }

        double encoderAfterCruise = fl.getCurrentPosition();
//deceleration
        while (fl.getCurrentPosition() < startEncoderPosition + totalTics) {
            double ticsUntilEnd = ticsDuringDeceleration + fl.getCurrentPosition() - startEncoderPosition + encoderAfterCruise;

            runMotors(MAX_DECELERATION * ticsUntilEnd + MIN_END_VELOCITY);
        }
        if (details) {
            log("distance after deceleration: " + (fl.getCurrentPosition() - startEncoderPosition));
        }
    }


public void centerOnAprilTag(int id, double xOffset, double yOffset, double heading){
    boolean details = true;
    //TODO: FIX THE CURVE AHHHHHH
    boolean seesId = false;
    double maxVelocity;
    if(lastVelocity<=MIN_END_VELOCITY){
        maxVelocity = MIN_END_VELOCITY;
    }else {
        maxVelocity = lastVelocity;
    }
    double forwardsDist = 0;
    double rightDist = 0;
    double distance = 0;
    double startDistance;
    double startDecelDist = 0;
    double driveHeading = 0;
    double robotHeading = 0;
    List<AprilTagDetection> startDetections = aprilTag.getDetections();
    for(AprilTagDetection detection : startDetections){
        if(detection.id == id){
            seesId = true;
            forwardsDist = detection.ftcPose.y * CMS_PER_INCH;
            rightDist = detection.ftcPose.x*CMS_PER_INCH;
            distance = Math.sqrt(Math.pow(forwardsDist, 2)+Math.pow(rightDist, 2));
            startDistance = distance;
            startDecelDist = (maxVelocity-MIN_END_VELOCITY)/MAX_DECELERATION/COUNTS_PER_CENTIMETER;
        }
    } // TODO: do more tries
    if(seesId == false){
        if(details){
            log("did not detect apriltag");
        }
        runMotors(0);
        return;
    }
    if(details){
        log("Estimated start distance: "+distance);
        log("Deceleration distance" + startDecelDist);
        log("Forwards dist "+forwardsDist+" right dist: "+rightDist);
    }
    List<AprilTagDetection> lastDetection = null;
    while(startDecelDist<distance && distance > 4){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections != lastDetection) {
            boolean foundId= false;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == id) {
                    seesId = true;
                    forwardsDist = detection.ftcPose.y * CMS_PER_INCH;
                    rightDist = detection.ftcPose.x * CMS_PER_INCH;
                    distance = Math.sqrt(Math.pow(forwardsDist-xOffset, 2) + Math.pow(rightDist-yOffset, 2));
                    foundId = true;
                    lastDetection = currentDetections;
                    double[] list = calculateAngle(rightDist, forwardsDist, xOffset, yOffset);

                    if(rightDist<0 && forwardsDist >0){
                        driveHeading = getHeading()+list[1];
                    }else if(rightDist>0 && forwardsDist > 0){
                        driveHeading = getHeading()-list[1];
                    }else if(rightDist>0){
                        driveHeading = adjustAngle(getHeading()-180)-list[1];
                    }else{
                        driveHeading = adjustAngle(getHeading()-180)+list[1];
                    }
                    log("robot heading "+robotHeading);
                    log("list: "+list[0]+", "+list[1]);
                    log("drive heading "+driveHeading);
                    log("Forwards dist "+forwardsDist+" right dist: "+rightDist);
                    robotHeading = heading;
                    driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity);
                }
            }
            if(!foundId){
                if(details){
                    log("lost sight of apriltag");
                }
                moveCm(maxVelocity, distance, driveHeading, robotHeading, 0);
                return;
            }
        }
    }
    if(details){
        log("ended cruise");
        log("Estimated start distance: "+distance);
        log("Deceleration distance" + startDecelDist);
        log("Forwards dist "+forwardsDist+" right dist: "+rightDist);
    }
    while(distance > 4){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if(currentDetections!=lastDetection) {
            boolean foundId = false;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == id) {
                    seesId = true;
                    forwardsDist = detection.ftcPose.y * CMS_PER_INCH;
                    rightDist = detection.ftcPose.x * CMS_PER_INCH;
                    distance = Math.sqrt(Math.pow(forwardsDist-xOffset, 2) + Math.pow(rightDist-yOffset, 2));
                    foundId = true;
                    lastDetection = currentDetections;
                    double[] list = calculateAngle(rightDist, forwardsDist, xOffset, yOffset);
                    if(rightDist<0 && forwardsDist >0){
                        driveHeading = getHeading()+list[1];
                    }else if(rightDist>0 && forwardsDist > 0){
                        driveHeading = getHeading()-list[1];
                    }else if(rightDist>0){
                        driveHeading = adjustAngle(getHeading()-180)-list[1];
                    }else{
                        driveHeading = adjustAngle(getHeading()-180)+list[1];
                    }
                    log("robot heading "+robotHeading);
                    log("list: "+list[0]+", "+list[1]);
                    log("drive heading "+driveHeading);
                    log("Forwards dist "+forwardsDist+" right dist: "+rightDist);
                    robotHeading = heading;
                    double velocity = -MAX_DECELERATION*COUNTS_PER_CENTIMETER*distance+maxVelocity;
                    driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
                }
            }
            if(!foundId){
                if(details){
                    log("lost sight of apriltag");
                    log("robot heading "+robotHeading);
                    log("drive heading "+driveHeading);
                    log("Forwards dist "+forwardsDist+" right dist: "+rightDist);
                }
                moveCm(-MAX_DECELERATION*COUNTS_PER_CENTIMETER*distance+maxVelocity, distance, driveHeading, robotHeading, 0);
                return;
            }
        }

    }
    runMotors(0);
    lastVelocity = 0;
}


public void localizeWithAprilTag(int aprilTagID, double xOffset, double yOffset, double robotHeading){
    log("Localize w/AprilTag ID:" + aprilTagID);
    boolean details = true;
    MIN_END_VELOCITY = 700;
    boolean seesId = false;
    double maxVelocity;
    if(lastVelocity<=MIN_END_VELOCITY){
        maxVelocity = MIN_END_VELOCITY;
    }else {
        maxVelocity = lastVelocity;
    }
    double forwardsDist = 0;
    double rightDist = 0;
    double distance = 0;
    double startDecelDist = 0;
    double driveHeading = 0;
    List<AprilTagDetection> startDetections = aprilTag.getDetections();
    for(AprilTagDetection detection : startDetections){
        if(detection.id == aprilTagID){
            seesId = true;
            forwardsDist = detection.ftcPose.y * CMS_PER_INCH;
            rightDist = detection.ftcPose.x*CMS_PER_INCH;
            distance = Math.sqrt(Math.pow(forwardsDist, 2)+Math.pow(rightDist, 2));
            startDecelDist = (maxVelocity-MIN_END_VELOCITY)/MAX_DECELERATION/COUNTS_PER_CENTIMETER;
        }
    }
    if(seesId == false){ // TODO: do more tries?
        if(details){
            log("did not detect apriltag");
        }
        runMotors(0);
        return;
    }
    if(details){
        log("Distance to Target: "+distance);
        log("Deceleration Phase Start" + startDecelDist);
        log("Forwards dist "+forwardsDist+" right dist: "+rightDist);
    }
    List<AprilTagDetection> lastDetections = null;
    while(startDecelDist<distance && distance > 4){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections != lastDetections) { // Did we get new data?
            boolean foundId= false;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == aprilTagID) { // Did we find the one we are looking for?
                    foundId = true;
                    forwardsDist = detection.ftcPose.y * CMS_PER_INCH - yOffset;
                    rightDist = detection.ftcPose.x * CMS_PER_INCH - xOffset;
                    distance = Math.sqrt(Math.pow(forwardsDist, 2) + Math.pow(rightDist, 2));
                    lastDetections = currentDetections;
                    driveHeading = adjustAngle(-Math.toDegrees(Math.atan(rightDist/forwardsDist)));

                    log("robot heading "+robotHeading);
                    log("drive heading "+driveHeading);
                    log("Forwards:"+forwardsDist+" Right: "+rightDist+" Distance: "+distance);
                    driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity);
                }
            }
            if(!foundId){
                if(details){
                    log("lost sight of apriltag");
                }
                moveCm(maxVelocity, distance, driveHeading, robotHeading, 0);
                return;
            }
        }
    }
    if(details){
        log("ended cruise");
        log("Estimated start distance: "+distance);
        log("Deceleration distance" + startDecelDist);
        log("Forwards dist "+forwardsDist+" right dist: "+rightDist);
    }

        while(distance > 4){
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if(currentDetections!=lastDetection) {
                boolean foundId = false;
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == id) {
                        seesId = true;
                        forwardsDist = detection.ftcPose.y * CMS_PER_INCH;
                        rightDist = detection.ftcPose.x * CMS_PER_INCH;
                        distance = Math.sqrt(Math.pow(forwardsDist-xOffset, 2) + Math.pow(rightDist-yOffset, 2));
                        foundId = true;
                        lastDetection = currentDetections;
                        double[] list = calculateAngle(rightDist, forwardsDist, xOffset, yOffset);
                        if(rightDist<0 && forwardsDist >0){
                            driveHeading = getHeading()+list[1];
                        }else if(rightDist>0 && forwardsDist > 0){
                            driveHeading = getHeading()-list[1];
                        }else if(rightDist>0){
                            driveHeading = adjustAngle(getHeading()-180)-list[1];
                        }else{
                            driveHeading = adjustAngle(getHeading()-180)+list[1];
                        }
                        log("robot heading "+robotHeading);
                        log("list: "+list[0]+", "+list[1]);
                        log("drive heading "+driveHeading);
                        log("Forwards dist "+forwardsDist+" right dist: "+rightDist);
                        robotHeading = heading;
                        double velocity = -MAX_DECELERATION*COUNTS_PER_CENTIMETER*distance+maxVelocity;
                        driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
                    }
                }
                if(!foundId){
                    if(details){
                        log("lost sight of apriltag");
                        log("robot heading "+robotHeading);
                        log("drive heading "+driveHeading);
                        log("Forwards dist "+forwardsDist+" right dist: "+rightDist);
                    }
                    moveCm(-MAX_DECELERATION*COUNTS_PER_CENTIMETER*distance+maxVelocity, distance, driveHeading, robotHeading, 0);
                    return;
                }
            }


    runMotors(0);
    lastVelocity = 0;
}


public void correctAndHug(int aprilTagID, double robotHeading) {
    log("CorrectAndHub w/AprilTag ID:" + aprilTagID);
    // Robot is currently traveling forward at velocity 500!
    boolean details = true;
    boolean seesId = false;

    // Get a reading from the AprilTag if we can
    List<AprilTagDetection> startDetections = aprilTag.getDetections();
    for(AprilTagDetection detection : startDetections){
        if(detection.id == aprilTagID){
            seesId = true;
            double xFix = detection.ftcPose.x*CMS_PER_INCH+1; // offset 1cm for robot build
            double cmsToGoAt45 = Math.sqrt(Math.pow(xFix, 2)*2);
            if (details) {
                log ("xFix: " + xFix);
                log("Correcting cms at 45: "+ cmsToGoAt45);
            }
            moveCm(500, cmsToGoAt45, xFix < 0 ? 45 : -45, robotHeading,500);
            break;
        }
    }
    if(seesId == false){ // TODO: Need a better failsafe?
        if(details){
            log("did not detect apriltag");
        }
        runMotors(0);
        return;
    }
    // Now squish to wall
    driveMotorsHeadingsFR(0,robotHeading,500);
    teamUtil.pause(500);
    setMotorsFloat(); // coast to wall
    runMotors(0);
    log("CorrectAndHub - Finished");
}
 */
// drive until either sensor sees some tape or we time out.
    // Returns true if it was successful, false if it timed out
    // Does NOT stop motors at end!
    /*
    public boolean driveToTape(double driveHeading, double robotHeading, double velocity, long timeout) {
        teamUtil.log("Drive To Tape");
        long timeOutTime = System.currentTimeMillis() + timeout;
        if (tapeSensor1.isOnTape()) {
            teamUtil.log("Drive To Tape-Saw 1, looking for 2");
            while (teamUtil.keepGoing(timeOutTime) && !tapeSensor2.isOnTape()) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        } else if (tapeSensor2.isOnTape()) {
            teamUtil.log("Drive To Tape-Saw 2, looking for 1");
            while (teamUtil.keepGoing(timeOutTime) && !tapeSensor1.isOnTape()) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        } else {
            teamUtil.log("Drive To Tape-Looking for either");
            while (teamUtil.keepGoing(timeOutTime) && !tapeSensor1.isOnTape() && !tapeSensor2.isOnTape()) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        }
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("Drive To Tape-TIMED OUT!");
        } else {
            teamUtil.log("Drive To Tape-Finished");
        }

        return System.currentTimeMillis() < timeOutTime;
    }

    public boolean driveToTapeSetPower(float power, long timeout) {
        teamUtil.log("Drive To Tape");
        long timeOutTime = System.currentTimeMillis() + timeout;
        setMotorPower(-power);
        while (teamUtil.keepGoing(timeOutTime) && !tapeSensor1.isOnTape() && !tapeSensor2.isOnTape()) {

        }
        setMotorPower(0);
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("Drive To Tape-TIMED OUT!");
        } else {
            teamUtil.log("Drive To Tape-Finished");
        }

        return System.currentTimeMillis() < timeOutTime;
    }

    // drive until either color sensor is triggered, OR we are interrupted, OR we time out.
    // Returns true if it was successful, false if it timed out
    // Does NOT stop motors at end!
    public boolean driveToTapeTelop(double driveHeading, double robotHeading, double velocity, long timeout) {
        teamUtil.log("Drive To Tape");
        long timeOutTime = System.currentTimeMillis() + timeout;
        while ((teamUtil.keepGoing(timeOutTime) && !manualInterrupt.get()) && !tapeSensor1.isOnTape() && !tapeSensor2.isOnTape()) {
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            //teamUtil.log("Closing: " + manualInterrupt.get() + " "+ getLeftProximity()+"/"+ getRightProximity());
        }
        stopMotors();
        //setMotorsActiveBrake(); 
        //teamUtil.pause(500);
        setMotorsWithEncoder();
        movingAutonomously.set(false);
        return System.currentTimeMillis() < timeOutTime;
    }

    public void driveToTapeTelopNoWait(double driveHeading, double robotHeading, double velocity, long timeout) {
        if (movingAutonomously.get()) { // Already in an autonomous operation
            teamUtil.log("WARNING: Attempt to driveToTape while drive system is in autonomous operation--ignored");
            return;
        } else {
            movingAutonomously.set(true); // signal that we are in an auto operation
            manualInterrupt.set(false); // reset interrupt flag
            teamUtil.log("Launching Thread to driveToTape");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    driveToTapeTelop(driveHeading, robotHeading, velocity, timeout);
                }
            });
            thread.start();
        }
    }

     */
/*
        teamUtil.log("Find Yellow Pixel Called");
        //Both Processor Running on Rear Cam
        List<AprilTagDetection> detections = aprilTag.getDetections();
        float offsetTotal = 0;
        int numTags = 0;
        double aprilTagMidpoint=0;
        double aprilTagDetectionNanoTime=0;

        for (AprilTagDetection detection : detections) { // Average whatever readings we have
            if ((path==1&&(detection.id == 1 || detection.id == 4))||
                    (path==2&&(detection.id == 2 || detection.id == 5))||
                    (path==3&&(detection.id == 3 || detection.id == 6))){
                aprilTagMidpoint = detection.center.x;
                aprilTagDetectionNanoTime = detection.frameAcquisitionNanoTime;
            }
        }
        if(aprilTagMidpoint==0){
            teamUtil.log("Find Yellow Pixel Failed Because No April Tag Detected");
            return YellowPixelPosition.FAILED;
        }
        teamUtil.log("April Tag Midpoint"+aprilTagMidpoint);

        if(findPixelProcesser.foundPixel.get()){
            if(findPixelProcesser.getMidpoint()<aprilTagMidpoint){
                return YellowPixelPosition.LEFT;
            }else{
                return YellowPixelPosition.RIGHT;
            }
        }else{
            teamUtil.log("No Yellow Pixel Detected");

            return YellowPixelPosition.NONE;
        }
*/