package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.bottomColorSensor;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.teamcode.testCode.calibrateCV;
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
    public bottomColorSensor tapeSensor1, tapeSensor2;
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
    public AnalogInput ultLeft = null;
    public DigitalChannel prxLeft = null, prxRight = null;

    // Image Processors and Cameras

    public VisionPortal rearVisionPortal, sideVisionPortal, frontVisionPortal;

    public AprilTagProcessor aprilTag;
    public static int aprilTagExposure = 3; // frame exposure in ms (use TestDrive opMode to calibrate)
    public static int aprilTagGain = 255; // High gain to compensate for fast exposure  DOESN'T WORK DUE TO FTC BUG
    public OpenCVFindLine findLineProcesser;
    //public OpenCVYellowPixelDetector findPixelProcesser;

    public OpenCVPropFinder findTeamPropProcesser;



    public double noAprilTag = 999.0;
    public float TAG_CENTER_TO_CENTER = 15.2f;

    public enum cvCam {NONE, REAR_APRILTAG, SIDE_PROP, FRONT_LINE};

    public Drive.cvCam currentCam = Drive.cvCam.NONE;




    public WebcamName frontCam;
    public WebcamName backCam;
    public WebcamName sideCam;

    public double COUNTS_PER_MOTOR_REV = 537.7;    // GoBilda 5202 312 RPM
    public double COUNTS_PER_CENTIMETER = 12.855; // Used with 435 RPM
    public double COUNTS_PER_CENTIMETER_312 = 17.923; // Used with 312 RPM


    public double MIN_START_VELOCITY = 300; //calibrated with 435s
    public double MIN_END_VELOCITY = 400; //calibrated with 435s
    public double MAX_ACCELERATION = 22; //calibrated with 435s
    public double MAX_DECELERATION = 1.0; //calibrated with 435s
    public double MAX_VELOCITY = 2450; // Was 2680
    public double MAX_VELOCITY_STRAFE = 2000; // Added with new motors
    public double ROTATION_ADJUST_FACTOR = 0.04;
    public double SIDE_VECTOR_COEFFICIENT = .92;
    public double FORWARD_VECTOR_COEFFICIENT = 1.08;
    public double SPIN_END_OF_MAX_VELOCITY = 60;
    public double DRIFT_SPINS = 3.5;
    public double CRAWL_SPEED = 200;
    public double CRAWL_DISTANCE_SPINS = 30;
    public boolean details = true;
    public double CMS_PER_INCH = 2.54;
    public double TICS_PER_CM_STRAFE_ENCODER = 130;

    public Output output;

    public Drive() {
        teamUtil.log("Constructing Drive");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize(Output theOutput) {
        teamUtil.log("Initializing Drive");
        //Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        output=theOutput;
        fl = hardwareMap.get(DcMotorEx.class, "flm");
        fr = hardwareMap.get(DcMotorEx.class, "frm");
        bl = hardwareMap.get(DcMotorEx.class, "blm");
        br = hardwareMap.get(DcMotorEx.class, "brm");
        strafeEncoder = hardwareMap.get(DcMotorEx.class, "liftAndStrafeEncoder");
        ultLeft = hardwareMap.analogInput.get("ult");
        prxLeft = hardwareMap.get(DigitalChannel.class, "prx_left");
        prxRight = hardwareMap.get(DigitalChannel.class, "prx_right");
        prxLeft.setMode(DigitalChannel.Mode.INPUT);
        prxRight.setMode(DigitalChannel.Mode.INPUT);

        // colorSensor.calibrate();
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        tapeSensor1 = new bottomColorSensor(hardwareMap.get(RevColorSensorV3.class, "bottomColor1"));
        tapeSensor2 = new bottomColorSensor(hardwareMap.get(RevColorSensorV3.class, "bottomColor2"));
        tapeSensor1.calibrate();
        tapeSensor2.calibrate();

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

        teamUtil.log("Updating Exposure Settings");
        switchCV(cvCam.REAR_APRILTAG);
        updateCVManualExposure(rearVisionPortal,aprilTagExposure,aprilTagGain);

        switchCV(cvCam.FRONT_LINE);
        updateCVManualExposure(frontVisionPortal, findLineProcesser.lineExposure, findLineProcesser.lineGain);
        switchCV(cvCam.NONE);


        currentCam = Drive.cvCam.NONE;

        teamUtil.log("Initializing Drive CV - FINISHED");
    }



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
            while (!teamUtil.theOpMode.isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                teamUtil.pause(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
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
        teamUtil.log("Switching CV to " + newCam);
        teamUtil.log("Stopping " + currentCam);
        switch (currentCam) {
            case REAR_APRILTAG:
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
        }
        teamUtil.log("Starting " + newCam);
        switch (newCam) {
            case REAR_APRILTAG:
                rearVisionPortal.resumeStreaming();
                break;
            case FRONT_LINE:
                frontVisionPortal.resumeStreaming();
                break;
            case SIDE_PROP:
                sideVisionPortal.resumeStreaming();
                break;
        }
        currentCam = newCam;
        teamUtil.log("switchCV - Finished ");
    }

//////////////////////



    public double getUltrasonicDistance() {
        double voltage = ultLeft.getVoltage();
        double distance = (260 / 3 * (voltage - .55) + 36) * 2.54; // based on real world distances measured
        return distance;
    }

    public boolean getProximity(boolean left) {
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
        telemetry.addData("Drive ", "flm:%d frm:%d blm:%d brm:%d strafe:%d heading:%f ",
                fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition(), strafeEncoder.getCurrentPosition(), getHeading());
    }

    public void sensorTelemetry() {
        telemetry.addData("On Line:", " %b/%b", tapeSensor1.isOnTape(), tapeSensor2.isOnTape());
        telemetry.addData("Red Value: ", "%d/%d", tapeSensor1.redValue(), tapeSensor2.redValue());
        telemetry.addData("Blue Value: ", "%d/%d", tapeSensor1.blueValue(), tapeSensor2.blueValue());

        telemetry.addData("UltrasonicLeft Distance: ", "%.1f", getUltrasonicDistance());
        telemetry.addLine("Right proximity sensor: " + getProximity(false));
        telemetry.addLine("Left proximity sensor: " + getProximity(true));
    }

    public void visionTelemetry() {
        if (currentCam==cvCam.REAR_APRILTAG) {
            //findPixelProcesser.outputTelemetry();
            //telemetry.addLine("BackDrop Offset: " + getRobotBackdropXOffset());
            telemetry.addLine("RearCam:" + rearVisionPortal.getCameraState() + " FPS:" + rearVisionPortal.getFps());
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addLine("AprilTag: " + detection.id + " X:" + detection.ftcPose.x * CMS_PER_INCH + " Y:" + detection.ftcPose.y * CMS_PER_INCH);
            }
        } else if (currentCam==cvCam.FRONT_LINE) {
            telemetry.addLine("FrontCam:" + frontVisionPortal.getCameraState() + " FPS:" + frontVisionPortal.getFps());
            findLineProcesser.outputTelemetry();
        } else if (currentCam==cvCam.SIDE_PROP) {
            telemetry.addLine("SideCam:" + sideVisionPortal.getCameraState() + " FPS:" + sideVisionPortal.getFps());
            findTeamPropProcesser.outputTelemetry();
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
    // Will rotate robot as needed to achieve and hold robotHeading RELATIVE TO FIELD
    public void driveMotorsHeadingsFR(double driveHeading, double robotHeading, double velocity) {
        double RRDriveHeading = getHeadingError(driveHeading);
        driveMotorsHeadings(RRDriveHeading, robotHeading, velocity);
    }

    public double getHeadingError(double targetAngle) {

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
        int eFL, eFR, eBL, eBR;
    }

    public void getDriveMotorData(MotorData data) {
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
        moveCm(MAX_VELOCITY, centimeters, driveHeading, getHeading(), MIN_END_VELOCITY);
    }

    public void moveCm(double centimeters, double driveHeading, double endVelocity) {
        moveCm(MAX_VELOCITY, centimeters, driveHeading, getHeading(), endVelocity);
    }

    public void moveCm(double maxVelocity, double centimeters, double driveHeading, double endVelocity) {
        moveCm(maxVelocity, centimeters, driveHeading, getHeading(), endVelocity);
    }

    public void moveCm(double maxVelocity, double centimeters, double driveHeading, double robotHeading, double endVelocity) {
        teamUtil.log("MoveCM cms:" + centimeters + " driveH:" + driveHeading + " robotH:" + robotHeading + " MaxV:" + maxVelocity + " EndV:" + endVelocity);

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
            distance = getEncoderDistance(data);
            if (lastVelocity == 0) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_ACCELERATION * distance + MIN_START_VELOCITY);
            } else {
                driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_ACCELERATION * distance + lastVelocity);
            }
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distance);
        }
//cruise
        while (distance < postCruiseDistance) {

            distance = getEncoderDistance(data);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity);
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
            driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_DECELERATION * ticsUntilEnd + endVelocity);

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

    public void moveStraightCmWithStrafeEncoder(double maxVelocity, double centimeters, int strafeTarget, double driveHeading, double robotHeading, double endVelocity) {
        teamUtil.log("Strafe Target" + strafeTarget);
        teamUtil.log("Strafe Start Value" + strafeEncoder.getCurrentPosition());

        teamUtil.log("MoveStraightCMwStrafeEnc cms:" + centimeters + " strafe:" + strafeTarget + " driveH:" + driveHeading + " robotH:" + robotHeading + " MaxV:" + maxVelocity + " EndV:" + endVelocity);

        float strafeFactor = .04f; // convert strafe encoder error into heading declination
        float maxHeadingDeclination = 20f; // don't veer off of straight more than this number of degrees
        float headingFactor = Math.abs(driveHeading-180)<.01 ? 1 : -1; // reverse correction for going backwards

        details = true;
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
                output.goToScoreNoWait(4,output.GrabberRotatorHorizontal2);
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

    /************************************************************************************************************
     /************************************************************************************************************
     /************************************************************************************************************
     *
     * Methods to turn the robot in place
     *
     /************************************************************************************************************
     /************************************************************************************************************/
    public void spinToHeading(double heading) {
        double velocity = MAX_VELOCITY;
        boolean turningLeft;
        double startHeading = getHeading();
        double currentHeading = getHeading();
        double leftCoefficient = 1;
        double rightCoefficient = 1;
        setMotorsWithEncoder();
        if (heading > currentHeading) {
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

    // drive until either sensor sees some tape or we time out.
    // Returns true if it was successful, false if it timed out
    // Does NOT stop motors at end!
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
        double cmsToStack = ((float) pixelsToClose / (float) visiblePixels) * (40.5 - 32.5) + 32.5; // Assume Camera view is linear.  Might not be

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

    public void frontLineCameraDimensionTelemetry(){
        if (currentCam==cvCam.FRONT_LINE) {
            float horizontalCmsPerPixel = 20f/(581f-findLineProcesser.MIDPOINTTARGET);
            float horizontalDistanceToStackPixels =Math.abs(findLineProcesser.lastValidMidPoint.get() - findLineProcesser.MIDPOINTTARGET);
            float horizontalDistanceToStackCms =horizontalDistanceToStackPixels*horizontalCmsPerPixel;

            telemetry.addLine("Horizontal Data");
            telemetry.addLine("Horizontal Last Valid Midpoint: " + findLineProcesser.lastValidMidPoint.get());
            telemetry.addLine("Horizontal Distance To Stack Pixels: " + horizontalDistanceToStackPixels);
            telemetry.addLine("Horizontal Distance To Stack Cms: " + horizontalDistanceToStackCms);

            int visiblePixels = findLineProcesser.CAMHEIGHT - findLineProcesser.cropRect.height;
            int pixelsToClose = findLineProcesser.CAMHEIGHT - findLineProcesser.lastValidBottom.get();
            double cmsToStack = ((float) pixelsToClose / (float) visiblePixels) * (40.5 - 32.5) + 32.5; // Assume Camera view is linear.  Might not be

            telemetry.addLine("Vertical Data");
            telemetry.addLine("Cms to Stack: " + cmsToStack);
        } else{
            teamUtil.log("Front camera not currently running");
        }
    }

    public double[] calculateAngle(double rightDist, double forwardsDist, double xOffset, double yOffset) {
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
    public boolean getRobotBackdropOffset(org.opencv.core.Point p,boolean freshDetection) {
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
                teamUtil.log("no fresh detections");
                return false;
            }
            float xOffsetTotal = 0, yOffsetTotal = 0;
            int numTags = 0;
            for (AprilTagDetection detection : detections) { // Average whatever readings we have
                if (detection.ftcPose == null) {
                    // We shouldn't be here, but there might be a bug in the FTC code which allows this to happen
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
                return true;
            } else {
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

    public boolean strafeToEncoder(double driveHeading, double robotHeading, double velocity, double targetEncoderValue, long timeout) {
        long timeOutTime = System.currentTimeMillis() + timeout;
        teamUtil.log("strafeToEncoder: Current: " + strafeEncoder.getCurrentPosition() + " Target: " + targetEncoderValue);
        float driftCms = 1;
        while (Math.abs(targetEncoderValue - strafeEncoder.getCurrentPosition()) > driftCms * TICS_PER_CM_STRAFE_ENCODER && teamUtil.keepGoing(timeOutTime)) {
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
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
