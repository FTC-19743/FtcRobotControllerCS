package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.bottomColorSensor;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
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
    public DigitalChannel prxLeft  = null, prxRight  = null;

    // Image Processors and Cameras
    public AprilTagProcessor aprilTag;
    public OpenCVFindLine findLineProcesser;
    //public OpenCVYellowPixelDetector findPixelProcesser;

    public OpenCVPropFinder findTeamPropProcesser;
    public VisionPortal visionPortal;

    public double noAprilTag = 999.0;
    public float TAG_CENTER_TO_CENTER = 15.2f;


    boolean aprilTagProcessorRunning = false;
    boolean findLineProcessorRunning = false;
    boolean findTeamPropProcessorRunning = false;
    public WebcamName frontCam;
    public WebcamName backCam;
    public WebcamName sideCam;

    public double COUNTS_PER_MOTOR_REV = 537.7;    // GoBilda 5202 312 RPM
    public double COUNTS_PER_CENTIMETER = 17.923;
    public double MIN_START_VELOCITY = 350; //tentative value
    public double MIN_END_VELOCITY = 250; //tentative value
    public double MAX_ACCELERATION = 12; //tentative value
    public double MAX_DECELERATION = 2; //tentative value (should be negative)
    public double MAX_VELOCITY = 2680;
    public double ROTATION_ADJUST_FACTOR = 0.04;
    public double SIDE_VECTOR_COEFFICIENT = .92;
    public double FORWARD_VECTOR_COEFFICIENT = 1.08;
    public double SPIN_END_OF_MAX_VELOCITY = 60;
    public double DRIFT_SPINS = 3.5;
    public double CRAWL_SPEED = 200;
    public double CRAWL_DISTANCE_SPINS = 30;
    public boolean details = true;
    public double VELOCITY_DECREASE_PER_CM = 10;
    public double CMS_PER_INCH = 2.54;
    public Drive() {
        teamUtil.log("Constructing Drive");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public void initalize() {
        teamUtil.log("Initializing Drive");
        //Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        fl = hardwareMap.get(DcMotorEx.class, "flm");
        fr = hardwareMap.get(DcMotorEx.class, "frm");
        bl = hardwareMap.get(DcMotorEx.class, "blm");
        br = hardwareMap.get(DcMotorEx.class, "brm");
        strafeEncoder = hardwareMap.get(DcMotorEx.class, "strafeEncoder");
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

    public void initCV() {
        teamUtil.log("Initializing Drive CV");

        // Setup a single VisionPortal with all cameras and all processers
        // We will switch betwen cameras and processers as needed
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

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(backCam, frontCam, sideCam);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTag)
                //.addProcessor(findPixelProcesser)
                .addProcessor(findLineProcesser)
                .addProcessor(findTeamPropProcesser)
                .build();

        teamUtil.pause(2000); // give initial camera a chance to get going

        teamUtil.log("Initializing Drive CV - FINISHED");
    }
    public void runRearAprilTagProcessor() {
        visionPortal.setActiveCamera(backCam);
        visionPortal.setProcessorEnabled(findLineProcesser,false);
        findLineProcessorRunning=false;
        visionPortal.setProcessorEnabled(findTeamPropProcesser,false);
        findTeamPropProcessorRunning=false;
        visionPortal.setProcessorEnabled(aprilTag,true);
        //visionPortal.setProcessorEnabled(findPixelProcesser,true);
        aprilTagProcessorRunning=true;
    }
    public void runFrontLineFinderProcessor() {
        visionPortal.setActiveCamera(frontCam);
        visionPortal.setProcessorEnabled(aprilTag ,false);
        //visionPortal.setProcessorEnabled(findPixelProcesser,false);
        aprilTagProcessorRunning=false;
        visionPortal.setProcessorEnabled(findTeamPropProcesser,false);
        findTeamPropProcessorRunning=false;
        visionPortal.setProcessorEnabled(findLineProcesser,true);
        findLineProcessorRunning=true;
    }
    public void runSideTeamPropFinderProcessor() {
        visionPortal.setActiveCamera(sideCam);
        visionPortal.setProcessorEnabled(aprilTag ,false);
        //visionPortal.setProcessorEnabled(findPixelProcesser,false);
        aprilTagProcessorRunning=false;
        visionPortal.setProcessorEnabled(findLineProcesser,false);
        findLineProcessorRunning=false;
        visionPortal.setProcessorEnabled(findTeamPropProcesser,true);
        findTeamPropProcessorRunning=true;
    }

    public double getUltrasonicDistance(){
        double voltage = ultLeft.getVoltage();
        double distance = (260/3*(voltage-.55)+36)*2.54; // based on real world distances measured
        return distance;
    }
    public boolean getProximity(boolean left){
        if(!left){
            return !prxRight.getState();
        }else{
            return !prxLeft.getState();
        }
    }
    public boolean getLeftProximity(){
        return !prxLeft.getState();
    }
    public boolean getRightProximity(){
        return !prxRight.getState();
    }
    public void driveMotorTelemetry() {
        telemetry.addData("Drive ", "flm:%d frm:%d blm:%d brm:%d strafe:%d heading:%f ",
                fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition(), strafeEncoder.getCurrentPosition(), getHeading());
    }
    public void sensorTelemetry() {
        telemetry.addData("On Line:"," %b/%b", tapeSensor1.isOnTape(), tapeSensor2.isOnTape());
        telemetry.addData("Red Value: ", "%d/%d", tapeSensor1.redValue(), tapeSensor2.redValue());
        telemetry.addData("Blue Value: ", "%d/%d", tapeSensor1.blueValue(), tapeSensor2.blueValue());

        telemetry.addData("UltrasonicLeft Distance: ", "%.1f", getUltrasonicDistance());
        telemetry.addLine("Right proximity sensor: "+getProximity(false));
        telemetry.addLine("Left proximity sensor: "+getProximity(true));
    }
    public void visionTelemetry () {
        if (aprilTagProcessorRunning) {
            //findPixelProcesser.outputTelemetry();
            telemetry.addLine("BackDrop Offset: " + getRobotBackdropXOffset());
            telemetry.addLine("RearCam:" + visionPortal.getCameraState()+ " FPS:" + visionPortal.getFps());
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addLine("AprilTag: " + detection.id + " X:" + detection.ftcPose.x * CMS_PER_INCH + " Y:" + detection.ftcPose.y * CMS_PER_INCH);
            }
        } else if (findLineProcessorRunning) {
            telemetry.addLine("FrontCam:" + visionPortal.getCameraState()+ " FPS:" + visionPortal.getFps());
            findLineProcesser.outputTelemetry();
        }
        else if (findTeamPropProcessorRunning) {
            telemetry.addLine("SideCam:" + visionPortal.getCameraState()+ " FPS:" + visionPortal.getFps());
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

    public void setMotorsBrake(){
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setMotorsFloat(){
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    public void setMotorPower(double power){
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

    }

    public void setMotorsActiveBrake(){
        int flPosition = fl.getCurrentPosition();
        int frPosition = fr.getCurrentPosition();
        int blPosition = bl.getCurrentPosition();
        int brPosition =  br.getCurrentPosition();
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

    public void setMotorsRunToPosition(){
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

    public void findMaxVelocity() {
        teamUtil.log("Finding Forward Max Velocities...");
        resetAllDriveEncoders();
        double travelTics = COUNTS_PER_CENTIMETER * 120;
        setMotorVelocities(3000, 3000, 3000, 3000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0, v;
        while (fr.getCurrentPosition() < travelTics) {
            flmax = (v = fl.getVelocity()) > flmax ? v : flmax;
            frmax = (v = fr.getVelocity()) > frmax ? v : frmax;
            blmax = (v = bl.getVelocity()) > blmax ? v : blmax;
            brmax = (v = br.getVelocity()) > brmax ? v : brmax;
            teamUtil.log("Looping FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);
        }
        stopMotors();
        teamUtil.log("Forward Max Velocities FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);
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
        log("fr: " + fr.getCurrentPosition());
        log("fl: " + fl.getCurrentPosition());
        log("br: " + br.getCurrentPosition());
        log("bl: " + bl.getCurrentPosition());
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
    public void moveCm(double centimeters, double driveHeading, double endVelocity){
        moveCm(MAX_VELOCITY, centimeters, driveHeading, getHeading(), endVelocity);
    }

    public void moveCm(double maxVelocity, double centimeters, double driveHeading, double endVelocity) {
        moveCm(maxVelocity, centimeters, driveHeading, getHeading(), endVelocity);
    }
    public void moveCm(double maxVelocity, double centimeters, double driveHeading, double robotHeading, double endVelocity){
        log("MoveCM cms:" + centimeters + " driveH:"+ driveHeading+" robotH:"+robotHeading+" MaxV:"+maxVelocity+" EndV:"+endVelocity);

        details=false;
        MotorData data = new MotorData();
        getDriveMotorData(data);

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if(endVelocity < MIN_END_VELOCITY){
            endVelocity = MIN_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }else{
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        setMotorsWithEncoder();
        // all are measured in tics
        double totalTics = centimeters * COUNTS_PER_CENTIMETER;
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_DECELERATION);
        double postCruiseDistance = totalTics - decelerationDistance;
        if (postCruiseDistance<0){
            double percentageToRemoveAccel = accelerationDistance/(accelerationDistance+decelerationDistance);
            accelerationDistance += postCruiseDistance*percentageToRemoveAccel;
            decelerationDistance += postCruiseDistance*percentageToRemoveAccel;
            postCruiseDistance = 0;
        }
        double distance = 0;
        if (details) {
            log("Heading:" + getHeading());
            log("Total tics: " + totalTics);
            log("Acceleration distance: " + accelerationDistance);
            log("Deceleration distance: " + decelerationDistance);
            log("Post Cruise distance: " + postCruiseDistance);
        }
//acceleration
        while (distance < accelerationDistance) {
            distance = getEncoderDistance(data);
            if (lastVelocity == 0) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_ACCELERATION * distance + MIN_START_VELOCITY);
            }else{
                driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_ACCELERATION * distance + lastVelocity);
            }
        }
        if (details) {
            log("Heading:" + getHeading());
            log("distance after acceleration: " + distance);
        }
//cruise
        while (distance < postCruiseDistance) {

            distance = getEncoderDistance(data);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity);
        }
        if (details) {
            log("Heading:" + getHeading());
            log("distance after cruise: " + distance);
        }


//deceleration
        double startDecelerationDistance = distance;
        double ticsUntilEnd = totalTics-distance;
        while (distance < totalTics) {
            distance = getEncoderDistance(data);
            ticsUntilEnd = totalTics-distance;
            driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_DECELERATION * ticsUntilEnd + endVelocity);

        }
        if (details) {
            log("distance after deceleration: " + distance);
        }
        if(endVelocity <= MIN_END_VELOCITY){
            runMotors(0);
            if(details) {
                log("Went below or was min end velocity");
            }
        }
        lastVelocity = endVelocity;
        log("MoveCM--Finished");

    }



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
            log("turning left: " + rightCoefficient);
            log("current heading: " + currentHeading);
            log("heading goal: " + (heading + DRIFT_SPINS));
        }
        if (details) {
            log("crossing 0/360 barrier");
        }
        while (Math.abs(currentHeading - heading)>SPIN_END_OF_MAX_VELOCITY) {
            setMotorVelocities(leftCoefficient*velocity, rightCoefficient*velocity, leftCoefficient*velocity, rightCoefficient*velocity);
            currentHeading = getHeading();
        }
        if (details) {
            log("current heading: " + currentHeading);
            log("heading cutoff (greater): " + adjustAngle(heading - CRAWL_DISTANCE_SPINS));
            log("done with max velocity phase");
            log("heading: " + currentHeading);
        }
        while (Math.abs(currentHeading-heading) > CRAWL_DISTANCE_SPINS) {
            currentHeading = getHeading();
            velocity = ((MAX_VELOCITY - CRAWL_SPEED) / (SPIN_END_OF_MAX_VELOCITY - CRAWL_DISTANCE_SPINS)) * (Math.abs(currentHeading - heading) - SPIN_END_OF_MAX_VELOCITY) + MAX_VELOCITY; // wrote an equasion
            if (velocity < CRAWL_SPEED) {
                velocity = CRAWL_SPEED;
            }
            setMotorVelocities(leftCoefficient*velocity, rightCoefficient*velocity, leftCoefficient*velocity, rightCoefficient*velocity);
        }

        if (details) {
            log("done with deceleration phase");
            log("heading: " + currentHeading);
        }
        while (Math.abs(currentHeading-heading) > DRIFT_SPINS) {
            currentHeading = getHeading();
            velocity = CRAWL_SPEED;
            setMotorVelocities(leftCoefficient*velocity, rightCoefficient*velocity, leftCoefficient*velocity, rightCoefficient*velocity);
        }

        if (details) {
            log("done with crawl phase");
            log("heading: " + currentHeading);
        }

        setMotorsBrake();
        setMotorPower(0);
    }

    public void stopAtUltDistance(double distanceFromWall, double robotHeading, double driveHeading){
        boolean details = true;
        MotorData data = new MotorData();
        getDriveMotorData(data);
        setMotorsWithEncoder();
        distanceFromWall = distanceFromWall+15; // adjust for moving average computed by sensor

        double distance = getUltrasonicDistance();

        if(details){
            log("starting distance: "+distance);
            log("goal distance: "+distanceFromWall);
        }
        if(distance <= distanceFromWall){
            if (details) {
                log("already within distance");
            }
            return;
        }
        while(distanceFromWall<distance){
            //log("in while loop");
            distance = getUltrasonicDistance();
            log("distance: "+distance);
            //double velocity = MIN_END_VELOCITY+Math.abs(distance-distanceFromWall)*VELOCITY_DECREASE_PER_CM;
            //log("Velocity: "+velocity);
            driveMotorsHeadingsFR(driveHeading,robotHeading, 500); // TODO: Tested at 500, could be faster?
        }
        if(details){
            log("distance before pause: "+distance);
        }
        stopMotors();
        setMotorsBrake();

        teamUtil.pause(1000);

        distance = getUltrasonicDistance();
        if(details){
            log("distance at end: "+distance);
        }
    }

    // drive until either sensor sees some tape or we time out.
    // Returns true if it was successful, false if it timed out
    // Does NOT stop motors at end!
    public boolean driveToTape(double driveHeading, double robotHeading, double velocity, long timeout ) {
        log ("Drive To Tape");
        long timeOutTime = System.currentTimeMillis() + timeout;
        if(tapeSensor1.isOnTape()){
            log ("Drive To Tape-Saw 1, looking for 2");
            while (teamUtil.keepGoing(timeOutTime) && !tapeSensor2.isOnTape()) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        }else if(tapeSensor2.isOnTape()){
            log ("Drive To Tape-Saw 2, looking for 1");
            while (teamUtil.keepGoing(timeOutTime) && !tapeSensor1.isOnTape()) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        }else {
            log ("Drive To Tape-Looking for either");
            while (teamUtil.keepGoing(timeOutTime) && !tapeSensor1.isOnTape() && !tapeSensor2.isOnTape() ) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        }
        if (System.currentTimeMillis()> timeOutTime) {
            log("Drive To Tape-TIMED OUT!");
        } else {
            log("Drive To Tape-Finished");
        }

        return System.currentTimeMillis() < timeOutTime;
    }

    // drive until either color sensor is triggered, OR we are interrupted, OR we time out.
    // Returns true if it was successful, false if it timed out
    // Does NOT stop motors at end!
    public boolean driveToTapeTelop(double driveHeading, double robotHeading, double velocity, long timeout ) {
        log ("Drive To Tape");
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
    public void driveToTapeTelopNoWait(double driveHeading, double robotHeading, double velocity, long timeout ) {
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
                    driveToTapeTelop( driveHeading,  robotHeading,  velocity,  timeout );
                }
            });
            thread.start();
        }
    }
    
    
    // drive until either rear proximity sensor is triggered, OR we are interrupted, OR we time out.
    // Returns true if it was successful, false if it timed out
    // Does NOT stop motors at end!
    public boolean driveToProximity(double driveHeading, double robotHeading, double velocity, long timeout ) {
        log ("Drive To Proximity");
        long timeOutTime = System.currentTimeMillis() + timeout;
        while ((teamUtil.keepGoing(timeOutTime) && !manualInterrupt.get()) && !getLeftProximity() && !getRightProximity()) {
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            teamUtil.log("Closing: " + manualInterrupt.get() + " "+ getLeftProximity()+"/"+ getRightProximity());
        }
        movingAutonomously.set(false);
        return System.currentTimeMillis() < timeOutTime;
    }

    public void driveToProximityNoWait(double driveHeading, double robotHeading, double velocity, long timeout ) {
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
                    driveToProximity( driveHeading,  robotHeading,  velocity,  timeout );
                }
            });
            thread.start();
        }
    }

    public double[] calculateAngle(double rightDist, double forwardsDist, double xOffset, double yOffset){
        int quadrant; // the quad the goal point would be in if the current spot was the origin
        if(rightDist<xOffset && forwardsDist > yOffset){
            quadrant = 1;
        }else if(rightDist > xOffset && forwardsDist > yOffset){
            quadrant = 2;
        }else if(rightDist > xOffset){
            quadrant = 3;
        }else{
            quadrant = 4;
        }
        double angle = Math.toDegrees(Math.atan(Math.abs(yOffset-rightDist)/Math.abs(xOffset-forwardsDist)));
        double[] list = {quadrant, angle};
        return list;
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
        /*
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

         */

        runMotors(0);
        lastVelocity = 0;
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


    public double returnAprilTagIDOffset(int id, long timeout){

        long timeOutTime = System.currentTimeMillis() + timeout;
        while(teamUtil.keepGoing(timeOutTime)){
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for(AprilTagDetection detection : detections){
                if(detection.id == id){
                    log("April Tag Detected");

                    return  detection.ftcPose.x*CMS_PER_INCH;
                }
            } // TODO: do more tries
        }
        log("No April Tag Seen");
        return noAprilTag;
    }
    // returns an x offset of the robot relative to the center of the backdrop.  Negative is left of center
    public float getRobotBackdropXOffset () {
        if (!aprilTagProcessorRunning) {
            log("ERROR: getRobotBackdropXOffset called without April Tag Processor Running");
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
                return -1* offsetTotal / numTags;
            } else {
                return (float) noAprilTag;
            }
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Drive robot based on two joystick values
    // Implements a deadband where joystick position will be ignored (translation and rotation)
    // Uses a linear scale that starts at the edge of the dead band
    // Attempts to hold the last heading that was commanded via a turn
    public void driveJoyStick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast) {
        boolean details = true;

        float DEADBAND = 0.1f;
        float SLOPE = 1 / (1-DEADBAND); // linear from edge of dead band to full power  TODO: Should this be a curve?

        float POWERFACTOR = 1; // MAKE LESS THAN 0 to cap upperlimit of power
        float leftX ;
        float leftY ;
        float rotationAdjustment;

        float scaleAmount = isFast ? 1f : 0.5f; // full power is "fast", half power is "slow"

        float frontLeft, frontRight, backRight, backLeft;

        // Ensure nothing happens if we are inside the deadband
        if (Math.abs(leftJoyStickX)<DEADBAND) {leftJoyStickX = 0;}
        if (Math.abs(leftJoyStickY)<DEADBAND) {leftJoyStickY = 0;}
        if (Math.abs(rightJoyStickX)<DEADBAND) {rightJoyStickX = 0;}

        if (movingAutonomously.get() && (leftJoyStickX != 0 || rightJoyStickX != 0 || leftJoyStickY != 0)) { // Do we need to interrupt an autonomous operation?
            manualInterrupt.set(true);
        }

        if (leftJoyStickX > 0) {
            leftX = (leftJoyStickX - DEADBAND) * SLOPE * scaleAmount * POWERFACTOR;
        } else if (leftJoyStickX < 0) {
            leftX = (leftJoyStickX + DEADBAND) * SLOPE * scaleAmount * POWERFACTOR;
        } else {
            leftX = 0;
        }
        if (leftJoyStickY > 0) {
            leftY = (leftJoyStickY - DEADBAND) * SLOPE * scaleAmount * POWERFACTOR;
        } else if (leftJoyStickY < 0) {
            leftY = (leftJoyStickY + DEADBAND) * SLOPE * scaleAmount * POWERFACTOR;
        } else {
            leftY = 0;
        }

        if (Math.abs(rightJoyStickX) > DEADBAND) { // driver is turning the robot
            rotationAdjustment = (float) (rightJoyStickX * 0.525 * scaleAmount);
            holdingHeading = false;
        } else { // Need to automatically hold the current heading
            if (!holdingHeading) { // start to hold current heading
                heldHeading = getHeading();
                holdingHeading = true;
            }
            rotationAdjustment = (float) getHeadingError(heldHeading) * -1f * .1f; // auto rotate to held heading
            rotationAdjustment = rotationAdjustment * Math.min(Math.max(Math.abs(leftX), Math.abs(leftY)), 0.7f); // make it proportional to speed
        }

        frontLeft = -(leftY - leftX - rotationAdjustment);
        frontRight = (-leftY - leftX - rotationAdjustment);
        backRight = (-leftY + leftX - rotationAdjustment);
        backLeft = -(leftY + leftX - rotationAdjustment);

        if (details) {
            teamUtil.telemetry.addData("LEFTX:", leftX);
            teamUtil.telemetry.addData("LEFTY:", leftY);
            if (holdingHeading) {
                teamUtil.telemetry.addData("HOLDING:", heldHeading);
            }
        }

        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        br.setPower(backRight);
        bl.setPower(backLeft);

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

    public void geoFenceDriveJoystick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast, double robotHeading) {
        if (Math.abs(robotHeading-180)<10) { // TODO: Also maybe don't do this unless output is in score position?
            if ((getLeftProximity() || getRightProximity()) && leftJoyStickX > 0) {
                leftJoyStickX = 0;
            }
        }
        universalDriveJoystick(leftJoyStickX, leftJoyStickY, rightJoyStickX, isFast, robotHeading);
    }
}

