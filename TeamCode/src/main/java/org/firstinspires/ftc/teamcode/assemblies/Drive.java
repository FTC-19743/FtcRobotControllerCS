package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.teamcode.assemblies.OLD.bottomColorSensor;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Drive {
////////////////////////////////////////////////////////////////////////////////////////////////
//
//    Drive class for mecanum wheels
//
////////////////////////////////////////////////////////////////////////////////////////////////

    // constants
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public bottomColorSensor colorSensor;


    public BNO055IMU imu; //This variable is the imu
    public static double HEADING_OFFSET; // offset between IMU heading and field

    public DcMotorEx fl = null;
    public DcMotorEx fr = null;
    public DcMotorEx bl = null;
    public DcMotorEx br = null;
    public AnalogInput ult = null;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public CameraName camera1;
    public CameraName camera2;
    public WebcamName webcam1;
    public WebcamName webcam2;


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
    public double lastVelocity;
    public double CMS_PER_INCH = 2.54;
    public Drive() {
        teamUtil.log("Constructing Drive");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
        colorSensor = new bottomColorSensor(hardwareMap.get(ColorSensor.class, "bottomColor"));
    }

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public void initialize() {
        teamUtil.log("Initializing Drive");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fl = hardwareMap.get(DcMotorEx.class, "flm");
        fr = hardwareMap.get(DcMotorEx.class, "frm");
        bl = hardwareMap.get(DcMotorEx.class, "blm");
        br = hardwareMap.get(DcMotorEx.class, "brm");
        ult = hardwareMap.analogInput.get("ult");

        // colorSensor.calibrate();
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        initAprilTag();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        setMotorsBrake();
        teamUtil.log("Initializing Drive - FINISHED");
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

    public void setHeading(int heading) {
        HEADING_OFFSET = getRawHeading() - heading;
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
    public double getUltrasonicDistance(){
        double voltage = ult.getVoltage();
        double distance = (260/3*(voltage-.55)+36)*2.54; // based on real world distances measured
        return distance;
    }
    public void stopAtUltDistance(double distanceFromWall, double robotHeading, double driveHeading){

        MotorData data = new MotorData();
        getDriveMotorData(data);
        setMotorsWithEncoder();
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
            log("in while loop");
            distance = getUltrasonicDistance();
            log("distance: "+distance);
            double velocity = MIN_END_VELOCITY+Math.abs(distance-distanceFromWall)*VELOCITY_DECREASE_PER_CM;
            log("Velocity: "+velocity);
            driveMotorsHeadingsFR(driveHeading,robotHeading, velocity);
        }
        if(details){
            log("distance before pause: "+distance);
        }
        stopMotors();
        teamUtil.pause(1000);

        distance = getUltrasonicDistance();
        if(details){
            log("distance at end: "+distance);
        }
    }
    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        camera1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTag)
                .build();

    }
    public void setCamera(int camera) {
        if (camera == 2) {
                visionPortal.setActiveCamera(webcam2);
        }else{
            visionPortal.setActiveCamera(webcam1);
        }
    }

    public void centerOnAprilTag(int id, double xOffset, double yOffset, double heading){
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
        while(startDecelDist<distance&&distance > 4){
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
}
