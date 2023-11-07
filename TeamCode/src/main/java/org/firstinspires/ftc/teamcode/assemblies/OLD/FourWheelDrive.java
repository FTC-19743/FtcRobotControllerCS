package org.firstinspires.ftc.teamcode.assemblies.OLD;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.bottomColorSensor;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public class FourWheelDrive {


    HardwareMap hardwareMap;
    Telemetry telemetry;
    public bottomColorSensor colorSensor;


    public BNO055IMU imu; //This variable is the imu
    public static double HEADING_OFFSET; // offset between IMU heading and field

    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;


    public double COUNTS_PER_MOTOR_REV = 537.7;    // GoBilda 5202 312 RPM
    public double COUNTS_PER_CENTIMETER = 17.923;
    public double MIN_START_VELOCITY = 200; //tentative value
    public double MIN_END_VELOCITY = 100; //tentative value
    public double MAX_ACCELERATION = 10; //tentative value
    public double MAX_DECELERATION = -5; //tentative value (should be negative)
    public double ROTATION_ADJUST_FACTOR = .04; // TODO: Tune this for  auto
    public PIDFCoefficients pidNew = new PIDFCoefficients(5,0.121,0,12.136, MotorControlAlgorithm.PIDF);

    public FourWheelDrive() {
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
        frontLeft = hardwareMap.get(DcMotorEx.class, "flm");
        frontRight = hardwareMap.get(DcMotorEx.class, "frm");
        backLeft = hardwareMap.get(DcMotorEx.class, "blm");
        backRight = hardwareMap.get(DcMotorEx.class, "brm");

        // colorSensor.calibrate();
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        setTargetPositionToleranceAllMotors(20);

        //frontLeft.setTargetPositionTolerance(20);
       // frontRight.setTargetPositionTolerance(20);
       // backLeft.setTargetPositionTolerance(20);
        //backRight.setTargetPositionTolerance(20);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        teamUtil.log("Initializing Drive - FINISHED");




    }

    public void initializePIDFCoefficients(){
        setNewPIDCoefficients();
        PIDFCoefficients frontLeftPID = frontLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        log("Front Left PID" + frontLeftPID);
    }
    public void calibrate(){
        colorSensor.calibrate();
        teamUtil.log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
    }
    public double getIMUHeading() {
        Orientation anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (anglesCurrent.firstAngle);
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return our current heading as a 0 to 360 range.
    public double getHeading() {
        return adjustAngle(getIMUHeading() - HEADING_OFFSET);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Make the current heading 0.
    public void resetHeading() {
        HEADING_OFFSET = getIMUHeading();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Make the current heading to specified number
    public void setHeading(int heading){
        HEADING_OFFSET = getIMUHeading()-heading;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // adjust the given angle to be in the range 0-360.
    public double adjustAngle(double angle) {
        //assuming imu runs from [0, 360] and angle is added/subtracted, adjust it to expected reading
        while (angle >= 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    /*
    public void updatePowers(){
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }

     */

    public void setTargetPositionToleranceAllMotors(int tolerance){
        frontRight.setTargetPositionTolerance(tolerance);
        frontLeft.setTargetPositionTolerance(tolerance);
        backLeft.setTargetPositionTolerance(tolerance);
        backRight.setTargetPositionTolerance(tolerance);
    }
    public void setNewPIDCoefficients(){
        frontLeft.setPositionPIDFCoefficients(pidNew.p);
        frontRight.setPositionPIDFCoefficients(pidNew.p);
        backLeft.setPositionPIDFCoefficients(pidNew.p);
        backRight.setPositionPIDFCoefficients(pidNew.p);

    }

    public void logPIDCoefficients(){

    }

    public void outputTelemetry() {
        telemetry.addData("Output  ", "flm:%d frm:%d blm:%d brm:%d heading:%f nh: %f",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition(), getIMUHeading(), getHeading());

        telemetry.addData("Is On Line", "%b", colorSensor.isOnTape());
        telemetry.addData("Red Value ", colorSensor.redValue());
        telemetry.addData("Blue Value ", colorSensor.blueValue());
        telemetry.addLine("Target Position Tolerance" + frontLeft.getTargetPositionTolerance());

    }

    public void setAllMotorsActiveBreak(){
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setVelocity(3000);

        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setVelocity(3000);

        backLeft.setTargetPosition(backLeft.getCurrentPosition());
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setVelocity(3000);

        backRight.setTargetPosition(backRight.getCurrentPosition());
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setVelocity(3000);

    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Some fancy new methods for driving at any angle while holding a heading
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public static class MotorData { // a helper class to allow for faster access to hub data
        int eFL, eFR, eBL, eBR;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Return the encoder positions for all 4 drive motors
    // TODO: Use the Batch data feature (see examples) to speed this up
    public void getDriveMotorData(MotorData data) {
        data.eFL = frontLeft.getCurrentPosition();
        data.eFR = frontRight.getCurrentPosition();
        data.eBL = backLeft.getCurrentPosition();
        data.eBR = backRight.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Calculate the distance the robot has moved from the specified initialPosition
    public int getEncoderDistance(MotorData initialPositions) {
        MotorData currentPositions = new MotorData();
        getDriveMotorData(currentPositions);

        // Calculate the vector along the forward/backward axis
        int ForwardVector = (currentPositions.eFL-initialPositions.eFL)
                + (currentPositions.eFR-initialPositions.eFR)
                + (currentPositions.eBL-initialPositions.eBL)
                + (currentPositions.eBR-initialPositions.eBR);
        // Calculate the vector along the left/right axis
        int SideVector = (currentPositions.eFL-initialPositions.eFL)
                + (currentPositions.eBR-initialPositions.eBR)
                - ((currentPositions.eFR-initialPositions.eFR)
                + (currentPositions.eBL-initialPositions.eBL));
        // Return the hypotenuse of the two vectors
        // divide by 4 to account for the math that adds all 4 motor encoders
        return (int) (Math.sqrt(Math.pow(ForwardVector,2)+Math.pow(SideVector,2)) / 4);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Update the drive motor velocities
    // TODO: Use the Batch data feature (see examples) to speed this up
    public void setDriveVelocities(double flV, double frV, double blV, double brV){
        frontLeft.setVelocity(flV);
        frontRight.setVelocity(frV);
        backLeft.setVelocity(blV);
        backRight.setVelocity(brV);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void stopDrive() {
        setDriveVelocities(0,0,0,0);
        teamUtil.log("STOP Motors");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int cmsToEncoderTics(double cms) { // return the # of encoder tics for the specified inches
        return (int) (cms * COUNTS_PER_CENTIMETER);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return the difference between the current heading and a target heading.  Returns -180 to 180
    // Positive number means we need to turn left
    public double getHeadingError(double targetAngle) {

        double robotError;

        // calculate heading error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set the velocity of all 4 motors based on a driveHeading RELATIVE TO ROBOT and provided velocity
    // Will rotate robot as needed to achieve and hold robotHeading RELATIVE TO FIELD
    public void driveMotorsHeadings(double driveHeading, double robotHeading, double velocity) {
        double flV, frV, blV, brV;
        double x, y, scale;
        boolean details = false;

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

        // Clip to motor power range and scale to velocity
        flV = Math.max(-1.0, Math.min(x + y, 1.0))*velocity;
        brV = flV;
        frV = Math.max(-1.0, Math.min(y - x, 1.0))*velocity;
        blV = frV;

        // Adjust for rotational drift
        flV = flV - rotationAdjust;
        brV = brV + rotationAdjust;
        frV = frV + rotationAdjust;
        blV = blV - rotationAdjust;

        if (details) {
            teamUtil.log("driveH: " + driveHeading + " robotH: "+ robotHeading + "HError: "+headingError+ " RotAdj: " + rotationAdjust);
            teamUtil.log("flV: " + flV + " frV: "+ frV + "blV: "+blV+ " brV: " + brV);

        }
        // Update the motors
        setDriveVelocities(flV, frV, blV, brV);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set the velocity of all 4 motors based on a driveHeading RELATIVE TO FIELD and provided velocity
    // Will rotate robot as needed to achieve and hold robotHeading RELATIVE TO FIELD
    public void driveMotorsHeadingsFR(double driveHeading, double robotHeading, double velocity) {
        double RRDriveHeading = getHeadingError(driveHeading);
        driveMotorsHeadings(RRDriveHeading, robotHeading, velocity);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void runToPole(){
        long opTime = System.currentTimeMillis();

        setAllMotorsRunUsingEncoder();
        MotorData start = new MotorData();
        getDriveMotorData(start);
        teamUtil.log("BACK UP");
        while (getEncoderDistance(start) < 40*COUNTS_PER_CENTIMETER) {
            driveMotorsHeadingsFR(350, 180,1500);
        }
        getDriveMotorData(start);
        teamUtil.log("BACK UP AND TURN");
        while (getEncoderDistance(start) < 30*COUNTS_PER_CENTIMETER) {
            driveMotorsHeadingsFR(355, 225,1000);
        }
        getDriveMotorData(start);
        teamUtil.log("BACKUP");
        while (getEncoderDistance(start) < 10*COUNTS_PER_CENTIMETER) {
            driveMotorsHeadingsFR(45, 225,500);
        }
        setAllMotorsActiveBreak();
        teamUtil.log("OP TIME: "+ (System.currentTimeMillis()-opTime));
        teamUtil.pause(1000);
        setAllMotorsRunUsingEncoder();
    }

    public void newRunToWall(){
        long opTime = System.currentTimeMillis();

        setAllMotorsRunUsingEncoder();
        MotorData start = new MotorData();
        getDriveMotorData(start);
        teamUtil.log("BACK UP AND TURN");
        while (getEncoderDistance(start) < 25*COUNTS_PER_CENTIMETER) {
            driveMotorsHeadingsFR(225, 180,1500);
        }
        teamUtil.log("DRIVE TOWARDS WALL");
        getDriveMotorData(start);
        while (getEncoderDistance(start) < 30*COUNTS_PER_CENTIMETER) {
            driveMotorsHeadingsFR(180 , 180,1500);
        }
        teamUtil.log("DRIFT TOWARDS LINE");
        getDriveMotorData(start);
        while (!colorSensor.isOnTape() && getEncoderDistance(start) < 30*COUNTS_PER_CENTIMETER) {
            driveMotorsHeadingsFR(135 , 180,1000);
        }
        if (colorSensor.isOnTape()) {
            teamUtil.log("FOUND TAPE");
        } else {
            teamUtil.log("FAIL SAFE");
        }
        long startTime = System.currentTimeMillis();
        teamUtil.log("SQUARE ON WALL");
        while (System.currentTimeMillis()-startTime < 500) {
            driveMotorsHeadingsFR(200 , 180,750); // drift left a bit to compensate for late tape reading
        }
        stopDrive();
        teamUtil.log("OP TIME: "+ (System.currentTimeMillis()-opTime));
    }

    public void setAllMotorsRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setAllMotorsRunUsingEncoder() {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeLeft(double speed, double centimeters) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        centimeters *= 1.11;
        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        teamUtil.log("Strafing left");
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeRight(double speed, double centimeters) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        centimeters *= 1.11;
        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        teamUtil.log("Strafing right");
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void newStrafeRight(double speed, double centimeters){
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.1));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.1));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.1));
        backRight.setTargetPosition((int)(newBackRightTarget*.1));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.4));
        frontRight.setPower(Math.abs(speed)*.4);
        backLeft.setPower(Math.abs(speed)*.4);
        backRight.setPower(Math.abs(speed)*.4);
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.3));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.3));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.3));
        backRight.setTargetPosition((int)(newBackRightTarget*.3));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.7));
        frontRight.setPower(Math.abs(speed)*.7);
        backLeft.setPower(Math.abs(speed)*.7);
        backRight.setPower(Math.abs(speed)*.7);
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.7));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.7));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.7));
        backRight.setTargetPosition((int)(newBackRightTarget*.7));

        // Turn On RUN_TO_POSITION


        // start motion.
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));
        teamUtil.log("Moving Forward");
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget));
        frontRight.setTargetPosition((int)(newFrontRightTarget));
        backLeft.setTargetPosition((int)(newBackLeftTarget));
        backRight.setTargetPosition((int)(newBackRightTarget));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.4));
        frontRight.setPower(Math.abs(speed)*.4);
        backLeft.setPower(Math.abs(speed)*.4);
        backRight.setPower(Math.abs(speed)*.4);
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void newStrafeLeft(double speed, double centimeters){
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.1));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.1));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.1));
        backRight.setTargetPosition((int)(newBackRightTarget*.1));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.4));
        frontRight.setPower(Math.abs(speed)*.4);
        backLeft.setPower(Math.abs(speed)*.4);
        backRight.setPower(Math.abs(speed)*.4);
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.3));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.3));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.3));
        backRight.setTargetPosition((int)(newBackRightTarget*.3));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.7));
        frontRight.setPower(Math.abs(speed)*.7);
        backLeft.setPower(Math.abs(speed)*.7);
        backRight.setPower(Math.abs(speed)*.7);
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.7));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.7));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.7));
        backRight.setTargetPosition((int)(newBackRightTarget*.7));

        // Turn On RUN_TO_POSITION


        // start motion.
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));
        teamUtil.log("Moving Forward");
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget));
        frontRight.setTargetPosition((int)(newFrontRightTarget));
        backLeft.setTargetPosition((int)(newBackLeftTarget));
        backRight.setTargetPosition((int)(newBackRightTarget));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.4));
        frontRight.setPower(Math.abs(speed)*.4);
        backLeft.setPower(Math.abs(speed)*.4);
        backRight.setPower(Math.abs(speed)*.4);
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }




    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveCM(double speed, double centimeters) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // start motion.
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));
        teamUtil.log("Moving Forward");
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }


    public void newMoveCM(double velocity, double centimeters) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // start motion.
        frontLeft.setVelocity(Math.abs(velocity));
        frontRight.setVelocity(Math.abs(velocity));
        backLeft.setVelocity(Math.abs(velocity));
        backRight.setVelocity(Math.abs(velocity));
        teamUtil.log("Moving Forward");
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);

    }

    public void backCM(double speed, double centimeters) {
        moveCM(speed, -centimeters);
    }

    public void newBackCM(double velocity, double centimeters) {
        newMoveCM(velocity, -centimeters);
    }

    public void runMotors(double velocity) {
        frontLeft.setVelocity(velocity);
        frontRight.setVelocity(velocity);
        backLeft.setVelocity(velocity);
        backRight.setVelocity(velocity);
    }


    public void moveCmWAcceleration(double cruiseVelocity, double centimeters) {
        double startEncoderPosition = frontLeft.getCurrentPosition();


        double velocityChangeNeededAccel = cruiseVelocity - MIN_START_VELOCITY;
        double velocityChangeNeededDecel = cruiseVelocity - MIN_END_VELOCITY;


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double totalTics = centimeters * COUNTS_PER_CENTIMETER;
        double ticsDuringAcceleration = velocityChangeNeededAccel / MAX_ACCELERATION;
        double ticsDuringDeceleration = velocityChangeNeededDecel / MAX_DECELERATION;
        double cruiseTics = totalTics - ticsDuringDeceleration - ticsDuringAcceleration;

        while (frontLeft.getCurrentPosition() < startEncoderPosition + ticsDuringAcceleration) {
            double ticsSinceStart = frontLeft.getCurrentPosition() - startEncoderPosition;

            runMotors(MAX_ACCELERATION * ticsSinceStart + MIN_START_VELOCITY);
        }

        while (frontLeft.getCurrentPosition() < cruiseTics + startEncoderPosition) {
            runMotors(cruiseVelocity);
        }

        double encoderAfterCruise = frontLeft.getCurrentPosition();

        while (frontLeft.getCurrentPosition() < startEncoderPosition + totalTics) {
            double ticsSinceCruise = frontLeft.getCurrentPosition() - encoderAfterCruise;

            runMotors(MAX_DECELERATION * ticsSinceCruise + MIN_END_VELOCITY);
        }
        runMotors(0);

    }

    public void spinRightWithIMU(double degrees, double speed) {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double initialIMU = getIMUHeading();
        double IMUNeeded = initialIMU - degrees + 8;
        String initialIMUToPrint = String.format("%.2f", initialIMU);
        String IMUNeededToPrint = String.format("%.2f", IMUNeeded);
        log("Initial IMU: " + initialIMUToPrint);
        log("IMU Needed: " + IMUNeededToPrint);
        //if the end IMU is an impossible value, this code allows the robot to transition
        //from the left hemisphere to the right without issues
        //IMU Diagram Doc: https://docs.google.com/document/d/1RI6dZkmHRWhUBy-ZgONwAEO7AxOb_vjcoX40VSjJYjg/edit
        if (IMUNeeded < -180) {
            double currentIMU = getIMUHeading();
            while (currentIMU < 0) {
                currentIMU = getIMUHeading();
                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                frontRight.setPower(-1 * speed);
                backRight.setPower(-1 * speed);
                String currentIMUToPrint = String.format("%.2f", currentIMU);
               // log(currentIMUToPrint);
            }
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            //finds out how many degrees were traveled depending on where the robot was initially
            //facing
            double degreesTraveled = 0;
            if (initialIMU < 0) {
                degreesTraveled = 180 + initialIMU;
            } else if (initialIMU == 0) {

            } else {
                degreesTraveled = 180 - initialIMU;
            }
            currentIMU = getIMUHeading();
            //Finds out how far to travel and prints important values
            double degreesLeft = degrees - degreesTraveled;
            double IMUNeeded2 = 179.999999 - degreesLeft;
            String degreesTraveledToPrint = String.format("%.2f", degreesTraveled);
            log(degreesTraveledToPrint);
            String degreesLeftToPrint = String.format("%.2f", degreesLeft);
            log(degreesLeftToPrint);

            String IMUNeeded2ToPrint = String.format("%.2f", IMUNeeded2);


            log(IMUNeeded2ToPrint);

            while (currentIMU > IMUNeeded2) {
                //currentIMU=getIMUHeading();
                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                frontRight.setPower(-1 * speed);
                backRight.setPower(-1 * speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
                //log(currentIMUToPrint);
                currentIMU = getIMUHeading();

            }
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

        }
        //If math is simple and no conversion is needed the robot will spin without issue
        else {
            double currentIMU = getIMUHeading();
            while (currentIMU > IMUNeeded) {
                //currentIMU=getIMUHeading();
                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                frontRight.setPower(-1 * speed);
                backRight.setPower(-1 * speed);
                String currentIMUToPrint = String.format("%.2f", currentIMU);
                //log(currentIMUToPrint);
                currentIMU = getIMUHeading();
            }
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }


    }

    public void spinLeftWithIMU(double degrees, double speed){
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double initialIMU = getIMUHeading();
        double IMUNeeded = initialIMU+degrees-8;
        String initialIMUToPrint = String.format("%.2f", initialIMU);
        String IMUNeededToPrint = String.format("%.2f", IMUNeeded);
        log("Initial IMU: " + initialIMUToPrint);
        log("IMU Needed: " + IMUNeededToPrint);

        //IMU Diagram Doc: https://docs.google.com/document/d/1RI6dZkmHRWhUBy-ZgONwAEO7AxOb_vjcoX40VSjJYjg/edit
        if(IMUNeeded>180){
            double currentIMU = getIMUHeading();
            while(currentIMU>0){
                currentIMU=getIMUHeading();
                frontLeft.setPower(-1*speed);
                backLeft.setPower(-1*speed);
                frontRight.setPower(speed);
                backRight.setPower(speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
                //log(currentIMUToPrint);
            }
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            //finds out how many degrees were traveled depending on where the robot was initially
            //facing
            double degreesTraveled=0;
            if(initialIMU < 0){
                degreesTraveled = 179.999-initialIMU;
            }
            else if(initialIMU==0){

            }
            else{
                degreesTraveled = 179.999+initialIMU*-1;
            }
            currentIMU=getIMUHeading();
            //Finds out how far to travel and prints important values
            double degreesLeft = degrees-degreesTraveled;
            double IMUNeeded2 = -179.999999+degreesLeft;
            String degreesTraveledToPrint = String.format("%.2f", degreesTraveled);
            String degreesLeftToPrint = String.format("%.2f", degreesLeft);
            String IMUNeeded2ToPrint = String.format("%.2f", IMUNeeded2);
            log("Degrees Traveled: " + degreesTraveledToPrint);
            log("Degrees Left To Travel " + degreesLeftToPrint);
            log("IMU needed as robot enters left hemisphere: " + IMUNeeded2ToPrint);

            while(currentIMU<IMUNeeded2){
                currentIMU=getIMUHeading();
                frontLeft.setPower(-1*speed);
                backLeft.setPower(-1*speed);
                frontRight.setPower(speed);
                backRight.setPower(speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
                //log(currentIMUToPrint);
                currentIMU=getIMUHeading();

            }
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

        }
        //If math is simple and no conversion is needed the robot will spin without issue
        else{
            double currentIMU = getIMUHeading();
            while(currentIMU<IMUNeeded){
                currentIMU=getIMUHeading();
                frontLeft.setPower(-1*speed);
                backLeft.setPower(-1*speed);
                frontRight.setPower(speed);
                backRight.setPower(speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
               // log(currentIMUToPrint);
                currentIMU=getIMUHeading();
            }
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }








    }


    public void moveCMNoStop(double cm, int velocity){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int ticks = frontLeft.getCurrentPosition();
        double ticksNeeded = cm*COUNTS_PER_CENTIMETER+ticks;

        frontLeft.setVelocity(velocity);
        backLeft.setVelocity(velocity);
        backRight.setVelocity(velocity);
        frontRight.setVelocity(velocity);
        while(frontLeft.getCurrentPosition()<ticksNeeded){

        }
        frontLeft.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);
        frontRight.setVelocity(0);


    }

    public void strafeRightToLine(int maxEncoderTicks, double speed){
        log("Strafing Right To Line");
        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);
        frontRight.setPower(-speed);
        int ticks = frontLeft.getCurrentPosition();
        while((!colorSensor.isOnTape())&&Math.abs(frontLeft.getCurrentPosition()-ticks) < maxEncoderTicks){
            //log("Not On Line");
            //log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" +colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
            //log("On Blue " +colorSensor.onBlue()+ " On Red " +colorSensor.onRed());

        }
        log("On Line");
        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition());
        backLeft.setTargetPosition(backLeft.getCurrentPosition());

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        log("Done Strafing Right To Line");



    }
    public void strafeLeftToLine(int maxEncoderTicks, double speed){
        log("Strafing Left To Line");
        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
        frontRight.setPower(speed);
        int ticks = frontLeft.getCurrentPosition();
        while(!colorSensor.isOnTape() &&Math.abs(frontLeft.getCurrentPosition()-ticks) < maxEncoderTicks){
            log("Not On Line");
            //log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" +colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
            //log("On Blue " +colorSensor.onBlue()+ " On Red " +colorSensor.onRed());

        }
        log("On Line");
        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition());
        backLeft.setTargetPosition(backLeft.getCurrentPosition());

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        log("Done Strafing Right To Line");



    }


    public void strafeLeftDiagonallyToLine(int maxEncoderTicks, double speed){
        log("Strafing Left Diagonally To Line");
        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());
        log("Back Left Start Encoder: " + backLeft.getCurrentPosition());

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(speed*1/3);
        backLeft.setPower(speed);
        backRight.setPower(speed*1/3);
        frontRight.setPower(speed);
        int ticks = frontLeft.getCurrentPosition();

        while(!colorSensor.isOnTape() &&Math.abs(frontLeft.getCurrentPosition()-ticks) < maxEncoderTicks){
            log("Not On Line");
            //log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" +colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
            //log("On Blue " +colorSensor.onBlue()+ " On Red " +colorSensor.onRed());

        }



        log("On Line");
        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition());
        backLeft.setTargetPosition(backLeft.getCurrentPosition());
        log("Back Left End Position: " + backLeft.getCurrentPosition());
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        log("Done Strafing Left Diagonally To Line");



    }

    public void strafeRightDiagonallyToLine(int maxEncoderTicks, double speed){
        log("Strafing Right Diagonally To Line");
        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());
        log("Front Left Start Encoder: " + frontLeft.getCurrentPosition());

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(speed);
        backLeft.setPower(speed*1/3);
        backRight.setPower(speed);
        frontRight.setPower(speed*1/3);
        int ticks = frontLeft.getCurrentPosition();

        while(!colorSensor.isOnTape() &&Math.abs(frontLeft.getCurrentPosition()-ticks) < maxEncoderTicks){
            log("Not On Line");
            //log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" +colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
            //log("On Blue " +colorSensor.onBlue()+ " On Red " +colorSensor.onRed());

        }
        if(colorSensor.isOnTape()){
            log("On Line");
        }




        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition());
        backLeft.setTargetPosition(backLeft.getCurrentPosition());
        log("Front Left End Position: " + frontLeft.getCurrentPosition());
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        log("Done Strafing Right Diagonally To Line");



    }
    public void relativeSpinRight(double degrees, double topSpeed){
        setHeading(0);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(getHeading()<degrees*.1){
            frontLeft.setPower(-.4*topSpeed);
            backLeft.setPower(-.4*topSpeed);
            frontRight.setPower(.4*topSpeed);
            backRight.setPower(.4*topSpeed);
        }
        while(getHeading()<degrees*.2){
            frontLeft.setPower(-.7*topSpeed);
            backLeft.setPower(-.7*topSpeed);
            frontRight.setPower(.7*topSpeed);
            backRight.setPower(.7*topSpeed);
        }
        while(getHeading()<degrees*.8){
            frontLeft.setPower(-topSpeed);
            backLeft.setPower(-topSpeed);
            frontRight.setPower(topSpeed);
            backRight.setPower(topSpeed);
        }
        while(getHeading()<degrees*.9){
            frontLeft.setPower(-.6*topSpeed);
            backLeft.setPower(-.6*topSpeed);
            frontRight.setPower(.6*topSpeed);
            backRight.setPower(.6*topSpeed);
        }
        while(getHeading()<degrees){
            frontLeft.setPower(-.2*topSpeed);
            backLeft.setPower(-.2*topSpeed);
            frontRight.setPower(.2*topSpeed);
            backRight.setPower(.2*topSpeed);
        }
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public void setAllMotorsToSpeed(double speed){
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
    }

    public void relativeSpinLeft(double degrees, double topSpeed){
        setHeading(0);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(getHeading()<360-degrees*.1){
            frontLeft.setPower(.2*topSpeed);
            backLeft.setPower(.2*topSpeed);
            frontRight.setPower(-.2*topSpeed);
            backRight.setPower(-.2*topSpeed);
        }
        while(getHeading()<360-degrees*.2){
            frontLeft.setPower(.6*topSpeed);
            backLeft.setPower(.6*topSpeed);
            frontRight.setPower(-.6*topSpeed);
            backRight.setPower(-.6*topSpeed);
        }
        while(getHeading()<360-degrees*.7){
            frontLeft.setPower(topSpeed);
            backLeft.setPower(topSpeed);
            frontRight.setPower(-topSpeed);
            backRight.setPower(-topSpeed);
        }
        while(getHeading()<360-degrees*.9){
            frontLeft.setPower(.6*topSpeed);
            backLeft.setPower(.6*topSpeed);
            frontRight.setPower(-.6*topSpeed);
            backRight.setPower(-.6*topSpeed);
        }
        while(getHeading()<360-degrees){
            frontLeft.setPower(.2*topSpeed);
            backLeft.setPower(.2*topSpeed);
            frontRight.setPower(-.2*topSpeed);
            backRight.setPower(-.2*topSpeed);
        }
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public void spinRightToHeading(double wantedIMU, double speed){
        log("Spin Right To Heading Called");
        log("Heading At Start" + getHeading());
        log("Desired Heading" + wantedIMU);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(getHeading()>(wantedIMU+30)){
            frontLeft.setPower(speed);
            backLeft.setPower(speed);
            frontRight.setPower(-speed);
            backRight.setPower(-speed);
            while(getHeading()>(wantedIMU+30)){

            }
        }
        frontLeft.setPower(0.15);
        backLeft.setPower(0.15);
        frontRight.setPower(-0.15);
        backRight.setPower(-0.15);
        while(getHeading()>(wantedIMU+3)){
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        log("Finishing Spin Right to Heading");



    }



    public void spinLeftToHeading(double wantedIMU, double speed){
        log("Spin Left To Heading Called");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(getHeading()<(wantedIMU-30)){
            frontLeft.setPower(-speed);
            backLeft.setPower(-speed);
            frontRight.setPower(speed);
            backRight.setPower(speed);
            while(getHeading()<(wantedIMU-30)){
            }
        }
        frontLeft.setPower(-0.15);
        backLeft.setPower(-0.15);
        frontRight.setPower(0.15);
        backRight.setPower(0.15);
        while(getHeading()<(wantedIMU-3)){
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);



    }

    public void spinLeftToHeadingNoAccel(int wantedIMU, double speed){
        log("Spin Left To Heading No AccelCalled");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        while(getHeading()<wantedIMU-10){
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

    }

    public void spinRightToHeadingNoAccel(int wantedIMU, double speed){
        log("Spin Left To Heading No AccelCalled");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(-speed);
        backRight.setPower(-speed);
        while(getHeading()>wantedIMU+10){
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

    }



}