package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class Robot {
    public BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public Drive drive;
    public Intake intake;
    public Output output;
    public Lift lift;
    public Launcher launcher;

    public OpenCVPropFinder openCVPropFinder;


    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);

    }

    public Robot(){
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new Drive();
        intake = new Intake();
        output = new Output(intake);
        launcher = new Launcher();
        lift = new Lift();
        openCVPropFinder =  new OpenCVPropFinder();
    }

    public void initialize(){
        drive.initalize();
        intake.initalize();
        output.initialize();
        lift.initialize();
        launcher.initialize();

    }

    public void outputTelemetry(){
        drive.driveMotorTelemetry();
        output.outputTelemetry();
    }

    public void calibrate(){
        output.calibrate();
        output.goToLoad(); // ready to load
    }

    public int fieldSide() { // helper method that returns heading out towards the field
        return teamUtil.alliance == teamUtil.Alliance.RED ? 90 : 270;
    }
    public int driverSide() { // helper method that returns heading backs towards drivers
        return teamUtil.alliance == teamUtil.Alliance.RED ? 270 : 90;
    }

    public int audienceSide(){return 180;}

    public int scoreSide(){return 0;}

    public void auto(int path, teamUtil.Side side){ // TODO: Lose the "left" parameter and use TeamUtil.SIDE instead
        drive.setHeading(180); // Zero is towards the scoring side of field
        if(path==1&&side == teamUtil.Side.WING){
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(67,fieldSide());
            drive.moveCm(33.5,fieldSide()+50);
            drive.moveCm(10,driverSide());
            drive.moveCm(25, scoreSide());
            drive.moveCm(69, fieldSide());
            drive.moveCm(127, scoreSide(), drive.MAX_VELOCITY);
            output.goToScoreNoWait(1);
            drive.moveCm(90, driverSide()); // strafe
            teamUtil.pause(500);
            double aprilTagOffset;
            if(teamUtil.alliance == teamUtil.Alliance.RED) {
                aprilTagOffset = drive.returnAprilTagIDOffset(4, 500);
            }else{
                aprilTagOffset = drive.returnAprilTagIDOffset(1, 500);
            }
            log("aprilTagOffset: "+aprilTagOffset);
            log("aprilTagOffset"+aprilTagOffset);

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0,180,350,4000);


            drive.setMotorsActiveBrake();


            if(Math.abs(aprilTagOffset)>3.25){
                if(aprilTagOffset>0&&aprilTagOffset<900){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

                    }
                }
                else if(aprilTagOffset<0){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                }

                else{
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(15,driverSide());
                    }
                    else{
                        drive.moveCm(15,fieldSide());

                    }
                }
            }
            output.dropPixels();
            teamUtil.pause(1000);
            drive.moveCm(5,180);

            output.goToLoadNoWait();
            drive.moveCm(60,fieldSide());




        }
        else if(path==2&&side == teamUtil.Side.WING){
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(86,fieldSide());
            drive.moveCm(8.5,driverSide());
            drive.moveCm(36, audienceSide());
            drive.moveCm(64, fieldSide());
            drive.moveCm(170, scoreSide(), drive.MAX_VELOCITY);
            output.goToScoreNoWait(1);
            drive.moveCm(100, driverSide()); // strafe
            teamUtil.pause(500);
            double aprilTagOffset;
            if(teamUtil.alliance == teamUtil.Alliance.RED) {
                aprilTagOffset = drive.returnAprilTagIDOffset(5, 500);
            }else{
                aprilTagOffset = drive.returnAprilTagIDOffset(2, 500);
            }
            log("aprilTagOffset: "+aprilTagOffset);

            log("aprilTagOffset"+aprilTagOffset);

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0,180,350,4000);
            //while (!drive.tapeSensor1.isOnTape()||!drive.tapeSensor2.isOnTape()){}

            drive.setMotorsActiveBrake();





            if(Math.abs(aprilTagOffset)>3.25){
                if(aprilTagOffset>0&&aprilTagOffset<900){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

                    }
                }
                else if(aprilTagOffset<0){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                }

                else{
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(15,driverSide());
                    }
                    else{
                        drive.moveCm(15,fieldSide());

                    }
                }
            }
            output.dropPixels();
            teamUtil.pause(1000);
            drive.moveCm(5, audienceSide());
            output.goToLoadNoWait();
            drive.moveCm(60,fieldSide());


        }
        else if(path==3&&side == teamUtil.Side.WING){
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(67,fieldSide());
            drive.moveCm(33.5,fieldSide()-55);
            drive.moveCm(10, driverSide());


            drive.moveCm(52, audienceSide());

            drive.moveCm(69, fieldSide());

            drive.moveCm(150, scoreSide(), drive.MAX_VELOCITY);
            output.goToScoreNoWait(1);
            drive.moveCm(100, driverSide()); // strafe
            teamUtil.pause(500);
            double aprilTagOffset;
            if(teamUtil.alliance == teamUtil.Alliance.RED) {
                aprilTagOffset = drive.returnAprilTagIDOffset(6, 500);
            }else{
                aprilTagOffset = drive.returnAprilTagIDOffset(3, 500);
            }
            log("aprilTagOffset: "+aprilTagOffset);
            log("aprilTagOffset"+aprilTagOffset);

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0,180,350,4000);


            drive.setMotorsActiveBrake();


            if(Math.abs(aprilTagOffset)>3.25){
                if(aprilTagOffset>0&&aprilTagOffset<900){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

                    }
                }
                else if(aprilTagOffset<0){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                }

                else{
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(15,driverSide());
                    }
                    else{
                        drive.moveCm(15,fieldSide());

                    }
                }
            }
            output.dropPixels();
            teamUtil.pause(1000);
            drive.moveCm(5,180);

            output.goToLoadNoWait();
            drive.moveCm(80,fieldSide());





        }
        else if(!(side == teamUtil.Side.WING)&&path==1){

            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(65,fieldSide());
            drive.moveCm(33,fieldSide()+50);
            drive.moveCm(10,driverSide());




            output.goToScoreNoWait(1);
            drive.moveCm(40,0,350);//TODO:change to min end when callibrated (also, should be more like 80)
            double aprilTagOffset;
            if(teamUtil.alliance == teamUtil.Alliance.RED) {
                aprilTagOffset = drive.returnAprilTagIDOffset(4, 500);
            }else{
                aprilTagOffset = drive.returnAprilTagIDOffset(1, 500);
            }

            log("aprilTagOffset"+aprilTagOffset);

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0,180,350,4000);
            //while (!drive.tapeSensor1.isOnTape()||!drive.tapeSensor2.isOnTape()){}

            drive.setMotorsActiveBrake();


            if(Math.abs(aprilTagOffset)>3.25){
                if(aprilTagOffset>0&&aprilTagOffset<900){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

                    }
                }
                else if(aprilTagOffset<0){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                }

                else{
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(15,driverSide());
                    }
                    else{
                        drive.moveCm(15,fieldSide());

                    }
                }
            }
            output.dropPixels();
            teamUtil.pause(1000);
            drive.moveCm(5,180);

            output.goToLoadNoWait();
            drive.moveCm(60,fieldSide());


        }

        else if(!(side == teamUtil.Side.WING)&&path==2){
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(86,fieldSide());
            drive.moveCm(8.5,driverSide());
            output.goToScoreNoWait(1);
            drive.moveCm(40,0,350);//TODO:change to min end when callibrated (also, should be more like 80)
            double aprilTagOffset;
            if(teamUtil.alliance == teamUtil.Alliance.RED) {
                aprilTagOffset = drive.returnAprilTagIDOffset(5, 500);
            }else{
                aprilTagOffset = drive.returnAprilTagIDOffset(2, 500);
            }
            log("aprilTagOffset: "+aprilTagOffset);

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0,180,350,3000);
            //while (!drive.tapeSensor1.isOnTape()||!drive.tapeSensor2.isOnTape()){}

            drive.setMotorsActiveBrake();

            //TODO fix 3.25

            if(Math.abs(aprilTagOffset)>3.25){
                if(aprilTagOffset>0&&aprilTagOffset<900){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

                    }
                }
                else if(aprilTagOffset<0){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                }

                else{
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(15,driverSide());
                    }
                    else{
                        drive.moveCm(15,fieldSide());

                    }
                }
            }
            output.dropPixels();

            teamUtil.pause(1000);
            drive.moveCm(5,180);
            output.goToLoadNoWait();
            drive.moveCm(60,fieldSide());




        }

        else if(!(side == teamUtil.Side.WING)&&path==3){
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(80,fieldSide());
            drive.moveCm(33,fieldSide()+300);
            drive.moveCm(13,driverSide());




            output.goToScoreNoWait(1);
            drive.moveCm(10,0,350);//TODO:change to min end when callibrated (also, should be more like 80)
            double aprilTagOffset;
            if(teamUtil.alliance == teamUtil.Alliance.RED) {
                aprilTagOffset = drive.returnAprilTagIDOffset(6, 500);
            }else{
                aprilTagOffset = drive.returnAprilTagIDOffset(3, 500);
            }
            drive.moveCm(40,0,350);
            log("aprilTagOffset: "+aprilTagOffset);

            log("aprilTagOffset"+aprilTagOffset);

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0,180,350,4000);
            //while (!drive.tapeSensor1.isOnTape()||!drive.tapeSensor2.isOnTape()){}

            drive.setMotorsActiveBrake();


            if(Math.abs(aprilTagOffset)>3.25){
                if(aprilTagOffset>0&&aprilTagOffset<900){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

                    }
                }
                else if(aprilTagOffset<0){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                }

                else{
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(15,driverSide());
                    }
                    else{
                        drive.moveCm(15,fieldSide());

                    }
                }
            }
            output.dropPixels();
            teamUtil.pause(1000);
            drive.moveCm(5,180);

            output.goToLoadNoWait();
            drive.moveCm(96,fieldSide());

        }
        teamUtil.pause(5000); // TODO: Remove this for competition, just helps with debugging auto as it is developed

    }

}


