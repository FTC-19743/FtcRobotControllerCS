package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
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

    public int straferDistanceFarStack = 17790;

    public Robot() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new Drive();
        intake = new Intake();
        output = new Output(intake);
        launcher = new Launcher();
        lift = new Lift();
    }

    public void initialize() {
        drive.initalize(output);
        intake.initalize();
        output.initialize();
        lift.initialize();
        launcher.initialize();

    }

    public void outputTelemetry() {
        drive.driveMotorTelemetry();
        output.outputTelemetry();
    }

    public void calibrate() {
        output.calibrate();
        //output.goToLoad(); // ready to load
    }


    public int fieldSide() { // helper method that returns heading out towards the field
        return teamUtil.alliance == RED ? 90 : 270;
    }

    public int driverSide() { // helper method that returns heading backs towards drivers
        return teamUtil.alliance == RED ? 270 : 90;
    }

    public int audienceSide() {
        return 180;
    }

    public int scoreSide() {
        return 0;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SCORE Side Start - Push the purple pixel and get into position for AprilTag Localization based on path
    public boolean pushPurplePixelScoreV3(int path) {
        teamUtil.log("Pushing Pixel and Driving to April Tag Viewing Location");
        // TODO: SPEED UP IDEAS: Maybe hold the pixel in the collectors and take more direct paths, Maybe don't stop between all the movements
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.VIOLET);
        if ((teamUtil.alliance==RED && path == 1) || (teamUtil.alliance==BLUE && path == 3)) { // Under the Rigging
            drive.moveCm(drive.MAX_VELOCITY,71,fieldSide(), 180,0); // Was 71
            drive.moveCm(drive.MAX_VELOCITY,28, teamUtil.alliance==RED ? 150:210, 180,0);
            drive.moveCm(drive.MAX_VELOCITY,8, driverSide(), 180, 800);
            drive.moveCm(drive.MAX_VELOCITY,83, teamUtil.alliance==RED ? 30:330, 180,1000);
            /*
            drive.moveCm(drive.MAX_VELOCITY,67, fieldSide(), 180, 1000);
            drive.moveCm(drive.MAX_VELOCITY,30, teamUtil.alliance==RED ? 140 : 220, 180, 0); // was fieldSide() + 50
            drive.moveCm(drive.MAX_VELOCITY,10, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY, 78, 0, 180, 1000);

             */
        } else if ((teamUtil.alliance==RED && path == 3) || (teamUtil.alliance==BLUE && path == 1)) { // Near the backdrop
            drive.moveCm(drive.MAX_VELOCITY,68,teamUtil.alliance==RED?58:302, 180, 0);
            drive.moveCm(drive.MAX_VELOCITY,13, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY,  23 , teamUtil.alliance==RED?22.5:337.5, 180, 1000);
        } else { // Path 2, middle for either Alliance
            drive.moveCm(drive.MAX_VELOCITY, 86, fieldSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY, 8.5, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY, 45, 0, 180, 1000);
        }

        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);

        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // WING Side Start - Push the purple pixel and drive to nearest stack
    // TODO: SPEED UP IDEAS: Make the pixel pusher less deep and try to go on a more direct path? move pixel pusher to back and target middle stack?

    public boolean pushPurplePixelWingV3(int path,boolean operateArms) {
        teamUtil.log("Pushing Pixel");
        boolean details = false;

        if ((teamUtil.alliance==RED && path == 1) || (teamUtil.alliance==BLUE && path == 3)) { // Near the Stacks
            drive.moveCm(drive.MAX_VELOCITY,88, teamUtil.alliance==RED ? 114 : 246, 180,0); // Was 71
            //drive.moveCm(drive.MAX_VELOCITY,24,  teamUtil.alliance==RED ? 135 : 225, 180,0);
            if (!drive.strafeToEncoder(driverSide(), 180, 400, (teamUtil.alliance==RED ? 1 : -1)*9300, 2000)) {
                drive.stopMotors();
                return false;
            }
            drive.stopMotors();
            teamUtil.pause(250);
            if(operateArms){
                intake.startIntake();
            }
            drive.moveCm(drive.MAX_VELOCITY,42, 180, 180,0);
        } else if ((teamUtil.alliance==RED && path == 3) || (teamUtil.alliance==BLUE && path == 1)) { // Under the Rigging
            drive.moveCm(drive.MAX_VELOCITY,58, fieldSide(), 180,1000);
            drive.moveCm(drive.MAX_VELOCITY,40,  teamUtil.alliance==RED ? 30 : 330, 180,0);
            drive.moveCm(drive.MAX_VELOCITY, 5.5, driverSide(), 180,1000);
            drive.moveCm(drive.MAX_VELOCITY, 10, 180, 180,400);
            if (!drive.strafeToEncoder(fieldSide(), 180, 400, (teamUtil.alliance==RED ? 1 : -1)*9200, 2000)) { // tweaked encoder target a bit
                drive.stopMotors();
                return false;
            }
            drive.stopMotors();
            //teamUtil.pause(250);
            if(operateArms){
                intake.startIntake();
            }
            drive.moveCm(drive.MAX_VELOCITY, 78, 180, 180,400); // was 81
            drive.setMotorsFloat(); // coast to wall

            drive.stopMotors();
            teamUtil.pause(250);
            drive.setMotorsBrake();

            /*  Before linking movements
            drive.moveCm(drive.MAX_VELOCITY, 73, fieldSide(), 180,0);
            drive.moveCm(drive.MAX_VELOCITY, 32, teamUtil.alliance==RED ? 30 : 330, 180,0); // was fieldSide() + 300
            drive.moveCm(drive.MAX_VELOCITY, 11, driverSide(), 180,0);
            drive.moveCm(drive.MAX_VELOCITY, 10, 180, 180,0);
            if (!drive.strafeToEncoder(fieldSide(), 180, 400, (teamUtil.alliance==RED ? 1 : -1)*9200, 2000)) { // tweaked encoder target a bit
                drive.stopMotors();
                return false;
            }
            drive.stopMotors();
            teamUtil.pause(250);
            if(operateArms){
                intake.startIntake();
            }
            drive.moveCm(drive.MAX_VELOCITY, 84, 180, 180,0);
            */
        } else { // Path 2, middle for either Alliance
            drive.moveCm(drive.MAX_VELOCITY,86, fieldSide(), 180,0);
            if (!drive.strafeToEncoder(driverSide(), 180, 400, (teamUtil.alliance==RED ? 1 : -1)*9400, 2000)) { // tweaked encoder target a bit
                drive.stopMotors();
                return false;
            }
            drive.stopMotors();
            teamUtil.pause(250);
            if(operateArms){
                intake.startIntake();
            }
            drive.moveCm(drive.MAX_VELOCITY,64, 180, 180,0);
        }

        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public boolean cycleV3(double xOffset, boolean operateArms, int path) {
        long startTime = System.currentTimeMillis();
        teamUtil.log("Start Cycle");
        drive.switchCV(Drive.cvCam.FRONT_LINE);
        /*
        if(!lastTime&&(path==3&&teamUtil.alliance== teamUtil.alliance.RED)||!lastTime&&(path==1&&teamUtil.alliance== teamUtil.alliance.BLUE)){
            drive.moveCm(drive.MAX_VELOCITY, drive.TAG_CENTER_TO_CENTER, fieldSide(), 180, 800);
            xOffset=0;
        }

         */
        drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int desiredStrafeEncoder;
        if(teamUtil.alliance == teamUtil.alliance.RED){
            desiredStrafeEncoder=(int) (xOffset*drive.TICS_PER_CM_STRAFE+61*drive.TICS_PER_CM_STRAFE);
        }else{
            desiredStrafeEncoder=(int) (xOffset*drive.TICS_PER_CM_STRAFE-61*drive.TICS_PER_CM_STRAFE);
        }
        teamUtil.log("Desired Strafe Encoder: " + desiredStrafeEncoder);

        // Strafe and drive across the field to where we can almost see the white tape
        double distanceOffset = teamUtil.alliance==RED ? xOffset : -xOffset; // flip the sign on the Xoffset so the following math works on both sides
        drive.moveCm(drive.MAX_VELOCITY, 100 + (distanceOffset > 0 ? 1 : -1) * (Math.sqrt(distanceOffset * distanceOffset * 2)), teamUtil.alliance==RED ? 135:225 , 180, 500); // Heading was fixed at 135
        drive.moveStraightCmWithStrafeEncoder(drive.MAX_VELOCITY, 188 - distanceOffset, desiredStrafeEncoder,180, 180, 350); // was 183
        if (operateArms) {
            intake.startIntake();
            intake.ready();
        }

        if(!drive.driveToStackNoStopWithStrafe(180, 180, 1000, 5000)){
            intake.stopIntake();
            return false;
        }
        teamUtil.log("Line Detector FPS: " + drive.frontVisionPortal.getFps());

        drive.stopCV(); // shut down line detector early to help april tag detector get started
        if (operateArms) {
            intake.grabTwoPixels();
        } else {
            teamUtil.pause(1000);//grabTwoPixels

        }
        intake.autoOffLoopNoWait(3000);
        drive.switchCV(Drive.cvCam.REAR_APRILTAG);
        drive.moveCm(drive.MAX_VELOCITY, 210, 0, 180, 1000);
        if (operateArms) {
            //if((teamUtil.SIDE==teamUtil.Side.WING&&path==1&&teamUtil.alliance==teamUtil.alliance.RED)||teamUtil.SIDE==teamUtil.Side.WING&&path==3&&teamUtil.alliance==teamUtil.alliance.BLUE){
                //output.goToScoreNoWait(3.5,output.GrabberRotatorHorizontal2);
            //}else{
                output.goToScoreNoWait(3.5f,output.GrabberRotatorHorizontal2);
            //}
             // TODO Adjust height for different paths?
        }
        drive.moveCm(drive.MAX_VELOCITY, 48, teamUtil.alliance==RED ? 300 : 60, 180, 1000); // Heading was fixed at 300
        if(!drive.driveToAprilTagOffset(1000, 0, 180, teamUtil.alliance==RED ? -drive.TAG_CENTER_TO_CENTER : drive.TAG_CENTER_TO_CENTER, 30, 4000)){
            return false;
        }
        teamUtil.log("April Tag FPS: " + drive.rearVisionPortal.getFps());
        drive.stopCV(); // shut down April Tag detector early to help next detector get started

        drive.moveCm(drive.MAX_VELOCITY,17, 0, 180, 0);
        // Old version using tape
        //drive.moveCm(drive.MAX_VELOCITY, 9, 0, 180, 0);
        //drive.driveToTapeSetPower(.1f, 3000);

        long cycleTime = System.currentTimeMillis() - startTime;
        teamUtil.log("cycleTime: " + cycleTime); // without blocking GoToLoad at end

        if (operateArms) {
            output.dropAndGoToLoadNoWait();
        } else {
            teamUtil.pause(100);
        }

        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    public void autoV3(int path, boolean operateArms, boolean cycle) {
        teamUtil.log("Running Auto Path: " + path + " Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE") + " Side: " + teamUtil.SIDE);

        drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setHeading(180); // Zero is towards the scoring side of field
        long startTime = System.currentTimeMillis();

        if (teamUtil.SIDE == teamUtil.Side.SCORE) {
            drive.switchCV(Drive.cvCam.REAR_APRILTAG);
            // Get AprilTag Finder up and running
            if (operateArms) {
                output.goToScoreNoWait(1.5f,output.GrabberRotatorHorizontal2); // TODO: Adjust this level for higher consistency?
            }
            if (!pushPurplePixelScoreV3(path)) { //pushes pixel and gets to good location for April tag localization
                return;   //Auto Bailout
            }
        } else { // WING Side
            if (!pushPurplePixelWingV3(path,operateArms)) { // pushes pixel and drives to stack
                return;   //Auto Bailout
            }

            if (operateArms) {
                intake.grabOnePixel();
            } else {
                teamUtil.pause(1000);// emulate the grab
            }



            drive.switchCV(Drive.cvCam.REAR_APRILTAG);
            intake.autoOffLoopNoWait(3000);


            // Drive around the purple pixel (could be optimized for different paths)
            drive.moveCm(400, 1.5, 0, 180, 400);

            drive.moveCm(drive.MAX_VELOCITY,51, fieldSide(), 180, 800);
            intake.stopIntake();
            // Head to other side of field
            drive.moveCm(drive.MAX_VELOCITY,220, 0, 180,800);
            if (operateArms) {
                output.goToScoreNoWait(3,output.GrabberRotatorLoad);
            }
            // Get to where we can see the AprilTags well
            drive.moveCm(drive.MAX_VELOCITY,65, teamUtil.alliance==RED? 285 : 75, 180, 0); // TODO: SPEED UP: Maybe don't end at at stop
        }
        teamUtil.log("April Tag FPS: " + drive.rearVisionPortal.getFps());
        // Line up for the Yellow Pixel Drop on the correct location
        //if(true)return; //temporary for test
        double xOffset = path == 2 ? 0 : (path == 1 ? -drive.TAG_CENTER_TO_CENTER : drive.TAG_CENTER_TO_CENTER); // TODO Adjust this for better consistency?
        drive.driveToAprilTagOffset(1000, 0, 180, xOffset, 30, 3000);
        drive.stopCV();
        drive.moveCm(drive.MAX_VELOCITY,17, 0, 180, 0);

        if (operateArms) {
            output.dropAndGoToLoadNoWait();
            teamUtil.pause(500);
        } else {
            teamUtil.pause(100);
        }
        if(cycle){
            if(cycleV3(xOffset, operateArms,path)){
                teamUtil.pause(500);
                if(teamUtil.SIDE == teamUtil.Side.SCORE) {
                    cycleV3(teamUtil.alliance == RED ? -drive.TAG_CENTER_TO_CENTER : drive.TAG_CENTER_TO_CENTER, operateArms, path);
                }
            }else{
                teamUtil.log("Cycle Failed");

            }

        }

        else{
            teamUtil.pause(3000);
        }// One for now...
            //TODO check return value on cycle for failsafe issue
        long elapsedTime = System.currentTimeMillis() - startTime;
        teamUtil.log("elapsedTime: " + elapsedTime);

        //teamUtil.pause(30000-elapsedTime-500); // Allow output to get back to loading position with correct time
        teamUtil.log("Finished Auto");
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


}

