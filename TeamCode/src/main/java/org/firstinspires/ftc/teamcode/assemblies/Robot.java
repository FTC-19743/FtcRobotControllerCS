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
    public PixelRelease releaser;

    public Launcher launcher;

    public int straferDistanceFarStack = 17790;

    public double a,b,c,d;



    public Robot() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new Drive();
        intake = new Intake();
        output = new Output(intake);
        launcher = new Launcher();
        releaser = new PixelRelease();

        lift = new Lift();
    }

    public void initialize() {
        drive.initalize(output);
        intake.initalize();
        output.initialize();
        lift.initialize();
        releaser.initialize();
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
            drive.moveCm(drive.MAX_VELOCITY,83, teamUtil.alliance==RED ? 30:330, 180,0);
            /*
            drive.moveCm(drive.MAX_VELOCITY,67, fieldSide(), 180, 1000);
            drive.moveCm(drive.MAX_VELOCITY,30, teamUtil.alliance==RED ? 140 : 220, 180, 0); // was fieldSide() + 50
            drive.moveCm(drive.MAX_VELOCITY,10, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY, 78, 0, 180, 1000);

             */
        } else if ((teamUtil.alliance==RED && path == 3) || (teamUtil.alliance==BLUE && path == 1)) { // Near the backdrop
            drive.moveCm(drive.MAX_VELOCITY,68,teamUtil.alliance==RED?58:302, 180, 0);
            drive.moveCm(drive.MAX_VELOCITY,13, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY,  23 , teamUtil.alliance==RED?22.5:337.5, 180, 0);
        } else { // Path 2, middle for either Alliance
            drive.moveCm(drive.MAX_VELOCITY, 86, fieldSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY, 8.5, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY, 45, 0, 180, 0);
        }

        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);

        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SCORE Side Start - Push the purple pixel and get into position for AprilTag Localization based on path
    public boolean pushPurplePixelScoreV4(int path) {

        teamUtil.log("Pushing Pixel and Driving to April Tag Viewing Location");
        // TODO: SPEED UP IDEAS: Maybe hold the pixel in the collectors and take more direct paths, Maybe don't stop between all the movements
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.VIOLET);
        if ((teamUtil.alliance==RED && path == 1) || (teamUtil.alliance==BLUE && path == 3)) { // Under the Rigging
            drive.moveCm(drive.MAX_VELOCITY,71,fieldSide(), 180,0); // Was 71
            drive.moveCm(drive.MAX_VELOCITY,30, teamUtil.alliance==RED ? 155:215, 180,0);
            drive.moveCm(drive.MAX_VELOCITY,8, driverSide(), 180, 800);
            drive.moveCm(drive.MAX_VELOCITY,83, teamUtil.alliance==RED ? 30:330, 180,0);
            /*
            drive.moveCm(drive.MAX_VELOCITY,67, fieldSide(), 180, 1000);
            drive.moveCm(drive.MAX_VELOCITY,30, teamUtil.alliance==RED ? 140 : 220, 180, 0); // was fieldSide() + 50
            drive.moveCm(drive.MAX_VELOCITY,10, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY, 78, 0, 180, 1000);

             */
        } else if ((teamUtil.alliance==RED && path == 3) || (teamUtil.alliance==BLUE && path == 1)) { // Near the backdrop
            drive.moveCm(drive.MAX_VELOCITY,68,teamUtil.alliance==RED?75:300, 180, 0);
            drive.moveCm(drive.MAX_VELOCITY,13, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY,  23 , teamUtil.alliance==RED?22.5:337.5, 180, 0);
        } else { // Path 2, middle for either Alliance
            drive.moveCm(drive.MAX_VELOCITY, 86, fieldSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY, 8.5, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY, 75, teamUtil.alliance == RED ? 340: 20, 180, 0);
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

    public boolean driveToBackDrop (int path, boolean operateArms, int encoderCenterTile, double initialDistance, float level, double rotatorPos, double straferPos) {
        int finishEncoderStrafe;
        if (teamUtil.alliance==RED) {
            finishEncoderStrafe = path==1? encoderCenterTile-4500 : path==2 ? encoderCenterTile-6400: encoderCenterTile-7500;
        } else {
            finishEncoderStrafe =path==1? encoderCenterTile+7500 : path==2 ? encoderCenterTile+6400: encoderCenterTile+4500;
        }
        double transitionVelocity = path==1? 650 : path==2 ? 950: 1250;
        double seekVelocity = 400;

        // Move across the field while holding the center of the tile
        drive.moveStraightCmWithStrafeEncoder(2300,initialDistance,encoderCenterTile,0,180, transitionVelocity);

        // launch the output in a seperate thread
        if (operateArms) {output.goToScoreNoWait(level, rotatorPos, straferPos);}

        // Strafe over to a good starting point for April Tag Localization
        drive.strafeToEncoderWithDecel(driverSide(), 180, transitionVelocity, finishEncoderStrafe, seekVelocity,drive.MAX_STRAFE_DECELERATION,2000);

        // Localize using the April Tags
        if (drive.driveToAprilTagOffsetV2(seekVelocity,driverSide(),180,path==1? -drive.TAG_CENTER_TO_CENTER : path==2 ? 0: drive.TAG_CENTER_TO_CENTER,20,5000))
        {
            // Compute the final movement to the backdrop location for pixel drop
            drive.stopCV();
            teamUtil.log("Final Offset x/y: " + drive.lastAprilTagOffset.x + "/" + drive.lastAprilTagOffset.y);
            drive.backToPoint(180,drive.lastAprilTagOffset.x, 6+ drive.lastAprilTagOffset.y, 0 );
            return true;
        } else {
            // April tag localization failed
            drive.stopCV();
            drive.stopMotors();
            output.dropAndGoToLoad();
            return false;
        }
    }

    public boolean driveToBackDropV2 (int path, boolean operateArms, int encoderCenterTile, double initialDistance, float level, double rotatorPos, double straferPos) {
        // TODO: Optimize Inside path to save another .5-1 seconds on cycle
        int finishEncoderStrafe;
        double transitionVelocity;
        if (teamUtil.alliance==RED) {
            finishEncoderStrafe = path==1? encoderCenterTile-4000 : path==2 ? encoderCenterTile-5800: encoderCenterTile-6900;
            transitionVelocity = path==1? 650 : path==2 ? 950: 1250;
        } else {
            finishEncoderStrafe =path==1? encoderCenterTile+6900 : path==2 ? encoderCenterTile+5800: encoderCenterTile+4000;
            transitionVelocity = path==3? 650 : path==2 ? 950: 1250;
        }
        double seekVelocity = 400;

        // Move across the field while holding the center of the tile
        drive.moveStraightCmWithStrafeEncoder(2300,initialDistance,encoderCenterTile,0,180, transitionVelocity);

        // launch the output in a separate thread
        if (operateArms) {output.goToScoreNoWait(level, rotatorPos, straferPos);}

        // Strafe over to a good starting point for April Tag Localization
        drive.strafeToEncoderWithDecel(driverSide(), 180, transitionVelocity, finishEncoderStrafe, seekVelocity,drive.MAX_STRAFE_DECELERATION,2000);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.NORMAL_WHITE);

        // Localize using the April Tags
//        if (drive.strafeToAprilTagOffsetV2(seekVelocity,driverSide(),180,path==1? -drive.TAG_CENTER_TO_CENTER : path==2 ? 0: drive.TAG_CENTER_TO_CENTER,20,5000))
        if (drive.strafeToAprilTagOffsetV3(seekVelocity,5,driverSide(),180,path==1? -drive.TAG_CENTER_TO_CENTER : path==2 ? 0: drive.TAG_CENTER_TO_CENTER,20,5000))
        {
            // Compute the final movement to the backdrop location for pixel drop
            drive.stopCV();
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
            teamUtil.log("Final Offset x/y: " + drive.lastAprilTagOffset.x + "/" + drive.lastAprilTagOffset.y);
            drive.backToPoint(180,drive.lastAprilTagOffset.x, 6+ drive.lastAprilTagOffset.y, 0 );
            return true;
        } else {
            // April tag localization failed
            drive.stopCV();
            drive.stopMotors();
            output.dropAndGoToLoad();
            return false;
        }
    }

    public boolean pushPurplePlaceYellowPixelWingV4(int path, boolean operateArms){
        long startTime = System.currentTimeMillis();
        drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.ready();
        intake.closeLid();
        if ((teamUtil.alliance==RED && path == 1) || (teamUtil.alliance==BLUE && path == 3)) { // Near the Stacks
            if(teamUtil.alliance == RED){
                //robot.drive.moveCm(2000,56+robot.a,90,180,650);

                drive.strafeToEncoderWithDecel(90,180,2000,7300,650, drive.MAX_STRAFE_DECELERATION,2000);
            }else{
                //robot.drive.moveCm(2000,59+robot.b,270,180,650);

                drive.strafeToEncoderWithDecel(270,180,2000,-7600,650, drive.MAX_STRAFE_DECELERATION,2000);
            }
            drive.moveCm(drive.MAX_VELOCITY,teamUtil.alliance == RED? 33:36,180,180,650);
            releaser.release();
            if (operateArms) {
                intake.startIntake();
            }
            drive.moveCm(650,4,180,180,650);
            drive.strafeToEncoderWithDecel(teamUtil.alliance == RED? 90:270,180,650, (teamUtil.alliance == RED? 1: -1)*12250,450, drive.MAX_STRAFE_DECELERATION,1500 );
            drive.moveCm(drive.MAX_VELOCITY,13,180,180,0);

            /*

            if(teamUtil.alliance == RED){
                drive.strafeToEncoder(124,180,2000,7000,2000); //timeout should probably be shorter //heading was 126,124

            }else{
                drive.strafeToEncoder(235,180,2000,-6600,2000); //timeout should probably be shorter //heading was 126,124

            }

            releaser.release();
            if (operateArms) {
                intake.startIntake();
            }
            drive.strafeToEncoder(teamUtil.alliance == RED? 110:250,180,500, (teamUtil.alliance == RED? 1: -1)*12500,2000); //was 12280
            drive.moveCm(drive.MAX_VELOCITY,12,180,180,0);

             */

        }
        else if(path == 2){
            /*
            drive.strafeToEncoder(teamUtil.alliance==RED?90:270,180,2300,(teamUtil.alliance == RED? 1:-1)*(6200), 10000);
            drive.strafeToEncoder(teamUtil.alliance == RED? 90:270, 180, 800, (teamUtil.alliance == RED? 1:-1)*(11300), 10000);

             */
            drive.strafeToEncoderWithDecel(teamUtil.alliance==RED?90:270,180,2300,(teamUtil.alliance == RED? 11225:-11375), 650, drive.MAX_DECELERATION,10000); //a is -75, b is 0, c is -12, d  is 12



            drive.moveCm(drive.MAX_VELOCITY,4,180,180,1000);
            releaser.release();


            if (operateArms) {
                intake.startIntake();
            }
            drive.moveCm(drive.MAX_VELOCITY,57,180,180,0);

        }

        else{
            drive.strafeToEncoder(teamUtil.alliance == RED? 90:270,180,1700,(teamUtil.alliance == RED? 1:-1)*(4100), 10000);
            drive.strafeToEncoder(teamUtil.alliance == RED? 90:270,180,600,(teamUtil.alliance == RED? 1:-1)*(8200), 10000);
            drive.moveCm(drive.MAX_VELOCITY,teamUtil.alliance == RED? 15:13,0,180,0);
            releaser.release();
            if (operateArms) {
                intake.startIntake();
            }
            drive.strafeToEncoder(teamUtil.alliance == RED? 135: 225,180,700,(teamUtil.alliance == RED? 1:-1)*(12500), 10000);
            drive.moveCm(drive.MAX_VELOCITY,44,180,180,0);
        }


        if(operateArms){
            intake.autoGrabOne();
        }
        else{
            teamUtil.pause(750);
        }

        drive.switchCV(Drive.cvCam.REAR_APRILTAG);

        drive.moveCm(drive.MAX_VELOCITY,2,0,180,750);
        intake.ready();
        drive.moveCm(drive.MAX_VELOCITY,15,0,180,750);
        int distance;
        if (teamUtil.alliance == teamUtil.alliance.RED){
            distance = path==1? 250 : path==2 ? 235: 215;
        }
        else{
            distance = path==1? 215 : path==2 ? 235: 250;

        }


        double rotation, strafe;
        if(path==2){
            if(teamUtil.alliance==RED){
                rotation = output.GrabberRotatorHorizontal2;
                strafe = output.StraferLoad+4*output.StraferPositionPerCm;
            }else{
                rotation = output.GrabberRotatorHorizontal1;
                strafe = output.StraferLoad-4*output.StraferPositionPerCm;
            }

        }else if(path==3){
            rotation = output.GrabberRotatorHorizontal2;
            strafe = output.StraferLoad+4*output.StraferPositionPerCm;

        }else{
            rotation = output.GrabberRotatorHorizontal1;
            strafe = output.StraferLoad-4.5*output.StraferPositionPerCm;
        }
        if(teamUtil.alliance == RED){
            drive.strafeToEncoder(90,180,1000,16700,2000);
        }else{
            drive.strafeToEncoder(270,180,1000,-15750,2000); //strafe value was 17560 when res
        }
        driveToBackDropV2(path, operateArms,17500* (teamUtil.alliance==RED ? 1 : -1),distance,3,rotation, strafe);

        teamUtil.log("AprilTagFPS" + drive.rearVisionPortal.getFps());


        long purpleYellowWingTime = System.currentTimeMillis() - startTime;
        teamUtil.log("purpleYellowWingTime: " + purpleYellowWingTime); // without blocking GoToLoad at end

        if (operateArms) {
            output.dropAndGoToLoadNoWait();
        } else {
            teamUtil.pause(100);
        }

//        teamUtil.pause(5000);
        intake.stopIntake();
        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public boolean cycleV4(double xOffset, boolean operateArms, int path,long autoStartTime){
        long startTime = System.currentTimeMillis();
        teamUtil.log("Start Cycle");
        drive.switchCV(Drive.cvCam.FRONT_LINE);

        drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int desiredStrafeEncoderTransition;

        int desiredStrafeEncoderCenter;

        if(teamUtil.alliance == teamUtil.alliance.RED){
            desiredStrafeEncoderTransition=(int) (xOffset*drive.TICS_PER_CM_STRAFE_ENCODER+6800);
            desiredStrafeEncoderCenter=(int) (xOffset*drive.TICS_PER_CM_STRAFE_ENCODER+8100);
        }else{
            desiredStrafeEncoderTransition=(int) (xOffset*drive.TICS_PER_CM_STRAFE_ENCODER-6800);
            desiredStrafeEncoderCenter=(int) (xOffset*drive.TICS_PER_CM_STRAFE_ENCODER-8100);
        }
        drive.moveCm(drive.MAX_VELOCITY, 2,180,180,0);
        //drive.strafeToEncoder(teamUtil.alliance==RED?90:270,180,750 ,desiredStrafeEncoderTransition,2000);
        drive.strafeToEncoderWithDecel(teamUtil.alliance==RED?90:270,180,2300 ,desiredStrafeEncoderTransition,750,drive.MAX_STRAFE_DECELERATION,2000);
        //double distanceOffset = teamUtil.alliance==RED ? xOffset : -xOffset; // flip the sign on the Xoffset so the following math works on both sides
        //drive.moveCm(drive.MAX_VELOCITY, 75 + (distanceOffset > 0 ? 1 : -1) * (Math.sqrt(distanceOffset * distanceOffset * 2)), teamUtil.alliance==RED ? 135:225 , 180, 1000); // Heading was fixed at 135 // a was 500 b was 100
        drive.moveStraightCmWithStrafeEncoder(drive.MAX_VELOCITY, 243, desiredStrafeEncoderCenter,180, 180, 1000); // velocity at end was 700 // c was 188

        if (operateArms) {
            intake.startIntake();
        }
        intake.ready();
        drive.driveToStackNoStopWithStrafeV2(180, 180, 1000, 5000);

        if (operateArms) {

            intake.automaticGrabTwoNoWait();
            teamUtil.pause(250);
        } else {
            teamUtil.pause(250);//Use the same amount of time
        }
         // TODO: Need to verify this is long enough after driveToStack shut down the line detector!

        // Drive to where rear camera can easily see the inside AprilTag deploying output when safe


        drive.switchCV(Drive.cvCam.REAR_APRILTAG);
        drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int distance = 285;
        if(!driveToBackDropV2(teamUtil.alliance == RED? 1:3, operateArms,0,distance,3.5f,output.GrabberRotatorHorizontal2, output.StraferLoad)){
            drive.stopMotors();
            teamUtil.log("Drive To BackDropV2 Failed");
            return false;
        }



        if (operateArms) {
            //intake.stopIntake();
            output.dropAndGoToLoadNoWait();
        } else {
            teamUtil.pause(100);
        }

        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        long endTime = System.currentTimeMillis();
        long cycleTime = endTime-startTime;
        teamUtil.log("Cycle Time: "+ cycleTime);

        return true;
    }
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
            desiredStrafeEncoder=(int) (xOffset*drive.TICS_PER_CM_STRAFE_ENCODER+61*drive.TICS_PER_CM_STRAFE_ENCODER);
        }else{
            desiredStrafeEncoder=(int) (xOffset*drive.TICS_PER_CM_STRAFE_ENCODER-61*drive.TICS_PER_CM_STRAFE_ENCODER);
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
                output.goToScoreNoWait(3.5f,output.GrabberRotatorHorizontal2,output.StraferLoad);
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
    public void autoV4(int path, boolean operateArms, int delaySeconds){
        long startTime = System.currentTimeMillis();
        teamUtil.log("Running Auto Path: " + path + " Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE") + " Side: " + teamUtil.SIDE);
        teamUtil.log("Delay Time Seconds" + delaySeconds*1000);

        teamUtil.pause(delaySeconds*1000);


        drive.setHeading(180); // Zero is towards the scoring side of field
        if(teamUtil.SIDE== teamUtil.Side.WING){
            pushPurplePlaceYellowPixelWingV4(path,operateArms);
        }else{ //score case
            //drive.switchCV(Drive.cvCam.REAR_APRILTAG);
            // Get AprilTag Finder up and running
            if (operateArms) {
                output.goToScoreNoWait(1.5f,output.GrabberRotatorHorizontal2,output.StraferLoad); // TODO: Adjust this level for higher consistency?
            }
            if (!pushPurplePixelScoreV4(path)) { //pushes pixel and gets to good location for April tag localization
                return;   //Auto Bailout
            }
            if (true) return ;

        }

        double xOffset = path == 2 ? 0 : (path == 1 ? -drive.TAG_CENTER_TO_CENTER : drive.TAG_CENTER_TO_CENTER);
        if(System.currentTimeMillis()-startTime<20000){
            if(cycleV4(xOffset,operateArms,path,startTime)){
                if(System.currentTimeMillis()-startTime<21000){
                    if(cycleV4(teamUtil.alliance==teamUtil.alliance.RED? -drive.TAG_CENTER_TO_CENTER :drive.TAG_CENTER_TO_CENTER,operateArms,path,startTime)){

                    }
                    else{
                        teamUtil.log("First CycleV4 failed");
                    }
                }else{
                    teamUtil.log("Second Cycle AutoV4 Timed Out");
                }
            }
        }

        else{
            teamUtil.log("Cycle V4 Failsafed Out");
        }

    }


    public void autoV3(int path, boolean operateArms, boolean cycle) {
        teamUtil.log("Running Auto Path: " + path + " Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE") + " Side: " + teamUtil.SIDE);

        drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setHeading(180); // Zero is towards the scoring side of field
        long startTime = System.currentTimeMillis();

        if (teamUtil.SIDE == teamUtil.Side.SCORE) {
            drive.switchCV(Drive.cvCam.REAR_APRILTAG);
            // Get AprilTag Finder up and running
            if (operateArms) {
                output.goToScoreNoWait(1.5f,output.GrabberRotatorHorizontal2,output.StraferLoad); // TODO: Adjust this level for higher consistency?
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
                output.goToScoreNoWait(3,output.GrabberRotatorLoad,output.StraferLoad);
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

