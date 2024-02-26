package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.opencv.core.Point;

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





    public boolean driveToBackDropInsideFast (boolean operateArms, int encoderCenterTile) { // TODO: Work in Progress!
        boolean details = true;
        teamUtil.log("driveToBackDropInsideFast");
        int finishEncoderStrafe;
        double transitionVelocity;
        double rotatorPos = output.GrabberRotatorLoad;
        double straferPos = output.StraferLoad;
        float level = 3; // is this supposed to be 3.5?

        finishEncoderStrafe = teamUtil.alliance==RED? encoderCenterTile-4000 : encoderCenterTile+4000;
        transitionVelocity =  1000;

        double seekVelocity = 450+teamUtil.robot.c; //850 velocity was consistent with extra 3-3.5 cm of drift
        double driftFactor = 5+teamUtil.robot.d; // cm assuming seek speed of 450
        // Move across the field while holding the center of the tile
        drive.driveStraightToTargetWithStrafeEncoderValue(drive.MAX_VELOCITY-200,-150000+teamUtil.robot.b,encoderCenterTile,0,180, transitionVelocity,3000);

        // launch the output in a separate thread
        if (operateArms) {output.goToScoreNoWait(level,rotatorPos, straferPos);}

        // Strafe over to a good starting point for April Tag Localization
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.NORMAL_WHITE); // turn on the headlights for better vision

        drive.strafeToTarget(transitionVelocity, finishEncoderStrafe, teamUtil.robot.a + (teamUtil.alliance==RED?315:45), 180, seekVelocity,1500);
        drive.aprilTag.getFreshDetections(); // clear out any detections it saw on the way over

        // Look for some April Tags
        org.opencv.core.Point aprilTagOffset = new Point();
        boolean sawAprilTag = false;
        long aprilTagTimeoutTime = System.currentTimeMillis() + 1000;

        while (teamUtil.keepGoing(aprilTagTimeoutTime)) {
            if (details) teamUtil.log("Looking for April Tags");
            drive.driveMotorsHeadingsFR(driverSide(), 180, seekVelocity);
            if (drive.getRobotBackdropOffset(aprilTagOffset, true)) {
                sawAprilTag = true;
                break;
            }
        }
        double strafeTarget = 0;
        if (sawAprilTag) { // Use ONLY April Tag info to localize.  We COULD average this with the strafe encoder...
            strafeTarget = drive.strafeEncoder.getCurrentPosition() // start with our current position
                    + aprilTagOffset.x*drive.TICS_PER_CM_STRAFE_ENCODER // offset with distance to center of backdrop
                    + drive.TAG_CENTER_TO_CENTER*drive.TICS_PER_CM_STRAFE_ENCODER// adjust for path
                    + driftFactor *drive.TICS_PER_CM_STRAFE_ENCODER*(teamUtil.alliance== RED?1:-1); // adjust for drift

        } else {
            // No April Tag Detection so fail over to pure encoder approach
            teamUtil.log("FAILED to see AprilTags");
            // TODO!!
            drive.stopCV();
            drive.stopMotors();
            if (operateArms) {output.dropAndGoToLoad();}
            return false;
        }

        //-11125
        drive.strafeToTarget(seekVelocity, strafeTarget, teamUtil.robot.a + (teamUtil.alliance==RED?315:45), 180, 0,1500); // TODO: Non-zero end velocity?

        // Get an updated Y reading here
        if (!drive.getRobotBackdropOffset(aprilTagOffset, false)) {
            // TODO rely on forward encoder?
            teamUtil.log("FAILED to get updated AprilTag reading for y offset");
            drive.stopMotors();
            if(operateArms){
                output.dropAndGoToLoad();
            }
        }
        drive.stopCV();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        double strafeCm = 0;

        // launch the output in a separate thread
        if (operateArms) {output.goToScoreNoWait(level, rotatorPos, straferPos);}

        if (details) {teamUtil.log("Final Offset x/y: " + aprilTagOffset.x + "/" + aprilTagOffset.y);}
        // TODO: Consider using driveStraightToTargetWithStrafeEncoderValue with a target derived from aprilTagOffset.y
if (true) return true;
        drive.moveCm(drive.MAX_VELOCITY,-13+ aprilTagOffset.y,0,180,0); // -13 is magic
        teamUtil.log("driveToBackDropV3 ---FINISHED");
        return true;
    }

    public boolean driveToBackDropInsideFastEncodersOnly (boolean operateArms, int encoderCenterTile) {
        boolean details = true;
        teamUtil.log("driveToBackDropInsideFast");
        int strafeEncoderTarget =  teamUtil.alliance==RED? encoderCenterTile-5900 : encoderCenterTile+5900;
        int strafeDriftTarget = (int) (strafeEncoderTarget + drive.TICS_PER_CM_STRAFE_ENCODER* (10+ (int)teamUtil.robot.d)); // TODO Fix for Blue side
        int straightEncoderTarget =  -205325+(int)teamUtil.robot.a;
        double straightDistance = -142000;
        double transitionVelocity1 = 1500+teamUtil.robot.b;
        double transitionVelocity2 = 1000+teamUtil.robot.c;
        double angle= 38;
        double rotatorPos = output.GrabberRotatorLoad;
        double straferPos = output.StraferLoad;
        float level = 3; // is this supposed to be 3.5?

        double driftFactor = 5+teamUtil.robot.d; // cm assuming seek speed of 450

        // Move across the field while holding the center of the tile
        drive.driveStraightToTargetWithStrafeEncoderValue(drive.MAX_VELOCITY-200,straightDistance,encoderCenterTile,0,180, transitionVelocity1,3000);
        teamUtil.log("strafe: " + drive.strafeEncoder.getCurrentPosition() + " forward: " + drive.forwardEncoder.getCurrentPosition());

        // launch the output in a separate thread
        if (operateArms) {output.goToScoreNoWait(level,rotatorPos, straferPos);}
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.NORMAL_WHITE);

        drive.strafeToTarget(transitionVelocity1*1.3, strafeDriftTarget, (teamUtil.alliance==RED?270+angle:angle), 180, transitionVelocity2,1500);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        teamUtil.log("strafe: " + drive.strafeEncoder.getCurrentPosition() + " forward: " + drive.forwardEncoder.getCurrentPosition());

        drive.driveStraightToTargetWithStrafeEncoderValue(transitionVelocity2, straightEncoderTarget,strafeEncoderTarget, 0, 180, 0,1500);
        teamUtil.log("strafe: " + drive.strafeEncoder.getCurrentPosition() + " forward: " + drive.forwardEncoder.getCurrentPosition());
        teamUtil.log("driveToBackDropV3 ---FINISHED");
        return true;
    }

    public boolean driveToBackDropV5(int path, boolean operateArms, int encoderCenterTile, int backupTarget, boolean yellowPixel) {
        // TODO: Optimize Inside path to save another .5-1 seconds on cycle
        boolean details = true;
        teamUtil.log("driveToBackDropV5");
        int finishEncoderStrafe;
        double transitionVelocity;
        double rotatorPos;
        double straferPos;
        float level;
        if(yellowPixel){
            rotatorPos=output.GrabberRotatorLoad;
            straferPos=output.StraferLoad;
            level = 3;
        }else{
            rotatorPos = output.GrabberRotatorHorizontal2;
            straferPos = output.StraferLoad;
            level = 3.5f;
        }



        if (teamUtil.alliance==RED) {
            finishEncoderStrafe = path==1? encoderCenterTile-4000 : path==2 ? encoderCenterTile-5800: encoderCenterTile-6900;
            transitionVelocity = path==1? 650 : path==2 ? 950: 1250;
        } else {
            finishEncoderStrafe =path==1? encoderCenterTile+6900 : path==2 ? encoderCenterTile+5800: encoderCenterTile+4000;
            transitionVelocity = path==3? 650 : path==2 ? 950: 1250;
        }
        double seekVelocity = 450+teamUtil.robot.c; //850 velocity was consistent with extra 3-3.5 cm of drift
        double driftFactor = 5+teamUtil.robot.d; // cm assuming seek speed of 450
        // Move across the field while holding the center of the tile
        drive.driveStraightToTargetWithStrafeEncoderValue(drive.MAX_VELOCITY-200,backupTarget,encoderCenterTile,0,180, transitionVelocity,3000);


        // launch the output in a separate thread
        if (operateArms) {output.goToScoreNoWait(level,rotatorPos, straferPos);}

        // Strafe over to a good starting point for April Tag Localization
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.NORMAL_WHITE); // turn on the headlights for better vision

        drive.strafeToTarget(transitionVelocity, finishEncoderStrafe, driverSide(), 180, seekVelocity,1500);
        drive.aprilTag.getFreshDetections(); // clear out any detections it saw on the way over

        // Look for some April Tags
        org.opencv.core.Point aprilTagOffset = new Point();
        boolean sawAprilTag = false;
        long aprilTagTimeoutTime = System.currentTimeMillis() + 1000;

        while (teamUtil.keepGoing(aprilTagTimeoutTime)) {
            if (details) teamUtil.log("Looking for April Tags");
            drive.driveMotorsHeadingsFR(driverSide(), 180, seekVelocity);
            if (drive.getRobotBackdropOffset(aprilTagOffset, true)) {
                sawAprilTag = true;
                break;
            }
        }
        double strafeTarget = 0;
        if (sawAprilTag) { // Use ONLY April Tag info to localize.  We COULD average this with the strafe encoder...
            strafeTarget = drive.strafeEncoder.getCurrentPosition() // start with our current position
                    + aprilTagOffset.x*drive.TICS_PER_CM_STRAFE_ENCODER // offset with distance to center of backdrop
                    + (path==1? drive.TAG_CENTER_TO_CENTER*drive.TICS_PER_CM_STRAFE_ENCODER: path==3 ? -drive.TAG_CENTER_TO_CENTER*drive.TICS_PER_CM_STRAFE_ENCODER: 0) // adjust for path
                    + driftFactor *drive.TICS_PER_CM_STRAFE_ENCODER*(teamUtil.alliance== RED?1:-1); // adjust for drift

        } else {
            // TODO: No April Tag Detection so fail over to pure encoder approach?
            teamUtil.log("FAILED to see AprilTags");
            drive.stopCV();
            drive.stopMotors();
            if (operateArms) {output.dropAndGoToLoad();}
            return false;
        }

        //-11125
        Drive.YellowPixelPosition position = Drive.YellowPixelPosition.NONE;
        if (yellowPixel) { drive.rearVisionPortal.setProcessorEnabled(drive.findPixelProcesser,true); }
        drive.strafeToTarget(seekVelocity, strafeTarget, driverSide(), 180, 0,1500); // TODO: Non-zero end velocity?
        if (yellowPixel) {
            drive.findPixelProcesser.reset();
            position = drive.findYellowPixel(path,100);
            teamUtil.log("Yellow Pixel Detection: " + position);
        }

        // Get an updated Y reading here
        if (!drive.getRobotBackdropOffset(aprilTagOffset, false)) {
            // TODO rely on forward encoder?
            teamUtil.log("Using Previous Y Offset");
        }
        if (yellowPixel) {drive.rearVisionPortal.setProcessorEnabled(drive.findPixelProcesser,false);}
        drive.stopCV();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);

        if(yellowPixel){
            double strafeCm = 0;
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

            if(operateArms){
                output.grabberRotater.setPosition(rotation);
                output.grabberStrafer.setPosition(strafe);
            }

            boolean blueSidePathOne;
            if(teamUtil.alliance==BLUE&&path==1){
                blueSidePathOne=true;
            }else{
                blueSidePathOne=false;
            }
            if(position == Drive.YellowPixelPosition.NONE&&operateArms){
                if(blueSidePathOne){
                    teamUtil.log("Blue Side Path One so no Level Change");
                }else{
                    teamUtil.log("No Yellow Pixel Adjusting To Level 2");
                    level = 2;
                    while(output.moving.get()){
                        teamUtil.pause(50);
                    }

                    output.runElevatorsToLevel(level);
                }


            }
        }

        drive.moveCm(drive.MAX_VELOCITY,aprilTagOffset.y-13.5,0,180,0); // -13 is magic
        teamUtil.log("driveToBackDropV5 ---FINISHED");
        return true;
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    /*
    if(yellowPixel){
            if(position == Drive.YellowPixelPosition.NONE){
                level = 2;
                if(teamUtil.alliance == RED){
                    if(path == 3){
                        rotatorPos = output.GrabberRotatorHorizontal2;
                    }
                    else{
                        rotatorPos = output.GrabberRotatorHorizontal1;
                    }
                }
                else{
                    if(path == 1){
                        rotatorPos = output.GrabberRotatorHorizontal1;
                    }
                    else{
                        rotatorPos = output.GrabberRotatorHorizontal2;
                    }
                }
            }
            else if(position == Drive.YellowPixelPosition.LEFT){ // LEFT
                if(path == 1){
                    rotatorPos = output.GrabberRotatorHorizontal1;
                    straferPos = 4; // TODO: positive or negative?
                    strafeCm = 3.5;
                }
                else if(path == 2){
                    rotatorPos =output.GrabberRotatorHorizontal1;
                    straferPos = 3;
                    strafeCm = 2;
                }
                else{
                    rotatorPos = output.GrabberRotatorHorizontal2;
                    straferPos = -3.75;
                }
            }
            else{ // RIGHT
                if(path == 1){
                    rotatorPos = output.GrabberRotatorHorizontal1;
                    straferPos = 3.75;
                }
                else if (path == 2){
                    rotatorPos = output.GrabberRotatorHorizontal2;
                    straferPos = -3;
                    strafeCm = -2;
                }
                else{
                    rotatorPos = output.GrabberRotatorHorizontal2;
                    straferPos = -4;
                    strafeCm = -3.5;
                }
            }
        }
        teamUtil.log("Rotator Pos" + rotatorPos);
        teamUtil.log("Strafer Pos" + straferPos);
        teamUtil.log("Strafer Cm" + strafeCm);

     */

    public boolean pushPurplePlaceYellowPixelWingV5(int path, boolean operateArms){
        long startTime = System.currentTimeMillis();
        drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.ready();
        intake.closeLid();
        if ((teamUtil.alliance==RED && path == 1) || (teamUtil.alliance==BLUE && path == 3)) { // Near the Stacks
            if(teamUtil.alliance == RED){
                drive.strafeToTarget(2000,7100,90,180,650,2000);
            }else{
                drive.strafeToTarget(2000,-7100,270,180,650,2000);
            }
            drive.driveStraightToTarget(1000+b,teamUtil.alliance == RED? 24750: 27250,180,180,0,2000);
            releaser.release();
            if (operateArms) { intake.startIntake();}
            drive.driveStraightToTarget(650,teamUtil.alliance == RED? 31205: 34000,180,180,650,2000);
            drive.strafeToTarget(650,(teamUtil.alliance == RED? 1: -1)*12250,teamUtil.alliance == RED? 90:270,180,450,1500);
            drive.driveStraightToTarget(drive.MAX_VELOCITY,42000,180,180,400,2000);
            drive.driveMotorsHeadingsFR(180,180,400);
            drive.waitForStall(1500);
        }
        else if(path == 2){
            drive.strafeToTarget(2000,(teamUtil.alliance == RED? 11225:-11375),teamUtil.alliance==RED?90:270,180,650,10000);
            drive.moveCm(drive.MAX_VELOCITY,4,180,180,1000);
            releaser.release();
            if (operateArms) { intake.startIntake();}
            drive.driveStraightToTargetWithStrafeEncoderValue(drive.MAX_VELOCITY,42000,(teamUtil.alliance == RED? 1:-1)*13000,180,180,400,2000);
            drive.driveMotorsHeadingsFR(180,180,400);
            drive.waitForStall(1500);
        } else{
            drive.strafeToEncoderWithDecel(teamUtil.alliance==RED?90:270,180,1700,(teamUtil.alliance == RED? 1:-1)*(8200), 600, drive.MAX_DECELERATION,10000);
            drive.moveCm(drive.MAX_VELOCITY,teamUtil.alliance == RED? 15:13,0,180,0);
            releaser.release();
            if (operateArms) { intake.startIntake();}
            drive.strafeToTarget(2000,(teamUtil.alliance == RED? 11000:-11000),teamUtil.alliance == RED? 135: 225,180,1500+b,10000);
            drive.driveStraightToTargetWithStrafeEncoderValue(drive.MAX_VELOCITY-200,42000,(teamUtil.alliance == RED? 1: -1)*13000,180,180,400,2000);
            drive.driveMotorsHeadingsFR(180,180,400);
            drive.waitForStall(1500);
        }
        drive.setMotorsRunWithoutEncoder();
        drive.setMotorPower(0.05);


        drive.forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // on audience wall
        if(operateArms){
            intake.autoGrabOne();
        }
        else{
            teamUtil.pause(750);
        }
        drive.switchCV(Drive.cvCam.REAR_APRILTAG); // Start the April Tag camera
        drive.setMotorsWithEncoder();

        // release the 2nd pixel
        drive.moveCm(drive.MAX_VELOCITY,4,0,180,750); //cm was 2
        intake.ready();
        drive.moveCm(drive.MAX_VELOCITY,13,0,180,750); //cm was 4

        if(teamUtil.alliance == RED){
            drive.strafeToEncoder(90,180,1000,16700,2000);
        }else{
            drive.strafeToEncoder(270,180,1000,-15750,2000); //strafe value was 17560 when res
        }
        int backupTarget;
        if(teamUtil.alliance==RED){
            backupTarget = path==1? -188500-3500 : path==2 ? -177500-3500: -166500-3500; // add 15 //path 2 = -177500 path 1 =-166500
        }else{
            backupTarget = path==3? -188500+2000 : path==2 ? -177500+2000: -166500+2000; // add 15 //path 2 = -177500 path 1 =-166500
        }
        driveToBackDropV5(path, operateArms,17500* (teamUtil.alliance==RED ? 1 : -1),backupTarget,true);
        teamUtil.pause(250);
        drive.spinToHeading(180);

        teamUtil.log("AprilTagFPS" + drive.rearVisionPortal.getFps());

        long purpleYellowWingTime = System.currentTimeMillis() - startTime;
        teamUtil.log("purpleYellowWingTime: " + purpleYellowWingTime); // without blocking GoToLoad at end

        if (operateArms) {
            output.dropAndGoToLoadNoWait();
        } else {
            teamUtil.pause(100);
        }

        //teamUtil.pause(5000);
        intake.stopIntake();
        return true;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public boolean cycleV5(double xOffset, boolean operateArms, int path,long autoStartTime){
        long startTime = System.currentTimeMillis();
        teamUtil.log("Start Cycle-----------------------------------------");
        int desiredStrafeEncoderTransition;
        int desiredStrafeEncoderCenter;

        drive.switchCV(Drive.cvCam.FRONT_LINE);
        drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(teamUtil.alliance == teamUtil.alliance.RED){
            desiredStrafeEncoderTransition=(int) (xOffset*drive.TICS_PER_CM_STRAFE_ENCODER+6800);
            desiredStrafeEncoderCenter=(int) (xOffset*drive.TICS_PER_CM_STRAFE_ENCODER+8100);
        }else{
            desiredStrafeEncoderTransition=(int) (xOffset*drive.TICS_PER_CM_STRAFE_ENCODER-6800);
            desiredStrafeEncoderCenter=(int) (xOffset*drive.TICS_PER_CM_STRAFE_ENCODER-8100);
        }
        drive.moveCm(drive.MAX_VELOCITY, 2,180,180,0);
        drive.strafeToTarget(drive.MAX_VELOCITY-200, desiredStrafeEncoderTransition,teamUtil.alliance==RED?90:270,180, 750, 2500 );
        drive.driveStraightToTargetWithStrafeEncoderValue(drive.MAX_VELOCITY-200,-40000, desiredStrafeEncoderCenter,180, 180, 1000, 4000);
        if (operateArms) {
            intake.startIntake();
        }
        intake.ready();
        if (!drive.driveToStackNoStopWithStrafeV3(180,180, 1000, desiredStrafeEncoderCenter,false,3000)) {
            drive.stopMotors();
            return false;
        }
        drive.forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // on audience wall
        if (operateArms) {
            intake.automaticGrabTwoNoWait();
            teamUtil.pause(250);
        } else {
            teamUtil.pause(250);//Use the same amount of time
        }
        drive.switchCV(Drive.cvCam.REAR_APRILTAG); // start April Tag detector
        int backupTarget = (teamUtil.alliance==RED ? -188500 : -188500);

        if(!driveToBackDropV5(teamUtil.alliance == RED? 1:3, operateArms,desiredStrafeEncoderCenter,backupTarget ,false)){
            drive.stopMotors();
            teamUtil.log("Drive To BackDropV2 Failed");
            return false;
        }

        drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (operateArms) {
            output.dropAndGoToLoadNoWait();
        } else {
            teamUtil.pause(100);
        }

        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        long endTime = System.currentTimeMillis();
        long cycleTime = endTime-startTime;
        teamUtil.log("Cycle Time: "+ cycleTime + " ------------------------------------");

        return true;
    }



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void autoV5(int path, boolean operateArms, int delaySeconds, boolean cycle){
        long startTime = System.currentTimeMillis();
        teamUtil.log("Running Auto Path: " + path + " Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE") + " Side: " + teamUtil.SIDE);
        teamUtil.log("Delay Time Seconds" + delaySeconds*1000);

        teamUtil.pause(delaySeconds*1000);

        drive.setHeading(180); // Zero is towards the scoring side of field
        if(teamUtil.SIDE== teamUtil.Side.WING){
            pushPurplePlaceYellowPixelWingV5(path,operateArms);
        }else{ //score case
            //drive.switchCV(Drive.cvCam.REAR_APRILTAG);
            // Get AprilTag Finder up and running
            if (operateArms) {
                output.goToScoreNoWait(1.5f,output.GrabberRotatorHorizontal2,output.StraferLoad); // TODO: Adjust this level for higher consistency?
            }
            if (!pushPurplePlaceYellowPixelWingV5(path,operateArms)) { //pushes pixel and gets to good location for April tag localization
                return;   //Auto Bailout
            }
            if (true) return ;

        }
        if(cycle){
            double xOffset = path == 2 ? 0 : (path == 1 ? -drive.TAG_CENTER_TO_CENTER : drive.TAG_CENTER_TO_CENTER);
            if(System.currentTimeMillis()-startTime<20000){
                if(cycleV5(xOffset,operateArms,path,startTime)){
                    if(System.currentTimeMillis()-startTime<21000){
                        if(cycleV5(teamUtil.alliance==teamUtil.alliance.RED? -drive.TAG_CENTER_TO_CENTER :drive.TAG_CENTER_TO_CENTER,operateArms,path,startTime)){

                        }
                        else{
                            teamUtil.log("First CycleV5 failed");
                        }
                    }else{
                        teamUtil.log("Second Cycle AutoV5 Timed Out");
                    }
                }
            }

            else{
                teamUtil.log("Cycle V5 Failsafed Out");
            }
        }


    }
    public void autoV4(int path, boolean operateArms, int delaySeconds, boolean cycle){
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
        if(cycle){
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


    }




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
        drive.aprilTag.getFreshDetections(); // clear out any detections it saw on the way over
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
            drive.moveCm(drive.MAX_VELOCITY,6+ drive.lastAprilTagOffset.y,0,180,0); // ignore x offset to overcome drift due to arms
            //drive.backToPoint(180,drive.lastAprilTagOffset.x, 6+ drive.lastAprilTagOffset.y, 0 );
            return true;
        } else {
            // April tag localization failed
            drive.stopCV();
            drive.stopMotors();
            output.dropAndGoToLoad();
            return false;
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
        drive.moveCm(drive.MAX_VELOCITY,teamUtil.alliance==RED?13:16,180,180,0);

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
        drive.moveCm(drive.MAX_VELOCITY,teamUtil.alliance==RED?57:61,180,180,0);

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
        drive.moveCm(drive.MAX_VELOCITY,teamUtil.alliance==RED?44:45,180,180,0);
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
        distance = path==1? 215 : path==2 ? 237: 250;

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
    driveToBackDropV2(path, operateArms,17500* (teamUtil.alliance==RED ? 1 : -1),distance,2.5f,rotation, strafe);
    teamUtil.pause(250);

    drive.spinToHeading(180);

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
        drive.driveToStackNoStopWithStrafeV2(180, 180, 1000, 5000); // TODO: do something with the return value

        if (operateArms) {

            intake.automaticGrabTwoNoWait();
            teamUtil.pause(250); // TODO: What is this about?  Previous line is a "no wait" method
        } else {
            teamUtil.pause(250);//Use the same amount of time
        }
        // TODO: Need to verify this is long enough after driveToStack shut down the line detector!

        // Drive to where rear camera can easily see the inside AprilTag deploying output when safe


        drive.switchCV(Drive.cvCam.REAR_APRILTAG);
        drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int distance = (teamUtil.alliance==RED ? 285 : 283);
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
            //intake.grabTwoPixels();
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


}

