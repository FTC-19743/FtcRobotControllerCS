package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public int straferDistanceFarStack = 17790;

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);

    }

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
        drive.initalize();
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
        output.goToLoad(); // ready to load
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
        if ((teamUtil.alliance==RED && path == 1) || (teamUtil.alliance==BLUE && path == 3)) { // Under the Rigging
            drive.moveCm(drive.MAX_VELOCITY,67, fieldSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY,30, teamUtil.alliance==RED ? 140 : 220, 180, 0); // was fieldSide() + 50
            drive.moveCm(drive.MAX_VELOCITY,10, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY, 78, 0, 180, 1000);
        } else if ((teamUtil.alliance==RED && path == 3) || (teamUtil.alliance==BLUE && path == 1)) { // Near the backdrop
            drive.moveCm(drive.MAX_VELOCITY,80, fieldSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY,30, teamUtil.alliance==RED ? 30 : 330 , 180, 0); // was fieldSide() + 300 on RED
            drive.moveCm(drive.MAX_VELOCITY,13, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY,  23 , 0, 180, 1000);
        } else { // Path 2, middle for either Alliance
            drive.moveCm(drive.MAX_VELOCITY,86, fieldSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY,8.5, driverSide(), 180, 0);
            drive.moveCm(drive.MAX_VELOCITY, 45, 0, 180, 1000);
        }
        /*
        switch (path) { /
            case 1:
                drive.moveCm(67, fieldSide());
                drive.moveCm(30, fieldSide() + 50);
                drive.moveCm(10, driverSide());
                drive.moveCm(drive.MAX_VELOCITY, teamUtil.alliance == RED ? 78 : 23, 0, 180, 1000);
                break;
            case 2:
                drive.moveCm(86, fieldSide());
                drive.moveCm(8.5, driverSide());
                drive.moveCm(drive.MAX_VELOCITY, 45, 0, 180, 1000);
                break;
            case 3:
                drive.moveCm(80, fieldSide());
                drive.moveCm(30, fieldSide() + 300);
                drive.moveCm(13, driverSide());
                drive.moveCm(drive.MAX_VELOCITY, teamUtil.alliance == RED ? 23 : 78, 0, 180, 1000);
                break;
        }
        */
        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // WING Side Start - Push the purple pixel and drive to nearest stack
    // TODO: SPEED UP IDEAS: Make the pixel pusher less deep and try to go on a more direct path? move pixel pusher to back and target middle stack?

    public boolean pushPurplePixelWingV3(int path) {
        teamUtil.log("Pushing Pixel");

        if ((teamUtil.alliance==RED && path == 1) || (teamUtil.alliance==BLUE && path == 3)) { // Near the Stacks
            drive.moveCm(drive.MAX_VELOCITY,71, fieldSide(), 180,0); // Was 71
            drive.moveCm(drive.MAX_VELOCITY,25,  teamUtil.alliance==RED ? 140 : 220, 180,0); // was fieldSide() + 50
            if (!drive.strafeToEncoder(driverSide(), 180, 400, (teamUtil.alliance==RED ? 1 : -1)*9300, 2000)) {
                drive.stopMotors();
                return false;
            }
            drive.stopMotors();
            teamUtil.pause(250);
            drive.moveCm(drive.MAX_VELOCITY,40, 180, 180,0);
        } else if ((teamUtil.alliance==RED && path == 3) || (teamUtil.alliance==BLUE && path == 1)) { // Under the Rigging
            drive.moveCm(drive.MAX_VELOCITY, 73, fieldSide(), 180,0);
            drive.moveCm(drive.MAX_VELOCITY, 32, teamUtil.alliance==RED ? 30 : 330, 180,0); // was fieldSide() + 300
            drive.moveCm(drive.MAX_VELOCITY, 11, driverSide(), 180,0);
            drive.moveCm(drive.MAX_VELOCITY, 10, 180, 180,0);
            teamUtil.log("Strafe: "+ drive.strafeEncoder.getCurrentPosition());
            if (!drive.strafeToEncoder(fieldSide(), 180, 400, (teamUtil.alliance==RED ? 1 : -1)*9200, 2000)) { // tweaked encoder target a bit
                drive.stopMotors();
                return false;
            }
            drive.stopMotors();
            teamUtil.pause(250);
            teamUtil.log("Strafe: "+ drive.strafeEncoder.getCurrentPosition());
            drive.moveCm(drive.MAX_VELOCITY, 84, 180, 180,0);
            teamUtil.log("Strafe: "+ drive.strafeEncoder.getCurrentPosition());
        } else { // Path 2, middle for either Alliance
            drive.moveCm(drive.MAX_VELOCITY,86, fieldSide(), 180,0);
            if (!drive.strafeToEncoder(driverSide(), 180, 400, (teamUtil.alliance==RED ? 1 : -1)*9400, 2000)) { // tweaked encoder target a bit
                drive.stopMotors();
                return false;
            }
            drive.stopMotors();
            teamUtil.pause(250);
            drive.moveCm(drive.MAX_VELOCITY,64, 180, 180,0);
        }

        /*
        switch (path) {
            case 1:
                drive.moveCm(drive.MAX_VELOCITY,71, fieldSide(), 180,0); // Was 71
                drive.moveCm(drive.MAX_VELOCITY,25, fieldSide() + 50, 180,0);
                if (!drive.strafeToEncoder(driverSide(), 180, 400, (teamUtil.alliance==RED ? 1 : -1)*9300, 2000)) {
                    drive.stopMotors();
                    return false;
                }
                drive.stopMotors();
                teamUtil.pause(250);
                drive.moveCm(drive.MAX_VELOCITY,40, 180, 180,0);
                break;
            case 2:
                drive.moveCm(drive.MAX_VELOCITY,86, fieldSide(), 180,0);
                if (!drive.strafeToEncoder(driverSide(), 180, 400, (teamUtil.alliance==RED ? 1 : -1)*9400, 2000)) { // tweaked encoder target a bit
                    drive.stopMotors();
                    return false;
                }
                drive.stopMotors();
                teamUtil.pause(250);
                drive.moveCm(drive.MAX_VELOCITY,64, 180, 180,0);
                break;
            case 3:
                drive.moveCm(drive.MAX_VELOCITY, 73, fieldSide(), 180,0);
                drive.moveCm(drive.MAX_VELOCITY, 32, fieldSide() + 300, 180,0);
                drive.moveCm(drive.MAX_VELOCITY, 11, driverSide(), 180,0);
                drive.moveCm(drive.MAX_VELOCITY, 10, 180, 180,0);
                teamUtil.log("Strafe: "+ drive.strafeEncoder.getCurrentPosition());
                if (!drive.strafeToEncoder(fieldSide(), 180, 400, (teamUtil.alliance==RED ? 1 : -1)*9200, 2000)) { // tweaked encoder target a bit
                    drive.stopMotors();
                    return false;
                }
                drive.stopMotors();
                teamUtil.pause(250);
                teamUtil.log("Strafe: "+ drive.strafeEncoder.getCurrentPosition());
                drive.moveCm(drive.MAX_VELOCITY, 84, 180, 180,0);
                teamUtil.log("Strafe: "+ drive.strafeEncoder.getCurrentPosition());

                break;
        }
         */
        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void cycleV3(double xOffset, boolean operateArms) {
        long startTime = System.currentTimeMillis();
        teamUtil.log("Start Cycle");
        drive.runFrontLineFinderProcessor();

        // Strafe and drive across the field to where we can almost see the white tape
        // TODO: Path 3 runs over the purple pixel on RED, Path 1 runs it over on BLUE!!  NEED A FIX!
        double distanceOffset = teamUtil.alliance==RED ? xOffset : -xOffset; // flip the sign on the Xoffset so the following math works on both sides
        drive.moveCm(drive.MAX_VELOCITY, 100 + (distanceOffset > 0 ? 1 : -1) * (Math.sqrt(distanceOffset * distanceOffset * 2)), teamUtil.alliance==RED ? 135:225 , 180, 800); // Heading was fixed at 135
        drive.moveCm(drive.MAX_VELOCITY, 183 - distanceOffset, 180, 180, 350);
        if (operateArms) {
            intake.startIntake();
        }
        drive.driveToStack(180, 180, 350, 3000);
        // TODO: driveToStack needs failsafe code INSIDE it (could also be much faster)
        // TODO: make failsafe for if above returns false

        if (operateArms) {
            intake.grabTwoPixels();
        } else {
            teamUtil.pause(1000);//grabTwoPixels

        }
        drive.runRearAprilTagProcessor();
        drive.moveCm(drive.MAX_VELOCITY, 210, 0, 180, 1000);
        if (operateArms) {
            output.goToScoreNoWait(3); // TODO Adjust height for different paths?
        }
        drive.moveCm(drive.MAX_VELOCITY, 48, teamUtil.alliance==RED ? 300 : 60, 180, 1000); // Heading was fixed at 300
        drive.driveToAprilTagOffset(1000, 0, 180, teamUtil.alliance==RED ? -drive.TAG_CENTER_TO_CENTER : drive.TAG_CENTER_TO_CENTER, 30, 4000);
        //TODO: make failsafe for if above returns false

        drive.moveCm(drive.MAX_VELOCITY,17, 0, 180, 0);
        // Old version using tape
        //drive.moveCm(drive.MAX_VELOCITY, 9, 0, 180, 0);
        //drive.driveToTapeSetPower(.1f, 3000);

        if (operateArms) {
            output.dropAndGoToLoadNoWait();
        } else {
            teamUtil.pause(100);
        }
        long cycleTime = System.currentTimeMillis() - startTime;
        log("cycleTime: " + cycleTime);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void autoV3(int path, boolean operateArms) {
        teamUtil.log("Running Auto Path: " + path + " Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE") + " Side: " + teamUtil.SIDE);

        drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setHeading(180); // Zero is towards the scoring side of field

        if (teamUtil.SIDE == teamUtil.Side.SCORE) {
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            if (operateArms) {
                output.goToScoreNoWait(2); // TODO: Adjust this level for higher consistency?
            }
            if (!pushPurplePixelScoreV3(path)) { //pushes pixel and gets to good location for April tag localization
                return;   //Auto Bailout
            }
        } else { // WING Side
            if (!pushPurplePixelWingV3(path)) { // pushes pixel and drives to stack
                return;   //Auto Bailout
            }
            if (operateArms) {
                intake.grabOnePixel();
            } else {
                teamUtil.pause(1000);// emulate the grab
            }
            intake.stopIntake();


            drive.runRearAprilTagProcessor();

            // Drive around the purple pixel (could be optimized for different paths)
            drive.moveCm(400, 3, 0, 180, 400);
            drive.moveCm(drive.MAX_VELOCITY,50, fieldSide(), 180, 800);

            // Head to other side of field
            drive.moveCm(drive.MAX_VELOCITY,220, 0, 180,800);
            if (operateArms) {
                output.goToScoreNoWait(2);
            }
            // Get to where we can see the AprilTags well
            drive.moveCm(drive.MAX_VELOCITY,65, teamUtil.alliance==RED? 285 : 75, 180, 0); // TODO: SPEED UP: Maybe don't end at at stop
        }
        teamUtil.log("April Tag FPS: " + drive.visionPortal.getFps());
        // Line up for the Yellow Pixel Drop on the correct location
        double xOffset = path == 2 ? 0 : (path == 1 ? -drive.TAG_CENTER_TO_CENTER : drive.TAG_CENTER_TO_CENTER); // TODO Adjust this for better consistency?
        drive.driveToAprilTagOffset(1000, 0, 180, xOffset, 30, 4000);
        drive.moveCm(drive.MAX_VELOCITY,17, 0, 180, 0);

        if (operateArms) {
            output.dropAndGoToLoadNoWait();
        } else {
            teamUtil.pause(100);
        }
        //if (true) return;

        cycleV3(xOffset, operateArms); // One for now...
        teamUtil.pause(1000); // Allow output to get back to loading position
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// OLDER CODE

    public float getPathOffset(int path) {
        float robotBackdropXOffset = drive.getRobotBackdropXOffset();
        if (robotBackdropXOffset > 900) {
            log("No April Tag Detected");
            return 0;
        } else {
            if (path == 2) {
                return robotBackdropXOffset;
            } else if (path == 1) {
                return robotBackdropXOffset + drive.TAG_CENTER_TO_CENTER;
            } else {
                return robotBackdropXOffset - drive.TAG_CENTER_TO_CENTER;
            }
        }
    }

    public boolean pushPurplePixelV3(int path) {
        teamUtil.log("Pushing Pixel");

        switch (path) { // Push the pixel and move back a bit
            case 1:
                drive.moveCm(teamUtil.SIDE == teamUtil.Side.WING ? 69 : 67, fieldSide());
                drive.moveCm(30, fieldSide() + 50);
                if (teamUtil.SIDE == teamUtil.Side.WING) {
                    if (!drive.strafeToEncoder(driverSide(), 180, 400, 9300, 2000)) {
                        drive.stopMotors();
                        return false;
                    }
                } else {
                    //Figure out later
                }

                break;
            case 2:
                drive.moveCm(86, fieldSide());
                if (!drive.strafeToEncoder(driverSide(), 180, 400, 9300, 2000)) {
                    drive.stopMotors();
                    return false;
                }
                break;
            case 3:
                drive.moveCm(teamUtil.SIDE == teamUtil.Side.WING ? 73 : 80, fieldSide());
                drive.moveCm(teamUtil.SIDE == teamUtil.Side.WING ? 32 : 30, fieldSide() + 300);
                if (teamUtil.SIDE == teamUtil.Side.SCORE) {
                    if (!drive.strafeToEncoder(driverSide(), 180, 400, 9300, 2000)) {
                        drive.stopMotors();
                        return false;
                    }
                } else {
                    drive.moveCm(11, driverSide());
                    drive.moveCm(10, 180);
                    if (!drive.strafeToEncoder(fieldSide(), 180, 400, 9300, 2000)) {
                        drive.stopMotors();
                        return false;
                    }
                }
                break;
        }

        return true;
    }

    /*
        // Previous V3 Code for SCORE side after pushing purple pixel
            drive.moveCm(drive.MAX_VELOCITY, 90, 0, 180, 350);
            drive.driveToTapeSetPower(.1f, 3000);
            drive.stopMotors();
            if (operateArms) {
                output.dropAndGoToLoadNoWait();
            } else {
                teamUtil.pause(1000);
            }
            drive.runFrontLineFinderProcessor();
            drive.moveCm(drive.MAX_VELOCITY, 100, 135, 180, 800);

            drive.moveCm(drive.MAX_VELOCITY, 185, 180, 180, 350);
            if (operateArms) {
                intake.startIntake();
            }
            drive.driveToStack(180, 180, 350, 2000);

            if (operateArms) {
                intake.grabTwoPixels();
            } else {
                teamUtil.pause(1000);//grabTwoPixels

            }

            drive.runRearAprilTagProcessor();
            drive.moveCm(drive.MAX_VELOCITY, 210, 0, 180, 1000);
            if (operateArms) {
                output.goToScoreNoWait(2);
            }
            drive.moveCm(drive.MAX_VELOCITY, 48, 300, 180, 1000);

            drive.driveToAprilTagOffset(1000, 0, 180, (-drive.TAG_CENTER_TO_CENTER), 30, 4000);

            drive.moveCm(drive.MAX_VELOCITY, 9, 0, 180, 0);
            drive.driveToTapeSetPower(.1f, 3000);
            if (operateArms) {
                output.dropAndGoToLoadNoWait();
            } else {
                teamUtil.pause(1000);
            }
            drive.runFrontLineFinderProcessor();
            drive.moveCm(drive.MAX_VELOCITY, 67, 135, 180, 800);
            drive.moveCm(drive.MAX_VELOCITY, 198, 180, 180, 350);

            if (operateArms) {
                intake.startIntake();
            }
            drive.driveToStack(180, 180, 350, 3000);

            if (operateArms) {
                intake.grabTwoPixels();
            } else {
                teamUtil.pause(1000);//grabTwoPixels

            }
            drive.runRearAprilTagProcessor();
            drive.moveCm(drive.MAX_VELOCITY, 210, 0, 180, 1000);
            if (operateArms) {
                output.goToScoreNoWait(3);
            }
            drive.moveCm(drive.MAX_VELOCITY, 48, 300, 180, 1000);
            drive.driveToAprilTagOffset(1000, 0, 180, (-drive.TAG_CENTER_TO_CENTER), 30, 4000);

            drive.moveCm(drive.MAX_VELOCITY, 9, 0, 180, 0);
            drive.driveToTapeSetPower(.1f, 3000);
            if (operateArms) {
                output.dropAndGoToLoadNoWait();
            } else {
                teamUtil.pause(1000);
            }
*/

    /*
        // Previous V3 Code for WING side after pushing purple pixel

            drive.stopMotors();
            teamUtil.pause(250);

            if (operateArms) {
                intake.startIntake();
            }
            switch (path) { // Movement to the wall
                case 1:
                    drive.moveCm(40.5,180);
                    break;
                case 2:
                    drive.moveCm(64,180);
                    break;
                case 3:
                    drive.moveCm(80,180);
                    break;
            }

            if (operateArms) {
                intake.grabOnePixel();
            } else {
                teamUtil.pause(1000);//grabTwoPixels
            }

            teamUtil.pause(1000);
            intake.stopIntake();
            if(true)return;

            drive.runRearAprilTagProcessor();
            drive.moveCm(400, 3, 0, 400);
            drive.moveCm( 50, 90, 800);
            drive.moveCm(220, 0, 800);
            if (operateArms) {
                output.goToScoreNoWait(2);
            }
            drive.moveCm(65, 285);
            double xOffset = 0;
            if(path == 1){
                xOffset= (-drive.TAG_CENTER_TO_CENTER);
            }else if(path == 3){
                xOffset = drive.TAG_CENTER_TO_CENTER;
            }
            drive.driveToAprilTagOffset(1000, 0, 180, xOffset, 30, 4000);
            drive.moveCm(17, 0);
            if (operateArms) {
                output.dropAndGoToLoadNoWait();
            } else {
                teamUtil.pause(1000);
            }

            cycleV3(0,operateArms);



            /*drive.runFrontLineFinderProcessor();


            drive.moveCm(drive.MAX_VELOCITY, 86, 135, 180, 800);

            drive.moveCm(drive.MAX_VELOCITY, 194, 180, 180, 350);
            if (operateArms) {
                intake.startIntake();
            }
            drive.driveToStack(180, 180, 350, 2000);

            if (operateArms) {
                intake.grabTwoPixels();
            } else {
                teamUtil.pause(1000);//grabTwoPixels

            }


            drive.runRearAprilTagProcessor();
            drive.moveCm(drive.MAX_VELOCITY, 210, 0, 180, 1000);
            if (operateArms) {
                output.goToScoreNoWait(2);
            }
            drive.moveCm(drive.MAX_VELOCITY, 48, 300, 180, 1000);

            drive.driveToAprilTagOffset(1000, 0, 180, (-drive.TAG_CENTER_TO_CENTER), 30, 4000);

            drive.moveCm(drive.MAX_VELOCITY, 9, 0, 180, 0);
            drive.driveToTapeSetPower(.1f, 3000);
            if (operateArms) {
                output.dropAndGoToLoadNoWait();
            } else {
                teamUtil.pause(1000);
            }

             */

    /*
// Experiment using ultrasonic sensor and strafe encoder to line up on stack
        log("encoder position" + drive.strafeEncoder.getCurrentPosition());
        double strafeDistanceFromStackTics = drive.strafeEncoder.getCurrentPosition()-straferDistanceFarStack;
        double strafeDistanceFromStackCms = strafeDistanceFromStackTics/130;
        log("strafe distance from stack tics" + strafeDistanceFromStackTics);
        log("strafe distance from stack cms" + strafeDistanceFromStackCms);

        long ultrasonicSensorReadingTime = System.currentTimeMillis()+1000;


        drive.moveCm(1000,Math.abs(strafeDistanceFromStackCms),strafeDistanceFromStackCms>0? 270:90,180,0);

        while(teamUtil.keepGoing(ultrasonicSensorReadingTime)){
        }


        log("ultrasonic sensor distance from wall" +drive.getUltrasonicDistance());
        teamUtil.pause(750);
        log("ultrasonic sensor distance from wall" +drive.getUltrasonicDistance());
        double ultrasonicSensorDistanceFromWall = drive.getUltrasonicDistance();
        drive.moveCm(800,ultrasonicSensorDistanceFromWall-10,180,180,0);

        drive.stopMotors();
*/

    public void pushPurplePixelAndGoToAprilTagViewing(int path) {
        teamUtil.log("Pushing Pixel");
        switch (path) { // Push the pixel and move back a bit
            case 1:
                drive.moveCm(67, fieldSide());
                drive.moveCm(30, fieldSide() + 50);
                drive.moveCm(10, driverSide());
                break;
            case 2:
                drive.moveCm(86, fieldSide());
                drive.moveCm(8.5, driverSide());
                break;
            case 3:
                drive.moveCm(75, fieldSide());
                drive.moveCm(30, fieldSide() + 300);
                drive.moveCm(13, driverSide());
                break;
        }
        teamUtil.log("Driving to April Tag Viewing Location");
        if (teamUtil.SIDE == teamUtil.Side.SCORE) { // Run to April Tag viewing location from SCORE Side
            switch (path) {
                case 1:
                    drive.moveCm(drive.MAX_VELOCITY, teamUtil.alliance == RED ? 83 : 23, 0, 180, 0);
                    break;
                case 2:
                    drive.moveCm(drive.MAX_VELOCITY, 45, 0, 180, 0);
                    break;
                case 3:
                    drive.moveCm(drive.MAX_VELOCITY, teamUtil.alliance == RED ? 23 : 83, 0, 180, 0);
                    break;
            }
        } else { // Run to April Tag viewing location from WING Side
            if ((path == 1 && teamUtil.alliance == RED) || (path == 3 && teamUtil.alliance == BLUE)) {
                drive.moveCm(24, scoreSide());
                drive.moveCm(69, fieldSide());
                drive.moveCm(159, scoreSide(), 800);
                drive.moveCm(79, driverSide()); // strafe
            } else if ((path == 3 && teamUtil.alliance == RED) || (path == 1 && teamUtil.alliance == BLUE)) {
                drive.moveCm(52, audienceSide());
                drive.moveCm(69, fieldSide());
                drive.moveCm(180, scoreSide(), 800);
                drive.moveCm(75, driverSide()); // strafe
            } else { // Path 2
                drive.moveCm(36, audienceSide());
                drive.moveCm(64, fieldSide());
                drive.moveCm(197, scoreSide(), 800);
                drive.moveCm(72, driverSide()); // strafe
            }
        }
    }

    public void autoV2(int path, boolean operateArms) {
        teamUtil.log("Running Auto Path: " + path + " Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE") + " Side: " + teamUtil.SIDE);
        drive.setHeading(180); // Zero is towards the scoring side of field
        drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running

        //Push Purple pixel into place and get in position to see April Tags
        pushPurplePixelAndGoToAprilTagViewing(path);
        if (operateArms) output.goToScoreNoWait(1);
        teamUtil.pause(1000); // Allow time for April Tag Viewer to get a solid reading and output to get in position
        float aprilTagOffset = getPathOffset(path);

        //Go to Scoring position
        drive.moveCm(1500, 35, 0, 180, 350);
        drive.driveToTape(0, 180, 350, 2000);
        drive.stopMotors();   //drive.setMotorsActiveBrake();
        teamUtil.pause(500);
        if (teamUtil.alliance == RED) { // use April tag localization offset
            drive.moveCm(Math.abs(aprilTagOffset), aprilTagOffset < 0 ? driverSide() : fieldSide());
        } else {
            drive.moveCm(Math.abs(aprilTagOffset), aprilTagOffset < 0 ? fieldSide() : driverSide());
        }

        // Drop the pixel
        if (operateArms) output.dropPixels();
        teamUtil.pause(1000);

        // get well out of the way
        drive.moveCm(5, audienceSide());
        if (operateArms) output.goToLoadNoWait();
        if (Math.abs(aprilTagOffset) < .01 || path == 2) {
            drive.moveCm(65, fieldSide());
        } else if (path == 1) {
            drive.moveCm(65 + (teamUtil.alliance == teamUtil.Alliance.RED ? -drive.TAG_CENTER_TO_CENTER : drive.TAG_CENTER_TO_CENTER), fieldSide());
        } else { // path 3
            drive.moveCm(65 + (teamUtil.alliance == teamUtil.Alliance.RED ? drive.TAG_CENTER_TO_CENTER : -drive.TAG_CENTER_TO_CENTER), fieldSide());
        }
        drive.moveCm(20, 0);
        log("Auto-Finished");
    }

    public void auto(int path, teamUtil.Side side) { // TODO: Lose the "left" parameter and use TeamUtil.SIDE instead
        drive.setHeading(180); // Zero is towards the scoring side of field
        if (path == 1 && side == teamUtil.Side.WING) {
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(67, fieldSide());
            drive.moveCm(33.5, fieldSide() + 50);
            drive.moveCm(10, driverSide());
            drive.moveCm(25, scoreSide());
            drive.moveCm(69, fieldSide());
            drive.moveCm(127, scoreSide(), drive.MAX_VELOCITY);
            output.goToScoreNoWait(1);
            drive.moveCm(90, driverSide()); // strafe
            teamUtil.pause(500);
            float aprilTagOffset = getPathOffset(path);


            log("aprilTagOffset: " + aprilTagOffset);
            log("aprilTagOffset" + aprilTagOffset);

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0, 180, 350, 4000);


            drive.setMotorsActiveBrake();
            teamUtil.pause(500);
            drive.moveCm(Math.abs(aprilTagOffset), aprilTagOffset < 0 ? fieldSide() : driverSide());


            /*
            if(Math.abs(aprilTagOffset)>3.25){
                if(aprilTagOffset>0&&aprilTagOffset<900){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                }
                else if(aprilTagOffset<0){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

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

             */
            output.dropPixels();
            teamUtil.pause(1000);
            drive.moveCm(5, 180);

            output.goToLoadNoWait();
            drive.moveCm(60, fieldSide());


        } else if (path == 2 && side == teamUtil.Side.WING) {
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(86, fieldSide());
            drive.moveCm(8.5, driverSide());
            drive.moveCm(36, audienceSide());
            drive.moveCm(64, fieldSide());
            drive.moveCm(170, scoreSide(), drive.MAX_VELOCITY);
            output.goToScoreNoWait(1);
            drive.moveCm(100, driverSide()); // strafe
            teamUtil.pause(500);
            float aprilTagOffset = getPathOffset(path);
            /*
            double aprilTagOffset;
            if(teamUtil.alliance == teamUtil.Alliance.RED) {
                aprilTagOffset = drive.returnAprilTagIDOffset(5, 500);
            }else{
                aprilTagOffset = drive.returnAprilTagIDOffset(2, 500);
            }
            log("aprilTagOffset: "+aprilTagOffset);

            log("aprilTagOffset"+aprilTagOffset);

             */

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0, 180, 350, 4000);
            //while (!drive.tapeSensor1.isOnTape()||!drive.tapeSensor2.isOnTape()){}

            drive.setMotorsActiveBrake();
            teamUtil.pause(500);
            drive.moveCm(Math.abs(aprilTagOffset), aprilTagOffset < 0 ? fieldSide() : driverSide());




/*
            if(Math.abs(aprilTagOffset)>3.25){
                if(aprilTagOffset>0&&aprilTagOffset<900){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                }
                else if(aprilTagOffset<0){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

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

 */
            output.dropPixels();
            teamUtil.pause(1000);
            drive.moveCm(5, audienceSide());
            output.goToLoadNoWait();
            drive.moveCm(60, fieldSide());


        } else if (path == 3 && side == teamUtil.Side.WING) {
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(67, fieldSide());
            drive.moveCm(33.5, fieldSide() - 55);
            drive.moveCm(10, driverSide());


            drive.moveCm(52, audienceSide());

            drive.moveCm(69, fieldSide());

            drive.moveCm(150, scoreSide(), drive.MAX_VELOCITY);
            output.goToScoreNoWait(1);
            drive.moveCm(100, driverSide()); // strafe
            teamUtil.pause(500);
            float aprilTagOffset = getPathOffset(path);
            /*
            if(teamUtil.alliance == teamUtil.Alliance.RED) {
                aprilTagOffset = drive.returnAprilTagIDOffset(6, 500);
            }else{
                aprilTagOffset = drive.returnAprilTagIDOffset(3, 500);
            }
            log("aprilTagOffset: "+aprilTagOffset);
            log("aprilTagOffset"+aprilTagOffset);

             */

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0, 180, 350, 4000);


            drive.setMotorsActiveBrake();


            if (Math.abs(aprilTagOffset) > 3.25) {
                if (aprilTagOffset > 0 && aprilTagOffset < 900) {
                    if (teamUtil.alliance == RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    } else {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                } else if (aprilTagOffset < 0) {
                    if (teamUtil.alliance == RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    } else {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

                    }
                } else {
                    if (teamUtil.alliance == RED) {
                        drive.moveCm(15, driverSide());
                    } else {
                        drive.moveCm(15, fieldSide());

                    }
                }
            }
            output.dropPixels();
            teamUtil.pause(1000);
            drive.moveCm(5, 180);

            output.goToLoadNoWait();
            drive.moveCm(80, fieldSide());


        } else if (!(side == teamUtil.Side.WING) && path == 1) {

            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(65, fieldSide());
            drive.moveCm(33, fieldSide() + 50);
            drive.moveCm(10, driverSide());


            output.goToScoreNoWait(1);
            drive.moveCm(75, 0, 0);//TODO:change to min end when callibrated (also, should be more like 80)
            teamUtil.pause(1000);
            float aprilTagOffset = getPathOffset(path);
            log("aprilTagOffset" + aprilTagOffset);
            drive.moveCm(25, 0, 350);
            /*
            drive.getRobotBackdropXOffset();
            if(teamUtil.alliance == teamUtil.Alliance.RED) {
                aprilTagOffset = drive.returnAprilTagIDOffset(4, 500);
            }else{
                aprilTagOffset = drive.returnAprilTagIDOffset(1, 500);
            }

            log("aprilTagOffset"+aprilTagOffset);

             */

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0, 180, 350, 4000);
            //while (!drive.tapeSensor1.isOnTape()||!drive.tapeSensor2.isOnTape()){}

            drive.setMotorsActiveBrake();
            teamUtil.pause(500);
            drive.moveCm(Math.abs(aprilTagOffset), aprilTagOffset < 0 ? driverSide() : fieldSide());
/*
            if(Math.abs(aprilTagOffset)>3.25){
                if(aprilTagOffset>0&&aprilTagOffset<900){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                }
                else if(aprilTagOffset<0){
                    if(teamUtil.alliance == teamUtil.Alliance.RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    }else{
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

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

 */
            output.dropPixels();
            teamUtil.pause(1000);
            drive.moveCm(5, 180);

            output.goToLoadNoWait();
            drive.moveCm(60, fieldSide());


        } else if (!(side == teamUtil.Side.WING) && path == 2) {
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(86, fieldSide());
            drive.moveCm(8.5, driverSide());
            output.goToScoreNoWait(1);
            drive.moveCm(40, 0, 350);//TODO:change to min end when callibrated (also, should be more like 80)
            double aprilTagOffset;
            if (teamUtil.alliance == RED) {
                aprilTagOffset = drive.returnAprilTagIDOffset(5, 500);
            } else {
                aprilTagOffset = drive.returnAprilTagIDOffset(2, 500);
            }
            log("aprilTagOffset: " + aprilTagOffset);

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0, 180, 350, 3000);
            //while (!drive.tapeSensor1.isOnTape()||!drive.tapeSensor2.isOnTape()){}

            drive.setMotorsActiveBrake();

            //TODO fix 3.25

            if (Math.abs(aprilTagOffset) > 3.25) {
                if (aprilTagOffset > 0 && aprilTagOffset < 900) {
                    if (teamUtil.alliance == RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    } else {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                } else if (aprilTagOffset < 0) {
                    if (teamUtil.alliance == RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    } else {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

                    }
                } else {
                    if (teamUtil.alliance == RED) {
                        drive.moveCm(15, driverSide());
                    } else {
                        drive.moveCm(15, fieldSide());

                    }
                }
            }
            output.dropPixels();

            teamUtil.pause(1000);
            drive.moveCm(5, 180);
            output.goToLoadNoWait();
            drive.moveCm(60, fieldSide());


        } else if (!(side == teamUtil.Side.WING) && path == 3) {
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(80, fieldSide());
            drive.moveCm(33, fieldSide() + 300);
            drive.moveCm(13, driverSide());


            output.goToScoreNoWait(1);
            drive.moveCm(10, 0, 350);//TODO:change to min end when callibrated (also, should be more like 80)
            double aprilTagOffset;
            if (teamUtil.alliance == RED) {
                aprilTagOffset = drive.returnAprilTagIDOffset(6, 500);
            } else {
                aprilTagOffset = drive.returnAprilTagIDOffset(3, 500);
            }
            drive.moveCm(40, 0, 350);
            log("aprilTagOffset: " + aprilTagOffset);

            log("aprilTagOffset" + aprilTagOffset);

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0, 180, 350, 4000);
            //while (!drive.tapeSensor1.isOnTape()||!drive.tapeSensor2.isOnTape()){}

            drive.setMotorsActiveBrake();


            if (Math.abs(aprilTagOffset) > 3.25) {
                if (aprilTagOffset > 0 && aprilTagOffset < 900) {
                    if (teamUtil.alliance == RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());
                    } else {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());

                    }
                } else if (aprilTagOffset < 0) {
                    if (teamUtil.alliance == RED) {
                        drive.moveCm(Math.abs(aprilTagOffset), driverSide());
                    } else {
                        drive.moveCm(Math.abs(aprilTagOffset), fieldSide());

                    }
                } else {
                    if (teamUtil.alliance == RED) {
                        drive.moveCm(15, driverSide());
                    } else {
                        drive.moveCm(15, fieldSide());

                    }
                }
            }
            output.dropPixels();
            teamUtil.pause(1000);
            drive.moveCm(5, 180);

            output.goToLoadNoWait();
            drive.moveCm(96, fieldSide());

        }
        teamUtil.pause(5000); // TODO: Remove this for competition, just helps with debugging auto as it is developed

    }
}

