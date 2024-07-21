package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.assemblies.Launcher;
import org.firstinspires.ftc.teamcode.assemblies.Output;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "CalibrateDriveSystem")
public class CalibrateDriveSystem extends LinearOpMode {

    Drive drive;
    TeamGamepad teamGamePad;
    //sampleStats distanceStats;
    double testVelocity = 1000;
    double testDistance = 100;

    double HEADING = 0;
    long TIME = 3;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.BLUE;
        teamUtil.SIDE=teamUtil.Side.SCORE;
        teamGamePad = new TeamGamepad();
        teamGamePad.initilize(true);
        drive = new Drive();
//        drive.initalize(null);
        drive.initalize();
        while(!teamGamePad.wasAPressed()){
            teamGamePad.loop();
            if(teamGamePad.wasLeftPressed()){ teamUtil.alliance = teamUtil.Alliance.RED;}
            if(teamGamePad.wasRightPressed()){ teamUtil.alliance = teamUtil.Alliance.BLUE;}

            teamUtil.telemetry.addLine("RED or BLUE? (use Game Pad 1 DPad)");
            teamUtil.telemetry.addLine(teamUtil.alliance == teamUtil.Alliance.RED ? "RED Alliance" : "BLUE Alliance");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }
        
        //teamUtil.initPerf();
        //distanceStats = new sampleStats();
        //distanceStats.setTimeWindow(250);
    }


    public void testDriveMotorWiring() {
        drive.setMotorVelocities(drive.MIN_START_VELOCITY,0,0,0);
        teamUtil.pause(1000);
        drive.setMotorVelocities(0,drive.MIN_START_VELOCITY,0,0);
        teamUtil.pause(1000);
        drive.setMotorVelocities(0,0,drive.MIN_START_VELOCITY,0);
        teamUtil.pause(1000);
        drive.setMotorVelocities(0,0,0,drive.MIN_START_VELOCITY);
        teamUtil.pause(1000);
        drive.stopMotors();
    }

    public void moveNoAccelerateNoHeadingControl () {
        drive.driveMotorsHeadings(HEADING, drive.getHeading(), drive.MIN_START_VELOCITY);
        teamUtil.pause(TIME*1000);
        drive.stopMotors();
    }

    public void moveNoAccelerateWithHeadingControl () {
        long doneTime = System.currentTimeMillis() + (int)(TIME*1000);
        double heldHeading = drive.getHeading();
        while (System.currentTimeMillis() < doneTime) {
            drive.driveMotorsHeadings(HEADING, heldHeading, drive.MIN_START_VELOCITY);
        }
        drive.stopMotors();
    }

    public void goForADrive() {
        drive.moveCm(50, 90);
        drive.moveCm(50, 180);
        drive.moveCm(50, 270);
        drive.moveCm(50, 0);

        drive.moveCm(50, 45);
        drive.moveCm(50, 135);
        drive.moveCm(50, 225);
        drive.moveCm(50, 315);
    }

    public void adjustSpinThresholds () {
        if (teamGamePad.wasUpPressed()) {
            drive.SPIN_END_OF_MAX_VELOCITY = drive.SPIN_END_OF_MAX_VELOCITY + 1;
        } else if (teamGamePad.wasDownPressed()) {
            drive.SPIN_END_OF_MAX_VELOCITY = drive.SPIN_END_OF_MAX_VELOCITY - 1;
        } else if (teamGamePad.wasLeftPressed()) {
            drive.CRAWL_DISTANCE_SPINS = drive.CRAWL_DISTANCE_SPINS + 1;
        } else if (teamGamePad.wasRightPressed()) {
            drive.CRAWL_DISTANCE_SPINS = drive.CRAWL_DISTANCE_SPINS - 1;
        } else if (teamGamePad.wasYPressed()) {
            drive.CRAWL_SPEED = drive.CRAWL_SPEED + 10;
        } else if (teamGamePad.wasAPressed()) {
            drive.CRAWL_SPEED = drive.CRAWL_SPEED - 10;
        } else if (teamGamePad.wasXPressed()) {
            drive.CRAWL_DISTANCE_SPINS = drive.CRAWL_DISTANCE_SPINS - 5;
        } else if (teamGamePad.wasBPressed()) {
            drive.CRAWL_DISTANCE_SPINS = drive.CRAWL_DISTANCE_SPINS + 5;
        }
    }

    public void adjustBasicMovementParams() {
        if (teamGamePad.wasUpPressed()) {
            drive.MIN_START_VELOCITY = drive.MIN_START_VELOCITY + 10;
        } else if (teamGamePad.wasDownPressed()) {
            drive.MIN_START_VELOCITY = drive.MIN_START_VELOCITY - 10;
        } else if (teamGamePad.wasLeftPressed()) {
            drive.MIN_END_VELOCITY = drive.MIN_END_VELOCITY - 10;
        } else if (teamGamePad.wasRightPressed()) {
            drive.MIN_END_VELOCITY = drive.MIN_END_VELOCITY + 10;
        } else if (teamGamePad.wasYPressed()) {
            drive.MAX_ACCELERATION = drive.MAX_ACCELERATION + 10;
        } else if (teamGamePad.wasAPressed()) {
            drive.MAX_ACCELERATION = drive.MAX_ACCELERATION - 10;
        } else if (teamGamePad.wasXPressed()) {
            drive.MAX_DECELERATION = drive.MAX_DECELERATION - 0.5f;
        } else if (teamGamePad.wasBPressed()) {
            drive.MAX_DECELERATION = drive.MAX_DECELERATION + 0.5f;
        }
    }

    public void adjustBasicMovementParams2() {
        if (teamGamePad.wasUpPressed()) {
            testVelocity = testVelocity + 50;
        } else if (teamGamePad.wasDownPressed()) {
            testVelocity = testVelocity - 50;
        } else if (teamGamePad.wasLeftPressed()) {
            testDistance = testDistance - 10;
        } else if (teamGamePad.wasRightPressed()) {
            testDistance = testDistance + 10;
        }
    }

    public void testSpins() {
        if (gamepad1.dpad_up) {
            drive.spinToHeading(0);
        }
        if (gamepad1.dpad_down) {
            drive.spinToHeading(180);
        }
        if (gamepad1.dpad_left) {
            drive.spinToHeading(270);
        }
        if (gamepad1.dpad_right) {
            drive.spinToHeading(90);
        }

    }

    public void testMoveInches() {
        if (teamGamePad.gamepad.dpad_up) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 0, 0,0);
        } else if (teamGamePad.gamepad.dpad_down) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 180, 0,0);
        } else if (teamGamePad.gamepad.dpad_left) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 90, 0,0);
        } else if (teamGamePad.gamepad.dpad_right) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 270, 0,0);
        } else if (teamGamePad.gamepad.y) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 45, 0,0);
        } else if (teamGamePad.gamepad.a) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 225, 0,0);
        } else if (teamGamePad.gamepad.x) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 135, 0,0);
        } else if (teamGamePad.gamepad.b) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 315, 0,0);
        }
    }

    public void crazyTest () {
        drive.setHeading(0);
        drive.moveCm(2000   , 200  , 0, 0,2000);
        drive.setMotorsBrake();
        drive.stopMotors();
    }
    /*
    public void adjustDistanceSensorMovementConstants () {
        if (teamGamePad.wasUpPressed()) {
            drive.MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD = drive.MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD + .1;
        } else if (teamGamePad.wasDownPressed()) {
            drive.MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD = drive.MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD - .1;
        } else if (teamGamePad.wasLeftPressed()) {
            drive.MOVE_TO_DISTANCE_DRIFT_DISTANCE = drive.MOVE_TO_DISTANCE_DRIFT_DISTANCE - .1;
        } else if (teamGamePad.wasRightPressed()) {
            drive.MOVE_TO_DISTANCE_DRIFT_DISTANCE = drive.MOVE_TO_DISTANCE_DRIFT_DISTANCE + .1;
        } else if (teamGamePad.wasYPressed()) {
            drive.MOVE_TO_DISTANCE_SLOW_DISTANCE = drive.MOVE_TO_DISTANCE_SLOW_DISTANCE + 1;
        } else if (teamGamePad.wasAPressed()) {
            drive.MOVE_TO_DISTANCE_SLOW_DISTANCE = drive.MOVE_TO_DISTANCE_SLOW_DISTANCE - 1;
        } else if (teamGamePad.wasXPressed()) {
            drive.MOVE_TO_DISTANCE_DECEL_DISTANCE = drive.MOVE_TO_DISTANCE_DECEL_DISTANCE - 1;
        } else if (teamGamePad.wasBPressed()) {
            drive.MOVE_TO_DISTANCE_DECEL_DISTANCE = drive.MOVE_TO_DISTANCE_DECEL_DISTANCE + 1;
        }
    }

    public void testMoveToDistance() {
        if (gamepad2.right_bumper) {
            // Test starting from a stop
            drive.moveToDistance(drive.rightDistance, 270, 5, 7000);
        } else if (gamepad2.left_bumper) {
            // Test starting from a full speed drive
            drive.moveInches(0, 48, 7000, drive.DRIVE_MAX_MOVE_TO_DISTANCE_VELOCITY);
            drive.moveToDistance(drive.frontDistance, 0, 5, 7000);
        }
    }
*/
    /*
    public void adjustMoveToLineSpeed () {
        if (teamGamePad.wasUpPressed()) {
            testVelocity = testVelocity + 100;
        } else if (teamGamePad.waDownPressed()) {
            testVelocity = testVelocity - 100;
        }
    }

    public void testMoveToLine() {
        if (gamepad2.right_bumper) {
            // Test starting from a stop
            drive.moveToLine(drive.frontRightColor, teamColorSensor.TapeColor.WHITE, 0, 7000);
            drive.stopMotors();
        } else if (gamepad2.left_bumper) {
            // Test starting from drive at various velocities
            drive.moveInches(0, 36, 7000, drive.FIND_LINE_SPEED); // go fast then slow down to find tape
            drive.moveToLine(drive.frontRightColor, teamColorSensor.TapeColor.RED, 0, 7000);
            drive.moveInches(0, 5, 7000); // move a certain distance after tape and stop
        }
    }


 */
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        drive.resetHeading();
        long start;
        long time = 0;
        while (opModeIsActive()) {
            teamGamePad.loop();
            if (teamGamePad.gamepad.left_bumper) { //
                if (teamGamePad.wasAPressed()) {
                    drive.findMaxVelocity(200);
                } else if (teamGamePad.wasBPressed()) {
                    drive.findMaxStrafeVelocity(200);
                }
            } else if (teamGamePad.gamepad.right_bumper) { //
                adjustBasicMovementParams();
            } else if (teamGamePad.gamepad.left_trigger > .5) {
                testMoveInches();
            } if (teamGamePad.gamepad.right_trigger > .5) { // HOLD LEFT TRIGGER TO TEST MOVE INCHES
                adjustBasicMovementParams2();
            }else {
                if (teamGamePad.wasAPressed()) {
                    drive.resetHeading();
                    start = System.currentTimeMillis();
                    crazyTest();
                    time = System.currentTimeMillis() - start;
                }
                if (teamGamePad.wasBPressed()) {
                    drive.resetHeading();
                    start = System.currentTimeMillis();

                    time = System.currentTimeMillis() - start;
                }
                if (teamGamePad.wasXPressed()) {
                    drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.strafeEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (teamGamePad.wasYPressed()) {

                }
                if (teamGamePad.wasUpPressed()) {

                }
                if (teamGamePad.wasDownPressed()) {

                }


                if (teamGamePad.gamepad.left_stick_button && teamGamePad.gamepad.right_stick_button) { // GP2 to Reset Heading
                    drive.resetHeading();
                }
            }
            /*
            if (teamGamePad.gamepad.left_bumper) { // HOLD GP1 LEFT BUMPER TO ADJUST SPIN PARAMS
                adjustSpinThresholds();
            } else if (teamGamePad.gamepad.right_bumper) { // HOLD GP1 RIGHT BUMPER TO TEST SPINS
                testSpins();
            }
             */




            drive.driveMotorTelemetry();
            //telemetry.addLine("strafeEncoder:"+ drive.strafeEncoder.getCurrentPosition());
            telemetry.addLine("Op Time:"+ time);


            //telemetry.addLine("DistNoMv:"+ drive.MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD +" Drift:"+drive.MOVE_TO_DISTANCE_DRIFT_DISTANCE +" Slow:"+drive.MOVE_TO_DISTANCE_SLOW_DISTANCE+" Dec:"+drive.MOVE_TO_DISTANCE_DECEL_DISTANCE);
            telemetry.addLine("Velocity:"+ testVelocity + "Distance:"+ testDistance);

            telemetry.addLine("Start:"+ drive.MIN_START_VELOCITY +" End:"+drive.MIN_END_VELOCITY +" Acc:"+drive.MAX_ACCELERATION+" Dec:"+drive.MAX_DECELERATION);
            telemetry.addLine("SpinSLOW:"+ drive.CRAWL_SPEED+" SpinMAX:"+drive.MAX_VELOCITY+" SlowThreshold:"+drive.CRAWL_DISTANCE_SPINS+" Dec:"+drive.SPIN_END_OF_MAX_VELOCITY);
            //telemetry.addLine("Heading:"+ HEADING+" TIME:"+TIME);
            teamUtil.telemetry.update();

        }
    }
}


