package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.assemblies.Launcher;
import org.firstinspires.ftc.teamcode.assemblies.Output;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Test Drive", group = "LinearOpMode")
public class TestDrive extends LinearOpMode {
    Drive drive;
    Blinkin blinkin;
    Intake intake;
    Output output;
    Launcher launcher;
    TeamGamepad gamepad;
    int currentCam = 0;
    private int     myExposure  ;
    private int     minExposure ;
    private int     maxExposure ;
    private int     myGain      ;
    private int     minGain ;
    private int     maxGain ;
    public void toggleCamera() {
        currentCam++;
        if (currentCam > 3) {
            currentCam = 1;
        }
        switch(currentCam) {
            case 1:
                teamUtil.log("Toggling LineFinder On");
                drive.runFrontLineFinderProcessor();
                break;
            case 2:
                teamUtil.log("Toggling AprilTag Finder On");
                drive.runRearAprilTagProcessor();
                break;
            case 3:
                teamUtil.log("Toggling TeamProp Finder On");
                drive.runSideTeamPropFinderProcessor();
                break;
            default:
        }
    }

    public void cycle(double xOffset){
        drive.runFrontLineFinderProcessor();
        drive.moveCm(86+(xOffset>0?1:-1)*(Math.sqrt(xOffset*xOffset*2)), 135, 800);
        drive.moveCm(drive.MAX_VELOCITY, 194-xOffset, 180, 180, 350);

    } // TODO: DELETE MEEEEEEEEEEEEEEEEEEEEEEEEEE
    private void getCameraSettings() {
        // Ensure Vision Portal has been setup.
        if (drive.visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (drive.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (drive.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = drive.visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = drive.visionPortal.getCameraControl(GainControl.class);
            if (gainControl == null) { // FTC software has a bug where this isn't supported on switchableCameras!
                minGain = 0;
                maxGain = 0;
            } else {
                minGain = gainControl.getMinGain();
                maxGain = gainControl.getMaxGain();
            }

        }
    }


@Override
    public void runOpMode(){
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.BLUE;
        teamUtil.SIDE=teamUtil.Side.SCORE;
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        intake = new Intake();
        intake.initalize();
        output = new Output(intake);
        output.initialize();
        output.calibrate();
        launcher = new Launcher();
        launcher.initialize();
        drive = new Drive();
        drive.initalize();
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            if(gamepad.wasLeftPressed()){ teamUtil.alliance = teamUtil.Alliance.RED;}
            if(gamepad.wasRightPressed()){ teamUtil.alliance = teamUtil.Alliance.BLUE;}

            teamUtil.telemetry.addLine("RED or BLUE? (use Game Pad 1 DPad)");
            teamUtil.telemetry.addLine(teamUtil.alliance == teamUtil.Alliance.RED ? "RED Alliance" : "BLUE Alliance");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            if(gamepad.wasLeftPressed()){ teamUtil.SIDE = teamUtil.Side.SCORE;}
            if(gamepad.wasRightPressed()){ teamUtil.SIDE = teamUtil.Side.WING;}

            teamUtil.telemetry.addLine("Score or Wing? (use Game Pad 1 DPad)");
            teamUtil.telemetry.addLine(teamUtil.SIDE == teamUtil.Side.SCORE ? "Score Side" : "Wing Side");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }


        drive.initCV(true);
        getCameraSettings();
        drive.runRearAprilTagProcessor();
        //drive.runSideTeamPropFinderProcessor();
        double velocity = drive.MAX_VELOCITY;
        telemetry.addLine("Ready");
        telemetry.update();
        while (!opModeIsActive()) {
            gamepad.loop();
            if(gamepad.wasRightBumperPressed()){
                toggleCamera();
            }
            if(gamepad.wasUpPressed()) {
                drive.findLineProcesser.viewingPipeline = !drive.findLineProcesser.viewingPipeline;
                //drive.findPixelProcesser.viewingPipeline = !drive.findPixelProcesser.viewingPipeline;
            }
            if(gamepad.wasDownPressed()) {
                drive.findLineProcesser.nextView(); ;
                //drive.findPixelProcesser.nextView(); ;
            }
            if(gamepad.wasLeftPressed()) {
                drive.findLineProcesser.whiteThreshold = drive.findLineProcesser.whiteThreshold-10;
            }
            if(gamepad.wasRightPressed()) {
                drive.findLineProcesser.whiteThreshold = drive.findLineProcesser.whiteThreshold+10;
            }
            if(gamepad.wasYPressed()) {
                myExposure = myExposure+1;
                drive.setCamExposure(myExposure,0);
            }
            if(gamepad.wasAPressed()) {
                myExposure = myExposure-1;
                drive.setCamExposure(myExposure,0);
            }
            telemetry.addData("Exposure","%d  (%d - %d)", myExposure, minExposure, maxExposure);
            telemetry.addData("Gain","%d  (%d - %d)", myGain, minGain, maxGain);

            intake.outputTelemetry();
            drive.sensorTelemetry();
            drive.visionTelemetry();
            telemetry.update();
        }
        waitForStart();
        drive.setHeading(180);
        while (opModeIsActive()) {
            gamepad.loop();

            ////////// Drive
            if (gamepad.gamepad.right_stick_button && gamepad.gamepad.left_stick_button) {
                drive.setHeading(180); // Zero is towards the scoring side of field
            }
            if (teamUtil.alliance == teamUtil.Alliance.RED) {
                drive.universalDriveJoystick(
                        gamepad.gamepad.left_stick_y,
                        -gamepad.gamepad.left_stick_x,
                        gamepad.gamepad.right_stick_x,
                        gamepad.gamepad.right_trigger > .5,
                        drive.getHeading());
            } else {
                drive.universalDriveJoystick(
                        -gamepad.gamepad.left_stick_y,
                        gamepad.gamepad.left_stick_x,
                        gamepad.gamepad.right_stick_x,
                        gamepad.gamepad.right_trigger > .5,
                        drive.getHeading());
            }

            if(gamepad.wasRightBumperPressed()){
                toggleCamera();
                //drive.setHeading(180);
                //drive.moveCm(1000, 20, 180, 180, 1000);
                //drive.centerOnAprilTag(8, 40, 0, 180);
            }
            if(gamepad.wasLeftBumperPressed()){
                launcher.toggleRelease();
            }
            if (gamepad.wasDownPressed()) {
                drive.moveStraightCmWithStrafeEncoder(drive.MAX_VELOCITY, 180, 0, 180, 180, 0);
            }
            if (gamepad.wasOptionsPressed()) {
                drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad.wasUpPressed()) {
                drive.driveToAprilTagOffset(1000, 0, 180, 0, 30, 4000);
                //drive.moveStraightCmWithStrafeEncoder(drive.MAX_VELOCITY, 180, 0, 0, 180, 0);
                //cycle((drive.TAG_CENTER_TO_CENTER));
            }

            if (gamepad.wasLeftPressed()) {
                //drive.findPixelProcesser.reset();
                output.moveStraferLeft();
            }
            if (gamepad.wasRightPressed()) {
                output.moveStraferRight();
            }
            /*
            if(gamepad.wasYPressed()){
                output.goToScore(2);
            }

             */
            if(gamepad.wasAPressed()){
                output.goToLoad();
            }
            if(gamepad.wasBPressed()){
                intake.toggleIntake();
            }
            if(gamepad.wasXPressed()){
                intake.reverseIntake();
            }

            /*
                // DriveToStack Test Code
                drive.moveCm(2000, 30,180, 180, 350);
                drive.findLineProcesser.reset();
                drive.driveToStack(180,180,350,2000);

                //Drive To April Tag Offset test code
                drive.moveCm(drive.MAX_VELOCITY,60,300, 180, 1000);
                drive.driveToAprilTagOffset(1000,0,180,(-drive.TAG_CENTER_TO_CENTER)/2,30,4000); // TODO: Fix timeout
                drive.moveCm(drive.MAX_VELOCITY,12,0, 180, drive.MIN_END_VELOCITY);
                drive.driveToTape(0,180,350,1500);
                drive.stopMotors();

                // Drive to proximity test code
                drive.driveToProximityNoWait(0,180,400,3000);
                drive.setMotorsActiveBrake();
                teamUtil.pause(500);
                drive.setMotorsWithEncoder();
             */

            /*
            telemetry.addLine("Acceleration (a): "+drive.MAX_ACCELERATION);
            telemetry.addLine("Deceleration (b): "+drive.MAX_DECELERATION);
            telemetry.addLine("Minimum Start (x):"+drive.MIN_START_VELOCITY);
            telemetry.addLine("Minimum End (y):"+drive.MIN_END_VELOCITY);
            telemetry.addLine("Velocity (options):"+velocity);
            telemetry.addLine("Heading (right trigger): "+drive.getHeading());
            telemetry.addLine("Up and down move up and down, hold directional button when you press the other button");
            telemetry.addLine("Ultrasonic Distance: "+drive.getUltrasonicDistance());
            */


            /*
            if(gamepad.gamepad.right_trigger > .5){
                drive.setHeading(0);
                //drive.moveCm(1000, 20, 180, 180, 1000);
                //drive.localizeWithAprilTag(9, 0, 30, 0);

                drive.stopAtUltDistance(30,0,0);
                drive.correctAndHug(9,0);

                // Start with robot back on line in front of goal, pointed straight at far wall, centered on the pass through
                //drive.moveCm(2500, 220, 0, 0, 500);
                //drive.stopAtUltDistance(30,0,0);
            }
             */
            /*
            if(gamepad.wasLeftBumperPressed()){
                drive.findMaxVelocity();
            }
            */

            /*
            if(gamepad.wasOptionsPressed()){
                if(gamepad1.dpad_up){
                    velocity += 100;
                }
                if(gamepad1.dpad_down){
                    velocity -= 100;
                }
            }
            if(gamepad.wasAPressed()){
                if(gamepad1.dpad_up){
                    drive.MAX_ACCELERATION += 1;
                }
                if(gamepad1.dpad_down){
                    drive.MAX_ACCELERATION -= 1;
                }
            }
            if(gamepad.wasBPressed()){
                if(gamepad1.dpad_up){
                    drive.MAX_DECELERATION += 0.5;
                }
                if(gamepad1.dpad_down){
                    drive.MAX_DECELERATION -= 0.5;
                }
            }
            if(gamepad.wasXPressed()){
                if(gamepad1.dpad_up){
                    drive.MIN_START_VELOCITY += 10;
                }
                if(gamepad1.dpad_down){
                    drive.MIN_START_VELOCITY -= 10;
                }
            }
            if(gamepad.wasYPressed()){
                if(gamepad1.dpad_up){
                    drive.MIN_END_VELOCITY += 10;
                }
                if(gamepad1.dpad_down){
                    drive.MIN_END_VELOCITY -= 10;
                }
            }
            */






            drive.driveMotorTelemetry();
            intake.outputTelemetry();
            drive.sensorTelemetry();
            drive.visionTelemetry();
            telemetry.addLine("Path: "+drive.findTeamPropProcesser.getPropPosition());
            telemetry.update();
        }
    }
}
