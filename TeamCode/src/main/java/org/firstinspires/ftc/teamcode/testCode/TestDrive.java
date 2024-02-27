package org.firstinspires.ftc.teamcode.testCode;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.assemblies.Launcher;
import org.firstinspires.ftc.teamcode.assemblies.Output;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "Test Drive", group = "LinearOpMode")
public class TestDrive extends LinearOpMode {
    Robot robot;
    Drive drive;
    Blinkin blinkin;
    Intake intake;
    Output output;
    Launcher launcher;
    TeamGamepad gamepad;
    int currentCam = 0;

    public void toggleCamera() {
        currentCam++;
        if (currentCam > 4) {
            currentCam = 1;
        }
        switch(currentCam) {
            case 1:
                teamUtil.log("Toggling LineFinder On");
                drive.switchCV(Drive.cvCam.FRONT_LINE);
                break;
            case 2:
                teamUtil.log("Toggling AprilTag Finder On Without Yellow");
                drive.switchCV(Drive.cvCam.REAR_APRILTAG);

                break;
            case 3:
                teamUtil.log("Toggling AprilTag Finder On With Yellow");
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.NORMAL_WHITE);
                drive.switchCV(Drive.cvCam.REAR_YELLOW_APRILTAG);

                break;

            case 4:
                teamUtil.log("Toggling TeamProp Finder On");
                drive.switchCV(Drive.cvCam.SIDE_PROP);
                break;
            default:
        }
    }





@Override
    public void runOpMode(){
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.BLUE;
        teamUtil.SIDE=teamUtil.Side.SCORE;
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        robot = new Robot();
        robot.initialize();
        teamUtil.robot = robot;
        robot.calibrate();

        intake = robot.intake;
        output = robot.output;
        launcher = robot.launcher;
        drive = robot.drive;

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

        telemetry.addLine("Initializing CV");
        telemetry.update();
        drive.initCV(true);
        double velocity = drive.MAX_VELOCITY;
        telemetry.addLine("Ready");
        telemetry.update();
        while (!opModeIsActive()) {
            gamepad.loop();
            if(gamepad.wasRightBumperPressed()){
                toggleCamera();
            }
            if(gamepad.wasUpPressed()) {
                //drive.findLineProcesser.viewingPipeline = !drive.findLineProcesser.viewingPipeline;
                drive.findPixelProcesser.viewingPipeline = !drive.findPixelProcesser.viewingPipeline;
            }
            if(gamepad.wasDownPressed()) {
                //drive.findLineProcesser.nextView(); ;
                drive.findPixelProcesser.nextView(); ;
            }

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
            }
            if (gamepad.wasRightTriggerPressed()) {
                drive.stopCV();
            }
            if(gamepad.wasLeftBumperPressed()){
                //launcher.toggleRelease();
                intake.startIntake();


            }
            /*
            if (gamepad.wasDownPressed()) {
                drive.moveCm(drive.MAX_VELOCITY,58, 90, 180,1000);
                drive.moveCm(drive.MAX_VELOCITY,40,  teamUtil.alliance==RED ? 30 : 330, 180,0);
                drive.moveCm(drive.MAX_VELOCITY,10, 270, 180, 0);

                //drive.moveStraightCmWithStrafeEncoder(drive.MAX_VELOCITY, 180, 0, 180, 180, 0);
            }

             */
            if (gamepad.wasOptionsPressed()) {
                drive.moveCm(drive.MAX_VELOCITY, 30, 180, 180,400); // was 81
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.SPARKLY);
                drive.setMotorsFloat(); // coast to wall

                drive.stopMotors();
                teamUtil.pause(250);
                drive.setMotorsBrake();
                //drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad.wasUpPressed()) {
                drive.moveCm(drive.MAX_VELOCITY,71, 90, 180,0); // Was 71
                drive.moveCm(drive.MAX_VELOCITY,28, teamUtil.alliance==RED ? 150:210, 180,0);
                drive.moveCm(drive.MAX_VELOCITY,6, 270, 180, 800);
                drive.moveCm(drive.MAX_VELOCITY,83, teamUtil.alliance==RED ? 30:330, 180,1000);
                drive.driveToAprilTagOffset(1000, 0, 180, teamUtil.alliance==RED ? -drive.TAG_CENTER_TO_CENTER : drive.TAG_CENTER_TO_CENTER, 30, 4000);
                drive.moveCm(drive.MAX_VELOCITY,17, 0, 180, 0);


                //drive.driveToAprilTagOffset(1000, 0, 180, 0, 30, 4000);
                //drive.moveStraightCmWithStrafeEncoder(drive.MAX_VELOCITY, 180, 0, 0, 180, 0);
                //cycle((drive.TAG_CENTER_TO_CENTER));
            }

            if (gamepad.wasLeftPressed()) {
                //drive.findPixelProcesser.reset();
               // output.moveStraferLeft();
                drive.spinToHeading(180);
            }
            if (gamepad.wasRightPressed()) {
                //output.moveStraferRight();
                //intake.testWithFlicker();
                drive.findPixelProcesser.reset();
                teamUtil.log("YELLOW:" + drive.findYellowPixel(1,500));
            }
            /*
            if(gamepad.wasYPressed()){
                output.goToScore(2);
            }

             */
            if(gamepad.wasAPressed()){
                output.goToLoad();
            }

            intake.autoOff();

            if(gamepad.wasBPressed()){
                intake.toggleIntake();
            }

            if(gamepad.wasYPressed()){
                //drive.moveCm(2000, 40,180, 180, 1000);
                //drive.driveToStackNoStop(180,180,1000,500,2000);
                //intake.reverseIntake();

                drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad.wasXPressed()){
                double kickerSpeed = 0;
                double sweeperSpeed = 0;
                boolean grabbed = false;
                output.dropPixels();
                while (!gamepad.wasXPressed() && opModeIsActive()){
                    gamepad.loop();
                    if (gamepad.wasYPressed()) {
                        intake.autoLoadTwoPixelsFromStackV2();
                    }
                    if (gamepad.wasBPressed()) {
                        intake.autoLoadSecondPixelFromStackV2();
                    }
                    if (gamepad.wasAPressed()) {
                        intake.ready();
                        intake.startIntake();
                        output.dropPixels();
                    }
                    if(gamepad.wasUpPressed()){
                        kickerSpeed=kickerSpeed+0.01;
                        intake.kicker.setPower(kickerSpeed);
                    }else if(gamepad.wasDownPressed()){
                        kickerSpeed = kickerSpeed - 0.01;
                        intake.kicker.setPower(kickerSpeed);

                    }else if(gamepad.wasLeftPressed()){
                        sweeperSpeed = sweeperSpeed - 0.1;
                        intake.sweeper.setPower(-sweeperSpeed);
                    }
                    else if(gamepad.wasRightPressed()){
                        sweeperSpeed = sweeperSpeed + 0.1;
                        intake.sweeper.setPower(-sweeperSpeed);
                    }
                    if (gamepad.wasLeftBumperPressed()) {
                        intake.ready();
                    }
                    if (gamepad.wasRightBumperPressed()) {
                        if (intake.kicker.getPower() > .4) {
                            intake.kicker.setPower(kickerSpeed);
                        } else {
                            intake.kicker.setPower(1);
                        }
                    }
                    output.grabber.setPosition(clamp(gamepad.gamepad.right_trigger,output.GrabberOpen, output.GrabberClosed));

                    if (gamepad.wasLeftTriggerPressed()) {
                        intake.toggleIntake();
                    }
                    telemetry.addLine("Sweeper Power: " + sweeperSpeed);
                    telemetry.addLine("Kicker Power: " + kickerSpeed);
                    telemetry.update();
                }
                intake.stopIntake();
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
                /*
                drive.setHeading(180);
                drive.moveCm(2000, 40,180, 180, 350);
                drive.driveToStackNoStopWithStrafe(180,180,1000,2000);

                 */

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
            drive.frontLineCameraDimensionTelemetry();
            telemetry.addLine("Path: "+drive.findTeamPropProcesser.getPropPosition());
            telemetry.addLine("Yellow Pixel Midpoint" + drive.findPixelProcesser.getMidpoint());
            //telemetry.addLine("Yellow Pixel Is" + drive.findYellowPixel(1));
            //teamUtil.log("Yellow Pixel Is " + drive.findYellowPixel(1));

            telemetry.update();
        }
    }
}
