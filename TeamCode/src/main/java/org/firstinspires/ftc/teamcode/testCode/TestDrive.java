package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "Test Drive", group = "LinearOpMode")
public class TestDrive extends LinearOpMode {
    Drive drive;
    TeamGamepad gamepad;
    int currentCam = 0;
    public void toggleCamera() {
        currentCam++;
        if (currentCam > 3) {
            currentCam = 1;
        }
        switch(currentCam) {
            case 1:
                drive.activateCamera(drive.frontCam);
                break;
            case 2:
                drive.activateCamera(drive.backCam);
                break;
            case 3:
                drive.activateCamera(drive.rightCam);
                break;
            default:
        }
    }

    @Override
    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        drive = new Drive();
        drive.initalize();
        drive.initVisionPortals();
        double velocity = drive.MAX_VELOCITY;
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            drive.aprilTagTelemetry();
            telemetry.addLine("Ultrasonic Distance: "+drive.getUltrasonicDistance());
            //teamUtil.log("largest area: "+drive.findLine.LinePipe.getLargestArea());

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

            if(gamepad.wasRightBumperPressed()){
                toggleCamera();
                //drive.setHeading(180);
                //drive.moveCm(1000, 20, 180, 180, 1000);
                //drive.centerOnAprilTag(8, 40, 0, 180);
            }
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
            /*
            if(gamepad.wasLeftBumperPressed()){
                drive.findMaxVelocity();
            }
            */

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
            telemetry.update();
        }
    }
}
