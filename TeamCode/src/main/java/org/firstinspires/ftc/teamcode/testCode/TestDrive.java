package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Test Drive", group = "LinearOpMode")
public class TestDrive extends LinearOpMode {
    Drive drive;
    TeamGamepad gamepad;

    @Override
    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        drive = new Drive();
        drive.initialize();
        drive.setCamera(2);
        double velocity = drive.MAX_VELOCITY;
        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            telemetry.addLine("Acceleration (a): "+drive.MAX_ACCELERATION);
            telemetry.addLine("Deceleration (b): "+drive.MAX_DECELERATION);
            telemetry.addLine("Minimum Start (x):"+drive.MIN_START_VELOCITY);
            telemetry.addLine("Minimum End (y):"+drive.MIN_END_VELOCITY);
            telemetry.addLine("Velocity (options):"+velocity);
            telemetry.addLine("Heading (right trigger): "+drive.getHeading());
            telemetry.addLine("Up and down move up and down, hold directional button when you press the other button");
            telemetry.addLine("Ultrasonic Distance: "+drive.getUltrasonicDistance());
            telemetry.update();

            if(gamepad.wasRightBumperPressed()){
                drive.spinToHeading(180);
                drive.setCamera(2);
                drive.moveCm(1000, 20, 180, 180, 1000);
                drive.centerOnAprilTag(8, 40, 0);
            }
            if(gamepad1.right_trigger >= .5){
                drive.setHeading(260);
            }
            if(gamepad.wasLeftBumperPressed()){
                drive.findMaxVelocity();
            }

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
            List<AprilTagDetection> currentDetections = drive.aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
        }
    }
}
