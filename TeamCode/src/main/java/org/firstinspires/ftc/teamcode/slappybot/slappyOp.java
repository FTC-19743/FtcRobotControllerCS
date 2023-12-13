package org.firstinspires.ftc.teamcode.slappybot;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "SlappyOp", group = "LinearOpMode")
public class slappyOp extends LinearOpMode {
    TeamGamepad gamepad;
    DcMotorEx leftMotor, rightMotor;
    Servo headServo;
    AprilTagProcessor aprilTag;
    boolean trackingAprilTags = false;
    VisionPortal visionPortal;
    double servoSkewInc = .01;
    double servoTravelTime = 280f/60f*.15; // Seconds to travel full range
    double servoSkewTimeMs = servoSkewInc * servoTravelTime * 1000;
    long lastServoSkew = 0;
    double HEAD_MIDDLE = 0.518;
    double HEAD_APRIL_TURN = .15;



    DcMotorEx armMotor;

    @Override

    public void runOpMode() {
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        telemetry.addLine("Initializing...");
        telemetry.update();
        setUp();
        headServo.setPosition(0.5);
        telemetry.addLine("Servo Skew Time in ms: " + servoSkewTimeMs);

        telemetry.addLine("Ready! Press Play to Slap");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Tracking April Tags: "+ trackingAprilTags);
            gamepad.loop();
            drive (gamepad.gamepad.left_stick_x, gamepad.gamepad.left_stick_y);
            if (Math.abs(gamepad.gamepad.right_stick_y)> .1f) {
                armMotor.setPower(gamepad.gamepad.right_stick_y);
            } else {
                armMotor.setPower(0);
            }
            if (gamepad.wasYPressed() && !trackingAprilTags) {
                trackingAprilTags = true;
                setManualExposure(5,250);
            } else  if (gamepad.wasAPressed() && trackingAprilTags) {
                trackingAprilTags = false;
                setAutoExposure();
            }
            if (trackingAprilTags) {
                trackAprilTags();
            } else {
                turnHead(gamepad.gamepad.left_trigger, gamepad.gamepad.right_trigger);
            }
            telemetry.update();
        }
        visionPortal.close();
    }

    public void setUp() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "lm");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = hardwareMap.get(DcMotorEx.class, "rm");
        armMotor = hardwareMap.get(DcMotorEx.class, "am");
        headServo = hardwareMap.get(Servo.class,"headServo");
        aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        teamUtil.pause(1500);
        setManualExposure(8,200); // Make April tag processor more responsive
    }
    public void drive (float x, float y) {
        float DEADBAND = 0.1f;
        if (Math.abs(x) > DEADBAND || Math.abs(y) > DEADBAND) {
            leftMotor.setPower(.5*(y * -1 + x));
            rightMotor.setPower(.5*(y * -1 - x));
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
    public void turnHead (float left, float right) {
        if (left > .1) {
            teamUtil.log("skewing left");
            if (System.currentTimeMillis() > lastServoSkew + servoSkewTimeMs && headServo.getPosition() > servoSkewInc) {
                // skew left
                headServo.setPosition(headServo.getPosition()-servoSkewInc*left);
                teamUtil.log("Position: "+(headServo.getPosition()-servoSkewInc));

                lastServoSkew = System.currentTimeMillis();
            }
        } else if (right > .1) {
            if (System.currentTimeMillis() > lastServoSkew + servoSkewTimeMs && headServo.getPosition() < 1-servoSkewInc) {
                // skew right
                headServo.setPosition(headServo.getPosition()+servoSkewInc*right);
                lastServoSkew = System.currentTimeMillis();
            }
        }
    }

    public void trackAprilTags() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.center != null) {
                telemetry.addLine("April Tag x: " + detection.center.x);
                telemetry.addLine("position:" + (HEAD_MIDDLE + (detection.center.x-320f)/320f*HEAD_APRIL_TURN));
                headServo.setPosition(HEAD_MIDDLE + (detection.center.x-320f)/320f*HEAD_APRIL_TURN);
            }
        }
    }

    private boolean    setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }
    private boolean    setAutoExposure() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Auto) {
                exposureControl.setMode(ExposureControl.Mode.Auto);
                sleep(50);
            }

            return (true);
        } else {
            return (false);
        }
    }
}
