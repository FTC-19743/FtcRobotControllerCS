package org.firstinspires.ftc.teamcode.slappybot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;



@TeleOp(name = "SlappyOp", group = "LinearOpMode")
public class slappyOp extends LinearOpMode {
    TeamGamepad gamepad;
    DcMotorEx leftMotor, rightMotor;
    Servo headServo;
    VisionPortal visionPortal;

    DcMotorEx armMotor;

    @Override

    public void runOpMode() {
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);

        setUp();

        waitForStart();

        while (opModeIsActive()) {
            gamepad.loop();
            drive (gamepad.gamepad.left_stick_x, gamepad.gamepad.left_stick_y);
            if (Math.abs(gamepad.gamepad.right_stick_y)> .1f) {
                armMotor.setPower(gamepad.gamepad.right_stick_y);
            } else {
                armMotor.setPower(0);
            }
            turnHead(gamepad.gamepad.left_trigger, gamepad.gamepad.right_trigger);
        }
        visionPortal.close();
    }

    public void setUp() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "lm");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rm");
        armMotor = hardwareMap.get(DcMotorEx.class, "am");
        headServo = hardwareMap.get(Servo.class,"headServo");
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        visionPortal = builder.build();
    }
    public void drive (float x, float y) {

    }
    public void turnHead (float left, float right) {

    }
}
