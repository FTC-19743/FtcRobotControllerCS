package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "Shooter", group = "LinearOpMode")
public class shooterOpMode extends LinearOpMode {
    TeamGamepad gamepad;

    DcMotor leftMotor, rightMotor, flyWheel;
    float flyWheelSpeed = 0;
    float leftPower = 0, rightPower = 0;

    @Override
    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        leftMotor = hardwareMap.get(DcMotor.class, "lm");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = hardwareMap.get(DcMotor.class, "rm");
        flyWheel = hardwareMap.get(DcMotor.class, "fwm");
        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            if (gamepad.wasUpPressed()) {
                if (flyWheelSpeed < 0.79f) {
                    flyWheelSpeed = flyWheelSpeed + 0.1f;
                }
            }
            if (gamepad.wasDownPressed()) {
                if (flyWheelSpeed > 0) {
                    flyWheelSpeed = flyWheelSpeed - 0.1f;
                }
            }
            if (gamepad.wasLeftPressed() || gamepad.wasRightPressed()) {
                flyWheelSpeed = 0;
            }
            if (gamepad1.left_stick_y < -0.5) {
                leftPower = .3f;
            } else if (gamepad1.left_stick_y > 0.5) {
                leftPower = -0.3f;
            } else {
                leftPower = 0;
            }
            if (gamepad1.right_stick_y < -0.5) {
                rightPower = .3f;
            } else if (gamepad1.right_stick_y > 0.5) {
                rightPower = -0.3f;
            } else {
                rightPower = 0;
            }
            if (gamepad1.right_stick_y < -0.5 && gamepad1.left_stick_y > 0.5) {
                leftPower = .5f;
                rightPower = -.5f;
            }
            if (gamepad1.left_stick_y < -0.5 && gamepad1.right_stick_y > 0.5) {
                leftPower = -.5f;
                rightPower = .5f;
            }
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
            flyWheel.setPower(flyWheelSpeed);

        }
    }
}
