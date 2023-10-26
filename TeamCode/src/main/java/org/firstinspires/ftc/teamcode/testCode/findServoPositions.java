package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name = "findServoPositions", group="Linear Opmode")

public class findServoPositions extends LinearOpMode {
    public static final double MAJOR_INCREMENT = 0.05;
    public static final double MINOR_INCREMENT = 0.01;
    public static final double MINOR_MINOR_INCREMENT = 0.001;
    public static double INITIAL_POS = .5;
    public double currentPosition = INITIAL_POS;
    TeamGamepad gamepad;
    private Servo servo;
    private int port = 0;
    private boolean ch = true;

    private void updateServo()
    {
        String name = "Servo";
        if (ch) {
            name = name+"CH";
        } else {
            name = name+"EH";
        }
        name = name + port;
        servo = hardwareMap.servo.get(name);
        servo.setPosition(INITIAL_POS);

    }

    public void runOpMode() {
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        updateServo();
        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            if (gamepad.wasUpPressed() && (currentPosition < 1)) {
                currentPosition = currentPosition + MAJOR_INCREMENT;
                servo.setPosition(currentPosition);
            }
            if (gamepad.wasDownPressed() && (currentPosition > 0)) {
                currentPosition = currentPosition - MAJOR_INCREMENT;
                servo.setPosition(currentPosition);
            }
            if (gamepad.wasLeftPressed() && (currentPosition < 1)) {
                currentPosition = currentPosition + MINOR_INCREMENT;
                servo.setPosition(currentPosition);
            }
            if (gamepad.wasRightPressed() && (currentPosition > 0)) {
                currentPosition = currentPosition - MINOR_INCREMENT;
                servo.setPosition(currentPosition);
            }
            if (gamepad.wasRightBumperPressed() && (currentPosition < 1)) {
                currentPosition = currentPosition + MINOR_MINOR_INCREMENT;
                servo.setPosition(currentPosition);
            }
            if (gamepad.wasLeftBumperPressed() && (currentPosition > 0)) {
                currentPosition = currentPosition - MINOR_MINOR_INCREMENT;
                servo.setPosition(currentPosition);
            }
            if (gamepad.wasXPressed() ){
                ch = !ch;
                updateServo();
            }
            if (gamepad.wasYPressed() ){
                port++;
                if (port == 6) {
                    port = 0;
                }
                updateServo();
            }
            if (gamepad.wasAPressed() ){
                port--;
                if (port == -1) {
                    port = 5;
                }
                updateServo();
            }
            telemetry.addLine("Hub: " + (ch ? "Control" : "Expansion"));
            telemetry.addData("Port: ", port);
            telemetry.addData("Position: ", servo.getPosition());
            telemetry.update();
        }
    }
}



