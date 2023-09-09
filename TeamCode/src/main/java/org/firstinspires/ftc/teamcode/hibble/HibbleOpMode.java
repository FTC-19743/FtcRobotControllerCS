package org.firstinspires.ftc.teamcode.hibble;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name = "Hibble Op Mode", group = "LinearOpMode")
public class HibbleOpMode extends LinearOpMode {
    TeamGamepad gamepad;
    HibbleMotors hibbleMotors;
    @Override

    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        hibbleMotors = new HibbleMotors();
        hibbleMotors.init();
        teamUtil.init(this);
        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            hibbleMotors.motorsDown(gamepad1.left_stick_y*2500);
        }
    }
}
