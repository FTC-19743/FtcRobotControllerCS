package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Disabled
@TeleOp(name = "Basic Linear Op Mode", group = "LinearOpMode")
public class SampleOpMode extends LinearOpMode {
    TeamGamepad gamepad;

    @Override
    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);

        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();

        }
    }
}
