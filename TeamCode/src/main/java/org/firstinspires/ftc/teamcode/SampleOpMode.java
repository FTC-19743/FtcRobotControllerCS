package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Disabled
@TeleOp(name = "Sample Op Mode", group = "LinearOpMode")
public class SampleOpMode extends LinearOpMode {
    @Override
    public void runOpMode(){
        teamUtil.init(this);
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
