package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Disabled
@TeleOp(name = "Auto", group = "LinearOpMode")
public class Auto extends LinearOpMode {
    double middleThreshold = 300; // tenative
    double leftThreshold = 300; // tenative
    Robot robot;
    @Override
    public void runOpMode(){
        teamUtil.init(this);
        robot = new Robot();
        robot.initialize();
        robot.output.grabOnePixel();
        teamUtil.pause(1000);
        robot.calibrate();
        robot.drive.initCV();
        robot.drive.runSideTeamPropFinderProcessor();
        while(!opModeIsActive()){
            robot.drive.visionTelemetry();
        }
        waitForStart();
        robot.auto(2,false);



    }
}

