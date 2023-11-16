package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@Autonomous(name = "Auto", group = "LinearOpMode")
public class Auto extends LinearOpMode {
    Robot robot;
    TeamGamepad gamepad;

    public void initializeRobot(){
        telemetry.addLine("Initializing Robot");
        teamUtil.telemetry.update();
        telemetry.update();
        robot = new Robot();
        robot.initialize();
    }
    @Override
    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        teamUtil.alliance = teamUtil.Alliance.RED;
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            teamUtil.telemetry.addLine("Check Robot (Stow Output!) and THEN");
            teamUtil.telemetry.addLine("Press A on Game Pad 1 to CALIBRATE");
            teamUtil.telemetry.update();
        }
        initializeRobot();
        robot.calibrate();

        while(!gamepad.wasAPressed()){
            gamepad.loop();
            if(gamepad.wasLeftPressed()){ teamUtil.alliance = teamUtil.Alliance.RED;}
            if(gamepad.wasRightPressed()){ teamUtil.alliance = teamUtil.Alliance.BLUE;}

            teamUtil.telemetry.addLine("RED or BLUE? (use Game Pad 1 DPad)");
            teamUtil.telemetry.addLine(teamUtil.alliance == teamUtil.Alliance.RED ? "RED Alliance" : "BLUE Alliance");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            if(gamepad.wasLeftPressed()){ teamUtil.SIDE = teamUtil.Side.SCORE;}
            if(gamepad.wasRightPressed()){ teamUtil.SIDE = teamUtil.Side.WING;}

            teamUtil.telemetry.addLine(teamUtil.alliance == teamUtil.Alliance.RED ? "RED Alliance" : "BLUE Alliance");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Score or Wing Side? (use Game Pad 1 DPad)");
            teamUtil.telemetry.addLine(teamUtil.SIDE== teamUtil.Side.SCORE  ? "SCORE Side" : "WING Side");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }
        teamUtil.telemetry.addLine("Initializing Cameras and Starting Stream");
        teamUtil.telemetry.addLine("Please wait");
        teamUtil.telemetry.update();
        robot.drive.initCV();
        robot.drive.runSideTeamPropFinderProcessor();

        while(!gamepad.wasAPressed()){
            gamepad.loop();

            if(gamepad.wasLeftPressed()){ robot.output.dropPixels();}
            if(gamepad.wasRightPressed()){robot.output.grabOnePixel();}

            teamUtil.telemetry.addLine(teamUtil.alliance == teamUtil.Alliance.RED ? "RED Alliance" : "BLUE Alliance");
            teamUtil.telemetry.addLine(teamUtil.SIDE== teamUtil.Side.SCORE  ? "SCORE Side" : "WING Side");
            teamUtil.telemetry.addLine("Path: " + robot.drive.findTeamPropProcesser.propPosition);
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Load Pixel (use Game Pad 1 DPad to grab/release)");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }

        while(!opModeIsActive()){
            telemetry.addLine("Ready to Go!");
            telemetry.addLine("Path: "+robot.drive.findTeamPropProcesser.propPosition);
            telemetry.addLine("Side: "+teamUtil.SIDE.toString());
            telemetry.update();
        }

        waitForStart();
        robot.autoV2(robot.drive.findTeamPropProcesser.propPosition,true);

        teamUtil.justRanAuto = true; // avoid recalibration at start of teleop
    }
}

