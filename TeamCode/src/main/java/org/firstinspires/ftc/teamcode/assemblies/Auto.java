package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@Autonomous(name = "Auto", group = "LinearOpMode")
public class Auto extends LinearOpMode {
    Robot robot;
    TeamGamepad gamepad;

    int delay = 0;
    boolean cycle = true;

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
        //Calibrate
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            teamUtil.telemetry.addLine("Check Robot (Stow Output!) and THEN");
            teamUtil.telemetry.addLine("Press A on Game Pad 1 to CALIBRATE");
            teamUtil.telemetry.update();
        }
        initializeRobot();
        robot.calibrate();
        teamUtil.robot = robot;

        //Delay
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            if(gamepad.wasLeftPressed()){ if (delay > 0) delay--; }
            if(gamepad.wasRightPressed()){ if (delay < 10) delay++;}

            teamUtil.telemetry.addLine("Delayed Start? (use Game Pad 1 DPad L/R)");
            teamUtil.telemetry.addLine("Delay Seconds: " +delay);
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }
        //Cycle
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            if(gamepad.wasLeftPressed()||gamepad.wasRightPressed()){cycle=!cycle;}


            teamUtil.telemetry.addLine("Cycle? (use Game Pad 1 DPad L/R)");
            teamUtil.telemetry.addLine("Cycle: " +cycle);
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }
        //Alliance
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            if(teamUtil.alliance == teamUtil.Alliance.RED){
                teamUtil.theBlinkin.setSignal((Blinkin.Signals.RED));
            }
            else{
                teamUtil.theBlinkin.setSignal((Blinkin.Signals.BLUE_PATH_1));

            }
            if(gamepad.wasLeftPressed()){ teamUtil.alliance = teamUtil.Alliance.RED;}
            if(gamepad.wasRightPressed()){ teamUtil.alliance = teamUtil.Alliance.BLUE;}

            teamUtil.telemetry.addLine("Delay Seconds: " +delay);
            teamUtil.telemetry.addLine("Cycle: " +cycle);
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("RED or BLUE? (use Game Pad 1 DPad L/R)");
            teamUtil.telemetry.addLine(teamUtil.alliance == teamUtil.Alliance.RED ? "RED Alliance" : "BLUE Alliance");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }
        //SCORE or WING
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            if(gamepad.wasLeftPressed()){ teamUtil.SIDE = teamUtil.Side.SCORE;}
            if(gamepad.wasRightPressed()){ teamUtil.SIDE = teamUtil.Side.WING;}

            teamUtil.telemetry.addLine("Delay Seconds: " +delay);
            teamUtil.telemetry.addLine("Cycle: " +cycle);
            teamUtil.telemetry.addLine(teamUtil.alliance == teamUtil.Alliance.RED ? "RED Alliance" : "BLUE Alliance");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Score or Wing Side? (use Game Pad 1 DPad L/R)");
            teamUtil.telemetry.addLine(teamUtil.SIDE== teamUtil.Side.SCORE  ? "SCORE Side" : "WING Side");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }
        teamUtil.telemetry.addLine("Initializing Cameras and Starting Stream");
        teamUtil.telemetry.addLine("Please wait");
        teamUtil.telemetry.update();
        robot.drive.initCV(false); // no live stream enabled means better FPS
        robot.drive.switchCV(Drive.cvCam.SIDE_PROP);

        //RELEASER
        while(!gamepad.wasAPressed()&&!robot.releaser.holding){
            gamepad.loop();

            if(gamepad.wasUpPressed()){robot.releaser.toggle();}
            if(!robot.releaser.holding){
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.FLASHING_RED);
            }else{
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.VIOLET);
            }

            teamUtil.telemetry.addLine("Delay Seconds: " +delay);
            teamUtil.telemetry.addLine("Cycle: " +cycle);
            teamUtil.telemetry.addLine(teamUtil.alliance == teamUtil.Alliance.RED ? "RED Alliance" : "BLUE Alliance");
            teamUtil.telemetry.addLine(teamUtil.SIDE== teamUtil.Side.SCORE  ? "SCORE Side" : "WING Side");
            teamUtil.telemetry.addLine(robot.releaser.holding ? "Releaser Holding Purple Pixel" : "Releaser NOT holding Purple Pixel");
            teamUtil.telemetry.addLine("Path: " + robot.drive.findTeamPropProcesser.getPropPosition());
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Grab Purple Pixel (use Game Pad 1 DPad U to toggle releaser)");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }
        //YELLOW PIXEL
        while(!gamepad.wasAPressed() && !robot.intake.bottomPixelPresent()){
            gamepad.loop();

            if(gamepad.wasLeftPressed()){ robot.output.dropPixels();}
            if(gamepad.wasRightPressed()){robot.output.grabOnePixel();}

            if(robot.intake.bottomPixelPresent() == false){
                teamUtil.theBlinkin.setSignal((Blinkin.Signals.GOLD));
            }
            else{
                teamUtil.theBlinkin.setSignal((Blinkin.Signals.OFF));
            }

            teamUtil.telemetry.addLine("Delay Seconds: " +delay);
            teamUtil.telemetry.addLine("Cycle: " +cycle);
            teamUtil.telemetry.addLine(teamUtil.alliance == teamUtil.Alliance.RED ? "RED Alliance" : "BLUE Alliance");
            teamUtil.telemetry.addLine(teamUtil.SIDE== teamUtil.Side.SCORE  ? "SCORE Side" : "WING Side");
            teamUtil.telemetry.addLine(robot.releaser.holding ? "Releaser Holding Purple Pixel" : "Releaser NOT holding Purple Pixel");
            teamUtil.telemetry.addLine("Path: " + robot.drive.findTeamPropProcesser.getPropPosition());
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Load Pixel (use Game Pad 1 DPad L/R to grab/release) - DON'T GRAB ON WING!!!!!!!!");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }


        while(!opModeIsActive()){
            int path = robot.drive.findTeamPropProcesser.getPropPosition();

            telemetry.addLine("Ready to Go!");
            telemetry.addLine("Delay Seconds: "+delay);
            teamUtil.telemetry.addLine("Cycle: " +cycle);
            telemetry.addLine("Alliance: "+teamUtil.alliance.toString());
            telemetry.addLine("Side: "+teamUtil.SIDE.toString());
            telemetry.addLine("Path: "+path);
            telemetry.update();




            if(path == 1 && teamUtil.alliance == teamUtil.alliance.RED){
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_PATH_1);
            } else if (path == 2 && teamUtil.alliance == teamUtil.alliance.RED) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_PATH_2);
            } else if (path == 3 && teamUtil.alliance == teamUtil.alliance.RED) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_PATH_3);
            } else if (path == 1 && teamUtil.alliance == teamUtil.alliance.BLUE) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_PATH_1);
            } else if (path == 2 && teamUtil.alliance == teamUtil.alliance.BLUE) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_PATH_2);
            } else{
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_PATH_3);
            }
        }

        waitForStart();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        robot.drive.stopCV(); // shut down prop detector
         // Delay start if needed
        long startTime = System.currentTimeMillis();
        robot.autoV4(robot.drive.findTeamPropProcesser.getPropPosition(), true,delay,cycle);
        long endTime = System.currentTimeMillis();
        long elapsedTime = endTime-startTime;
        teamUtil.log("Elapsed Auto Time Without Wait At End: " + elapsedTime);

        while(opModeIsActive()){
        }
        if(!robot.output.loading.get()) {
            teamUtil.log("USING SECRET CHEATER TIME");
            teamUtil.pause(400);
        }
        teamUtil.justRanAuto = true; // avoid recalibration at start of teleop
        if(robot.output.loading.get()){
            teamUtil.finishedGoToLoad = true;
        }else{
            teamUtil.finishedGoToLoad = false;
        }
    }
}

