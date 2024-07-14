package org.firstinspires.ftc.teamcode.testCode;
import java.util.Random;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "GameTest", group = "LinearOpMode")
public class Game extends LinearOpMode {


    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;


    public void runOpMode() {
        teamUtil.init(this);
        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        armsGamepad = new TeamGamepad();
        armsGamepad.initilize(false);

        //telemetry.addLine("Ready to start");

       /*
        while (!opModeIsActive()) {
            driverGamepad.loop();
            if(driverGamepad.wasRightBumperPressed()||driverGamepad.wasLeftBumperPressed()){
                if(teamUtil.alliance == teamUtil.Alliance.BLUE){
                    teamUtil.alliance = teamUtil.Alliance.RED;
                }else{
                    teamUtil.alliance= teamUtil.Alliance.BLUE;
                }
            }
            telemetry.addLine("Ready to start");
            telemetry.addLine("ALLIANCE : "+ teamUtil.alliance);
            telemetry.update();
        }

        */

        waitForStart();

        int cursorX = 0;
        int cursorY = 0;
        String empty = " ";
        String speck = "\uD83C\uDFD8\n";
        String printString;
        String printHouse;
        Random randomNumberX;
        Random rand = new Random();

        // Generate random integers in range 0 to 94
        int starX = rand.nextInt(93);
        int starY = rand.nextInt(17);




        while (opModeIsActive()) {

            String emptySpeckAmountX = new String(new char[4]).replace("\0", empty);
            String emptySpeckAmountY = new String(new char[4]).replace("\0", "\n");
            String emptyAmountX = new String(new char[cursorX]).replace("\0", empty);
            String emptyAmountY = new String(new char[cursorY]).replace("\0", "\n");

            String stickGuy = "\uD83D\uDE00";
            driverGamepad.loop();
            armsGamepad.loop();
            if(driverGamepad.wasRightPressed()){
                if(cursorX==94){

                }else{
                    cursorX+=1;
                }
            }
            if(driverGamepad.wasLeftPressed()){
                if(cursorX==0){

                }else{
                    cursorX-=1;
                }
            }
            if(driverGamepad.wasUpPressed()){
                if(cursorY==0){

                }
                else{
                    cursorY-=1;
                }
            }
            if(driverGamepad.wasDownPressed()){
                if(cursorY==18){

                }else{

                    cursorY+=1;

                }
            }
            printString = emptyAmountY+emptyAmountX+stickGuy;
            printHouse = emptySpeckAmountY+emptySpeckAmountX+speck;
            telemetry.addLine(printString);
            telemetry.addLine(printHouse);
            telemetry.update();


        }
    }
}