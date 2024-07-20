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
        Random randomNumberX;
        Random rand = new Random();

        // Generate random integers in range 0 to 94
        int randX = rand.nextInt(9);
        int randY = rand.nextInt(9);

        String[][] board = new String [9][9];

        // Generate random integers in range 0 to 94




        while (opModeIsActive()) {

            for(int y=0;y<9;y++){
                for(int x=0;x<9;x++){
                    board[x][y] = "?";
                }
            }
            for (int r = 0; r<9;r++){
                String line = "";
                for (int c = 0; c <9;c++){
                    line+="["+board[c][r]+"]";
                }
                telemetry.addLine(line);
            }
            telemetry.update();





        }
    }
}