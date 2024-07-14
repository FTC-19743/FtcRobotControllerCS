package org.firstinspires.ftc.teamcode.testCode;
import java.util.Random;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "InitTest", group = "LinearOpMode")
public class InitTest extends LinearOpMode {


    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;


    public void runOpMode() {
        teamUtil.init(this);
        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        armsGamepad = new TeamGamepad();
        armsGamepad.initilize(false);


        String[] arr = { "Wing", "Red", "Win","Delay (Use Bumpers Below) (Press A To Exit Delay Change Mode)","5"};
        int cursorPos = 1;
        int underlinePos = cursorPos;
        int delayValue=0;
        while (!opModeIsActive()) {
            driverGamepad.loop();
            if(driverGamepad.wasDownPressed()){
                if(cursorPos<5){
                    cursorPos+=1;
                } else{
                    cursorPos=1;
                }
            }
            if(driverGamepad.wasUpPressed()){
                if(cursorPos>1){
                    cursorPos-=1;
                }else{
                    cursorPos=5;
                }
            }
            if(cursorPos==5) {
                delayValue=Integer.parseInt(arr[cursorPos-1]);
                while(!driverGamepad.wasAPressed()){
                    if(driverGamepad.wasRightBumperPressed()){
                        delayValue+=1;
                    }else if(driverGamepad.wasLeftBumperPressed()){
                        delayValue-=1;
                    }

                    arr[cursorPos-1]=String.valueOf(delayValue);
                    for (int i = 0; i < arr.length; i++) {
                        int itemNumber = i+1;
                        String selectionSymbol;
                        if(itemNumber==cursorPos){
                            selectionSymbol="✅";
                        }else{
                            selectionSymbol="";

                        }
                        telemetry.addLine(selectionSymbol+" "+arr[i]+" "+selectionSymbol);
                    }
                    telemetry.addLine("\n" + cursorPos);
                    telemetry.addLine("\n" + delayValue);




                    telemetry.update();
                    //telemetry.addLine(""+delayValue);
                    //telemetry.update();
                }
                cursorPos=1;
            }
            if(driverGamepad.wasRightPressed()||driverGamepad.wasLeftPressed()){
                if(arr[cursorPos-1]=="Wing"){
                    arr[cursorPos-1]="Score";
                }else if(arr[cursorPos-1]=="Score"){
                    arr[cursorPos-1]="Wing";
                }else if(arr[cursorPos-1]=="Red"){
                    arr[cursorPos-1]="Blue";
                }else if(arr[cursorPos-1]=="Blue"){
                    arr[cursorPos-1]="Red";
                }else if(arr[cursorPos-1]=="Win"){
                    arr[cursorPos-1]="Loss";
                }else if(arr[cursorPos-1]=="Loss"){
                    arr[cursorPos-1]="Win";
                }else if(arr[cursorPos-1]=="Delay (Use Bumpers Below) (Press A To Exit Delay Change Mode)"){
                }else if(cursorPos==5){




                }
            }

            for (int i = 0; i < arr.length; i++) {
                int itemNumber = i+1;
                String selectionSymbol;
                if(itemNumber==cursorPos){
                    selectionSymbol="✅";
                }else{
                    selectionSymbol="";

                }
                telemetry.addLine(selectionSymbol+" "+arr[i]+" "+selectionSymbol);
            }
            telemetry.addLine("\n" + cursorPos);
            telemetry.addLine("\n" + delayValue);




            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            for (int i = 0; i < arr.length; i++) {
                telemetry.addLine(arr[i]+" ");
            }
            telemetry.update();


        }
    }
}