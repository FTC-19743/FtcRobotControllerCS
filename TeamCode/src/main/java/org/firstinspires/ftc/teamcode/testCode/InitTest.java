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


        String[] arr = { "Wing", "Red", "Win", "5"};

        int cursorPos = 1;
        int underlinePos = cursorPos;
        int delayValue=0;
        int delayNumber = Integer.parseInt(arr[3]);
        String newDelayNumber;
        while (!opModeIsActive()) {

            driverGamepad.loop();

            if(driverGamepad.wasDownPressed()){
                if(cursorPos<4){
                    cursorPos+=1;
                } else{
                    cursorPos=1;
                }
            }
            if(driverGamepad.wasUpPressed()){
                if(cursorPos>1){
                    cursorPos-=1;
                }else{
                    cursorPos=4;
                }
            }

            if(driverGamepad.wasRightPressed()){
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
                }else if(arr[cursorPos-1]=="Delay"){
                }else if(cursorPos==4){
                    delayNumber+=1;
                    newDelayNumber = Integer.toString(delayNumber);
                    arr[cursorPos-1]=newDelayNumber;
                }
            }
            if(driverGamepad.wasLeftPressed()){
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
                }else if(arr[cursorPos-1]=="Delay"){
                }else if(cursorPos==4){
                    delayNumber-=1;
                    newDelayNumber = Integer.toString(delayNumber);
                    arr[cursorPos-1]=newDelayNumber;
                }
            }
            /*
            if(driverGamepad.wasRightPressed()&&cursorPos==4){
                delayNumber+=1;
                newDelayNumber = Integer.toString(delayNumber);
                arr[cursorPos-1]=newDelayNumber;

            }
            if(driverGamepad.wasLeftPressed()&&cursorPos==4){
                delayNumber-=1;
                newDelayNumber = Integer.toString(delayNumber);
                arr[cursorPos-1]=newDelayNumber;

            }

             */

            for (int i = 0; i < arr.length; i++) {
                int itemNumber = i+1;
                String selectionSymbol;
                String delay;
                if(i==3){
                    delay = "Delay: ";

                }else{
                    delay="";
                }
                if(itemNumber==cursorPos){
                    selectionSymbol="✅";
                }else{
                    selectionSymbol="";

                }
                telemetry.addLine(delay+selectionSymbol+" "+arr[i]+" "+selectionSymbol);
            }
            //telemetry.addLine("\n" + cursorPos);






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