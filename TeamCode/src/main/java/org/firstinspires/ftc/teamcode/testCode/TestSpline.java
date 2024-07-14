package org.firstinspires.ftc.teamcode.testCode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "SplineTest", group = "LinearOpMode")
public class TestSpline extends LinearOpMode {
    TeamGamepad gamepad;
    Robot robot;
    Drive drive;
    double MIN_SPLINE_DISTANCE = 5;

    public void runSpline(int maxVelocity, int robotHeading, double[] firstPosition, double[] secondPosition, double[] thirdPosition, double[] fourthPosition, long timeoutTime){
        //drive.MotorData data = new robot.drive.MotorData();
        //double initialPosition = robot.drive.getDriveMotorData(data); TODO: make positional data
        // think of it as robot on the left moving right as x
        // assume already at first point TODO: change this later
        if(firstPosition[0] > secondPosition[0]){
            teamUtil.log("Spine points not in order");
            return;
        }
        if(secondPosition[0] > thirdPosition[0]){
            teamUtil.log("Spine points not in order");
            return;
        }
        if(thirdPosition[0] > fourthPosition[0]){
            teamUtil.log("Spine points not in order");
            return;
        }
        double a, b, c, d;
        double xOffset = 0;
        double yOffset = 0;
        double[][] positions = {firstPosition, secondPosition, thirdPosition, fourthPosition};
        for(int i = 0; i < 4; i++) { // make x and y offset to make solving easier
            if(i == 0){
                xOffset = positions[0][0];
                yOffset = positions[0][1];
            }else{
                if(positions[i][0] < xOffset){
                    xOffset = positions[i][0];
                }
                if(positions[i][1] < yOffset){
                    yOffset = positions[i][1];
                }
            }
        }
        teamUtil.log("yOffset:"+yOffset+" xOffset:"+xOffset);
        d = firstPosition[1]-yOffset;
        // offests simplify the first row of the matrix by immediately finding d
        //rows are in a^3 b^2 c d y value
        double[] secondRow = {Math.pow((secondPosition[0]-xOffset), 3), Math.pow((secondPosition[0]-xOffset), 2), secondPosition[0]-xOffset, d, secondPosition[1]-yOffset};
        double[] thirdRow = {Math.pow((thirdPosition[0]-xOffset), 3), Math.pow((thirdPosition[0]-xOffset), 2), thirdPosition[0]-xOffset, d, thirdPosition[1]-yOffset};
        double[] fourthRow = {Math.pow((fourthPosition[0]-xOffset), 3), Math.pow((fourthPosition[0]-xOffset), 2), fourthPosition[0]-xOffset, d, fourthPosition[1]-yOffset};

        // clear first pivot column
        double factor = thirdRow[0]/secondPosition[0];
        thirdRow = new double[]{thirdRow[0]-secondRow[0]*factor, thirdRow[1]-secondRow[1]*factor, thirdRow[2]-secondRow[2]*factor, thirdRow[3]-secondRow[3]*factor, thirdRow[4]-secondRow[4]*factor};

        factor = fourthRow[0]/secondPosition[0];
        fourthRow = new double[]{fourthRow[0]-secondRow[0]*factor, fourthRow[1]-secondRow[1]*factor, fourthRow[2]-secondRow[2]*factor, fourthRow[3]-secondRow[3]*factor, fourthRow[4]-secondRow[4]*factor};

        //lead with 1
        factor = 1/secondRow[0];
        secondRow = new double[]{secondRow[0]*factor, secondRow[1]*factor, secondRow[2]*factor, secondRow[3]*factor, secondRow[4]*factor};

        // clear second pivot column
        factor = fourthRow[1]/thirdRow[1];
        fourthRow = new double[]{fourthRow[0]-thirdRow[0]*factor, fourthRow[1]-thirdRow[1]*factor, fourthRow[2]-thirdRow[2]*factor, fourthRow[3]-thirdRow[3]*factor, fourthRow[4]-thirdRow[4]*factor};

        // lead with 1
        factor = 1/thirdRow[1];
        thirdRow = new double[]{thirdRow[0]*factor, thirdRow[1]*factor, thirdRow[2]*factor, thirdRow[3]*factor, thirdRow[4]*factor};

        // lead with 1
        factor = 1/fourthRow[2];
        fourthRow = new double[]{fourthRow[0]*factor, fourthRow[1]*factor, fourthRow[2]*factor, fourthRow[3]*factor, fourthRow[4]*factor};

        // get rid of stuff above pivots
        factor = thirdRow[2]/fourthPosition[2];
        thirdRow = new double[]{thirdRow[0]-fourthRow[0]*factor, thirdRow[1]-fourthRow[1]*factor, thirdRow[2]-fourthRow[2]*factor, thirdRow[3]-fourthRow[3]*factor, thirdRow[4]-fourthRow[4]*factor};

        factor = secondRow[2]/fourthPosition[2];
        secondRow = new double[]{secondRow[0]-fourthRow[0]*factor, secondRow[1]-fourthRow[1]*factor, secondRow[2]-fourthRow[2]*factor, secondRow[3]-fourthRow[3]*factor, secondRow[4]-fourthRow[4]*factor};

        factor = secondRow[1]/thirdPosition[1];
        secondRow = new double[]{secondRow[0]-thirdRow[0]*factor, secondRow[1]-thirdRow[1]*factor, secondRow[2]-thirdRow[2]*factor, secondRow[3]-thirdRow[3]*factor, secondRow[4]-thirdRow[4]*factor};

        // isolate a/b/c = a number
        a = secondRow[4] - secondRow[3];
        b = thirdRow[4] - secondRow[3];
        c = thirdRow[4] - secondRow[3];

        double distance = 500; // make this work
        while(teamUtil.keepGoing(timeoutTime) || distance <= MIN_SPLINE_DISTANCE){
            // strafe is dependant on forwards
            double forwardsPosition = robot.drive.forwardEncoder.getCurrentPosition()-xOffset;
            // math i swear i understand that doesnt work
            //double slope = 3*a*Math.pow(forwardsPosition, 2) + 2*b*forwardsPosition+c;
            //double yValue = a*Math.pow(forwardsPosition, 3)+b*Math.pow(forwardsPosition, 2)+ c*forwardsPosition+d;
            // y = slope(x - forwardsPostition) - yValue
        }
    }

    @Override
    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        robot.initialize();

        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();

        }
    }
}
