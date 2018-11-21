package com.terrabats.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="TerraBot: Minreal", group="Terrabot")
public class TerraAuto extends LinearOpMode {
    Terrabot robot = new Terrabot();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();
        while(opModeIsActive()&&runtime.seconds() < 3.2) {
            robot.lift(0.3);
        }
        runtime.reset();
        while(opModeIsActive()&&runtime.seconds() < 0.5) {
            robot.move(0, -0.5);
        }
        robot.moveDis(10,0.5,0.5);

        
        while(opModeIsActive()) {
            detectingGoldMineral();
        }

    }

    public void detectingGoldMineral() {
        GoldAlignDetector gold = new GoldAlignDetector();
        ElapsedTime runtime = new ElapsedTime();
        double result = gold.getXPosition();
        if (result > 150) {
            robot.move(1,.25);
            while(runtime.seconds()<2){}
            robot.move(0,0);
        }   else {
            if (result > 150){
                robot.move(1, -.25);

                robot.move(0,0);
            }   else {

                robot.move(1,0);
                while(runtime.seconds() < 2){};
                robot.move(0,0);
            }

        }
    }
}
