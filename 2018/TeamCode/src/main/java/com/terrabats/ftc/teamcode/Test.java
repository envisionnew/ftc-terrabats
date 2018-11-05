package org.terrabats.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="TerraBot: Auto Crater", group="Terrabot")
public class Test extends LinearOpMode {
    Terrabot robot = new Terrabot();
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();
        while(opModeIsActive()&&runtime.seconds() < 5) {
            robot.lift(0.2);
        }
    }
}
