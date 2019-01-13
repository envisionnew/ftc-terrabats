package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;


@TeleOp(name = "TerraOp: Test")
public class TerraOp extends OpMode{
    //All RobotParts
    Terrabot robot = new Terrabot();
    boolean record = false;
    ArrayList disRight = new ArrayList();
    ArrayList disLeft = new ArrayList();

    @Override
    public void init() {

        robot.init(hardwareMap);

        telemetry.addData("", "Press Start");
        telemetry.update();
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        robot.move(-gamepad1.right_stick_y, -gamepad1.left_stick_x);

        if (gamepad1.a) {
            robot.reset();
            record = true;
        }

        if(record){
            if(robot.left.getCurrentPosition()>0||robot.right.getCurrentPosition()>0){
                if(!robot.right.isBusy()&&!robot.left.isBusy()){
                    disRight.add(robot.right.getCurrentPosition());
                    disLeft.add(robot.left.getCurrentPosition());
                    telemetry.addData("Recorded: R",robot.right.getCurrentPosition());
                    telemetry.addData("Recorded: L",robot.left.getCurrentPosition());
                    telemetry.update();
                    robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }


        }

        if (gamepad1.x){
            record = false;
        }

        if (gamepad1.dpad_up){
            int num = 0;
            while(num < disRight.size()-1){
                robot.apply(1*(int)(disLeft.get(num)),1*(int)(disRight.get(num)));
                num += 1;
            }
            robot.left.setPower(0);
            robot.left2.setPower(0);
            robot.right.setPower(0);
            robot.right2.setPower(0);
        }

        if (gamepad1.dpad_down){
            int num = disRight.size()-1;
            while(num<=0){
                robot.apply(-1*(int)(disLeft.get(num)),-1*(int)(disRight.get(num)));
                num -= 1;
            }
            robot.left.setPower(0);
            robot.left2.setPower(0);
            robot.right.setPower(0);
            robot.right2.setPower(0);
        }

        robot.neverest.setPower(gamepad1.left_trigger);
        robot.neverest.setPower(-gamepad1.right_trigger);

        robot.al.setPower(gamepad2.left_trigger);
        robot.al.setPower(-gamepad2.right_trigger);

        robot.ar.setPower(gamepad2.left_stick_x);
        robot.ar.setPower(-gamepad2.left_stick_y);

    }
    @Override
    public void stop() {
    }

}
