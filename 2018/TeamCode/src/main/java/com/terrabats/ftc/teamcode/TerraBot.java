package com.terrabats.ftc.teamcode;


/*/ Imports /*/

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Terrabot {

    /*/ Define all the motors and variables that are being used in Terrabot /*/

    DcMotor left = null;
    DcMotor right = null;
    DcMotor lift = null;
    DcMotor intakeLift1 = null;
    DcMotor intakeLift2 = null;
    CRServo intake1 = null;
    CRServo intake2 = null;
    Servo outtake = null;
    final int TICKS_PER_REV = 1020;
    final double REV_PER_DIS = 0.0796;

    HardwareMap hwMap = null;

    public void init(HardwareMap Map) {
        hwMap = Map;

        /*/ Naming all of the motors /*/

        left = hwMap.get(DcMotor.class, "l");
        right = hwMap.get(DcMotor.class, "r");
        lift = hwMap.get(DcMotor.class, "lift");
        intakeLift1 = hwMap.get(DcMotor.class, "il1");
        intakeLift2 = hwMap.get(DcMotor.class, "il2");
        intake1 = hwMap.get(CRServo.class, "i1");
        intake2 = hwMap.get(CRServo.class, "i2");
        outtake = hwMap.get(Servo.class, "o");


        /*/ Setting the direction for all the motors /*/

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        intakeLift1.setDirection(DcMotor.Direction.FORWARD);
        intakeLift2.setDirection(DcMotor.Direction.FORWARD);
        intake1.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.FORWARD);
        outtake.setPosition(180);

        /*/ Power of motors upon initialize. /*/

        left.setPower(0);
        right.setPower(0);
        lift.setPower(0);
        intakeLift1.setPower(0);
        intakeLift2.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*/ No encoder being used on the robot, so this line should be placed. /*/

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*/ The controls give reverse values, thus, we are subtracting powers in the left to turn right since it actually shows up as a negative power, and vice versa /*/
    }

    public void move(double power, double offset) {
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setPower(power - offset);
        left.setPower(power + offset);
    }

    public void lift(double power) {
        lift.setPower(power);
    }

    public void intakeLift(double power) {


        if (power > 0) {
            power *= 0.9;
        } else {
            power *= 0.75;
        }
        intakeLift1.setPower(power);
        intakeLift2.setPower(power);
    }

    public void intake(double power) {

        intake1.setPower(power);
        intake2.setPower(power);
    }

    public void out(double pos) {
        outtake.setPosition(pos);
    }
    public void moveDis(double dis, double rp, double lp){

        int d = (int) (dis*REV_PER_DIS*TICKS_PER_REV);



        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setTargetPosition(d);
        right.setTargetPosition(d);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        right.setPower(lp);
        left.setPower(rp);

        while (right.isBusy() && left.isBusy()){}

        left.setPower(0);
        right.setPower(0);
    }
    public void strafe(){
        moveDis(1.5,0,1);
        moveDis(0.5,1,0);
        moveDis(-1.5,0,-1);
        moveDis(-0.5,-1,0);


    }
}
