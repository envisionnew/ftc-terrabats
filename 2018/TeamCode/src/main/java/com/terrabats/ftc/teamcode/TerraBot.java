package com.terrabats.ftc.teamcode;

/*/ Imports /*/

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;


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

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);
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

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*/ The controls give reverse values, thus, we are subtracting powers in the left to turn right since it actually shows up as a negative power, and vice versa /*/
    }

    public void move(double power, double offset) {
        right.setPower(power - offset);
        left.setPower(power + offset);
    }

    public void lift(double power) {
        lift.setPower(power / 2);
    }

    public void intakeLift(double power) {


        if (power > 0) {
            power *= 0.9;
        } else {
            power *= 0.6;
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
    public void detectingGoldMineral() {
        GoldAlignDetector gold = new GoldAlignDetector();
        ElapsedTime runtime = new ElapsedTime();
        double result = gold.getXPosition();
        if (result > 150) {
            move(1,.25);
            while(runtime.seconds()<2){}
            move(0,0);
        }   else {
            if (result > 150){
                move(1, -.25);

                move(0,0);
            }   else {

                move(1,0);
                while(runtime.seconds() < 2){};
                move(0,0);
            }

        }
    }
}
