package org.terrabats.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class TerraRunner {
    DcMotor l1 = null;
    DcMotor l2 = null;
    DcMotor r1 = null;
    DcMotor r2 = null;
    DcMotor arm = null;
    DcMotor ext = null;
    DcMotor hang = null;
    CRServo intake = null;
    HardwareMap hwMap = null;
    ArrayList<double[]> moves = new ArrayList();
    private double oldL = 0;
    public int oldE = 0;
    final int DELAY_MS = 200;
    final double REV_PER_DIS = 0.0796;
    final int TICKS_PER_NEV40 = 1120;
    final double TICKS_FOR_MOVE = REV_PER_DIS*TICKS_PER_NEV40;
    final double TICKS_PER_REVHD = 2240;
    final double TICKS_EXT_LIMIT = 4600;
    final double INCHS_PER_ROBOT_TURN = 53.407;
    public void init(HardwareMap Map){
        hwMap = Map;
        l1 = hwMap.get(DcMotor.class, "l");
        l2 = hwMap.get(DcMotor.class, "l2");
        r1 = hwMap.get(DcMotor.class, "r");
        r2 = hwMap.get(DcMotor.class, "r2");
        hang = hwMap.get(DcMotor.class,"nr");
        arm = hwMap.get(DcMotor.class,"ar");
        ext = hwMap.get(DcMotor.class,"al");
        intake = hwMap.get(CRServo.class, "in");

        l1.setDirection(DcMotor.Direction.FORWARD);
        l2.setDirection(DcMotor.Direction.FORWARD);
        r1.setDirection(DcMotor.Direction.REVERSE);
        r2.setDirection(DcMotor.Direction.REVERSE);
        hang.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);
        ext.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(CRServo.Direction.FORWARD);

        l1.setPower(0);
        l2.setPower(0);
        r1.setPower(0);
        r2.setPower(0);
        hang.setPower(0);
        arm.setPower(0);
        ext.setPower(0);
        intake.setPower(0);

        l1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void move(double p, double o){
        r1.setPower(p - o);
        r2.setPower(p - o);
        l1.setPower(p + o);
        l2.setPower(p + o);
    }
    public void lift(double p){
        arm.setPower(p);
    }
    public void extend(double p){ext.setPower(p);}
    public void hang(double p){hang.setPower(p);}
    public void moveDis(double dis, double p, LinearOpMode o){
        int d = (int) (dis*TICKS_FOR_MOVE);
        l1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l1.setTargetPosition(d);
        r1.setTargetPosition(d);
        l1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        move(p,0);
        while (o.opModeIsActive() && (l1.isBusy() && r1.isBusy())){}
        move(0,0);
    }
    public boolean isExtInLimits(Gamepad g2){
        ExtMode dir = null;
        boolean inLim = true;
        if(0 >= ext.getCurrentPosition()){
            dir = ExtMode.FORWARD;
            inLim = false;
        }else if(ext.getCurrentPosition() >= TICKS_EXT_LIMIT){
            dir = ExtMode.REVERSE;
            inLim = false;
        }
        if(dir != null){
            if(dir == getControllerDirection(g2)){
                inLim = true;
            }
        }
        return inLim;
    }
    public ExtMode getControllerDirection(Gamepad in){
        if(-in.right_stick_y > 0){
            return ExtMode.FORWARD;
        }else if(in.right_stick_y > 0){
            return ExtMode.REVERSE;
        }else{
            return ExtMode.STATIC;
        }
    }
    public void intake(double p){
        intake.setPower(p);
    }
    public void moveDis(double dis, double p, LinearOpMode o){
        int d = (int) (dis*TICKS_FOR_MOVE);
        l1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move(p,0);
        while (o.opModeIsActive() && (l1.getCurrentPosition() > d)){}
        move(0,0);
    }
    private enum ExtMode{
        FORWARD,
        REVERSE,
        STATIC;
    }
    public enum TurnMode {
        GYRO,
        ENCODER;
    }
}

