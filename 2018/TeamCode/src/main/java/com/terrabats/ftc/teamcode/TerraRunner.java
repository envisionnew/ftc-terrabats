package org.terrabats.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    ModernRoboticsI2cGyro gyro = null;
    HardwareMap hwMap = null;
    ArrayList<double[]> moves = new ArrayList();
    private int stage = 0;
    public boolean isDropping = false;
    private double oldL = 0;
    public double restPower = 0;
    final double REV_PER_DIS = 0.0796;
    final int TICKS_PER_NEV40 = 1120;
    final double TICKS_FOR_MOVE = REV_PER_DIS*TICKS_PER_NEV40;
    final double TICKS_PER_REVHD = 2240;
    final double MAX_STALL_POWER = 0.25;
    final double START_ANGLE = 51.27;
    final double ARM_GEAR_RATIO = 2.6666667;
    final double TICKS_PER_NEV60 = 1680;
    final double TICKS_EXT_LIMIT = 5900;
    final double INCHS_PER_ROBOT_TURN = 53.407;
    private ElapsedTime timer = new ElapsedTime();
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
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "g");

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

        gyro.calibrate();
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
        l1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move(p,0);
        while (o.opModeIsActive() && (l1.getCurrentPosition() < d)){}
        move(0,0);
    }
    public void turnDeg(double deg ,double p, TurnMode t, LinearOpMode o){
        if(t.equals(TurnMode.ENCODER)) {
            int d = (int) ((deg/360)*TICKS_FOR_MOVE*INCHS_PER_ROBOT_TURN);
            l1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if(deg > 0) {
                l1.setTargetPosition(d);
                r1.setTargetPosition(-d);
            }else{
                l1.setTargetPosition(-d);
                r1.setTargetPosition(d);
            }
            l1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            move(0, p);
            while (o.opModeIsActive() && (l1.isBusy() && r1.isBusy())) {
            }
            move(0, 0);
        }else if(t.equals(TurnMode.GYRO)){
            gyro.resetZAxisIntegrator();
            timer.reset();
            while (o.opModeIsActive() && timer.seconds() < 0.5){}
            if(deg > 0) {
                move(0,-p);
                gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
                while (o.opModeIsActive() && gyro.getHeading() > deg-2){}
            }else{
                move(0,p);
                gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
                while (o.opModeIsActive() && gyro.getHeading() > (-deg)-2){}
            }
            move(0,0);
        }
    }

    public void record(){
        double move[] = new double[2];
        double total_L = (r1.getCurrentPosition()+l1.getCurrentPosition())/2;
        double angle = (l1.getCurrentPosition()-r1.getCurrentPosition())/2;
        move[0] = (total_L/TICKS_FOR_MOVE) - oldL;
        move[1] = (angle/(TICKS_FOR_MOVE*INCHS_PER_ROBOT_TURN))*360;
        oldL = move[0];
        moves.add(move);
    }
    public void goToStart(OpMode o){
        for(double[]i:moves){
            o.telemetry.addData("Len",i[0]);
            o.telemetry.addData("Angle",i[1]);
        }
        o.telemetry.update();
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
    public void resetArmPos(){
        ext.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ext.setPower(-0.1);
        ext.setTargetPosition(0);
        ext.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (ext.isBusy()){

        }
        ext.setPower(0);
    }
    public void intake(double p){
        intake.setPower(p);
    }
    public double getArmAngle(){
        return -((arm.getCurrentPosition()/(TICKS_PER_NEV60*ARM_GEAR_RATIO))*360)+START_ANGLE;
    }
    public void update(){
        restPower = ((ext.getCurrentPosition()/TICKS_EXT_LIMIT))*Math.sin(Math.toRadians(getArmAngle()))*MAX_STALL_POWER;
        oldL = ext.getCurrentPosition();
        minerals();
    }
    public void dropMinerals(){
        isDropping = true;
    }
    private void minerals(){
        if(isDropping) {
            switch (stage) {
                case 0:
                    intake.setPower(-1);
                    timer.reset();
                    stage++;
                    break;
                case 1:
                    if(timer.seconds() > 1){
                        stage++;
                    }
                    break;
                case 2:
                    intake.setPower(0);
                    arm.setPower(-1);
                    stage++;
                    break;
                case 3:
                    if (getArmAngle() > 210) {
                        stage++;
                    }
                    break;
                case 4:
                    arm.setPower(1);
                    stage++;
                    break;
                case 5:
                    if(getArmAngle() < 135){
                        stage++;
                    }
                    break;
                case 6:
                    arm.setPower(0);
                    stage = 0;
                    isDropping = false;
                    break;
            }
        }
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
    private enum ArmMode{
        Auto,
        Manual;
    }
}

