package org.firstinspires.ftc.teamcode;

/*/ Imports /*/

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Terrabot {

    /*/ Define all the motors and variables that are being used in Terrabot /*/
    DcMotor left = null;
    DcMotor left2 = null;
    DcMotor right = null;
    DcMotor right2 = null;
    DcMotor neverest = null;
    //RevRoboticsCoreHexMotor arm = null;
    ModernRoboticsI2cGyro gyro = null;
    CRServo intake = null;
    Servo phone = null;
    final int TICKS_PER_REV = 1020;
    final double REV_PER_DIS = 0.0796;
    HardwareMap hwMap = null;



    public void init(HardwareMap Map)   {
        hwMap = Map;
        /*/ Naming all of the motors /*/
        left2 = hwMap.get(DcMotor.class, "l2");
        left = hwMap.get(DcMotor.class, "l");
        right = hwMap.get(DcMotor.class, "r");
        right2 = hwMap.get(DcMotor.class, "r2");
        neverest = hwMap.get(DcMotor.class, "nr");
        //arm = hwMap.get(RevRoboticsCoreHexMotor.class, "arm");
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "g");
        phone = hwMap.get(Servo.class, "p");




        left.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);
        neverest.setDirection(DcMotor.Direction.FORWARD);


        left.setPower(0);
        left2.setPower(0);
        right.setPower(0);
        right2.setPower(0);


        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /*/ No encoder being used on the robot, so this line should be placed. /*/

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        neverest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /*/ The controls give reverse values, thus, we are subtracting powers in the left to turn right since it actually shows up as a negative power, and vice versa /*/
    }

    public void move(double power, double offset) {
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setPower(power - offset);
        right2.setPower(power - offset);
        left.setPower(power + offset);
        left2.setPower(power + offset);
    }


    public void setDis(double dis, double p){
        int d = (int) (dis*REV_PER_DIS*TICKS_PER_REV);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setTargetPosition(d);
        right.setTargetPosition(d);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setPower(p);
        left.setPower(p);

    }
    public void reset(){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void apply(int firstLeftPosition, int firstRightPosition){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setTargetPosition(firstLeftPosition);
        right.setTargetPosition(firstRightPosition);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (left.getTargetPosition()>=0) {
            left.setPower(1);
            left2.setPower(1);
        }   else{
            left.setPower(-1);
            left2.setPower(-1);
        }
        if (right.getTargetPosition()>=0) {
            right.setPower(1);
            right2.setPower(1);
        }   else{
            right.setPower(-1);
            right2.setPower(-1);
        }
        while(left.isBusy() && right.isBusy()){}
        right2.setPower(0);
        right.setPower(0);
        left2.setPower(0);
        left.setPower(0);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void turnDeg(double deg,double p,boolean stop){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (stop && timer.seconds() < 0.5){}
        gyro.resetZAxisIntegrator();
        move(0,p);
        if(deg > 0) {
            gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
            while (stop && gyro.getHeading() < deg-2){}
        }else{
            gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
            while (stop && gyro.getHeading() < (-deg)-2){}
        }
        move(0,0);

    }

    private enum Conversions {
        REV_PER_DIS(0.0796),
        TICKS_PER_NEV40(1120),
        TICKS_FOR_MOVE(REV_PER_DIS.get()*TICKS_PER_NEV40.get());

        private double value;

        Conversions(double n) {
            this.value = n;
        }

        public double get(){
            return value;
        }
    }

    public void moveDis(double dis, double p, LinearOpMode o){
        int d = (int) (dis*Conversions.TICKS_FOR_MOVE.get());
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setTargetPosition(d);
        right.setTargetPosition(d);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        move(p,0);
        while (o.opModeIsActive() && (left.isBusy() || right.isBusy())){}
        move(0,0);
    }

}
