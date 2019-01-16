package org.firstinspires.ftc.teamcode;

/*/ Define all Imports /*/

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Terrabot {

    /*/ Define all the motors and variables that are being used in Terrabot /*/
    
    DcMotor left = null;
    DcMotor left2 = null;
    DcMotor right = null;
    DcMotor right2 = null;
    DcMotor hang = null;
    DcMotor arm = null;
    DcMotor ext = null;
    ModernRoboticsI2cGyro gyro = null;
    CRServo intake = null;
    Servo phone = null;
    final int TICKS_PER_REV = 1020;
    final double REV_PER_DIS = 0.0796;
    HardwareMap hwMap = null;
    private double oldL = 0;
    public int oldE = 0;
    final int DELAY_MS = 200;
    final int TICKS_PER_NEV40 = 1120;
    final double TICKS_FOR_MOVE = REV_PER_DIS * TICKS_PER_NEV40;
    final double TICKS_PER_REVHD = 2240;
    final double TICKS_EXT_LIMIT = 4600;
    final double INCHS_PER_ROBOT_TURN = 53.407;


    public void init(HardwareMap Map) {
        hwMap = Map;
        /*/ Naming of all the motors /*/
        left2 = hwMap.get(DcMotor.class, "l2");
        left = hwMap.get(DcMotor.class, "l");
        right = hwMap.get(DcMotor.class, "r");
        right2 = hwMap.get(DcMotor.class, "r2");
        hang = hwMap.get(DcMotor.class, "nr");
        arm = hwMap.get(DcMotor.class, "ar");
        ext = hwMap.get(DcMotor.class, "al");
        intake = hwMap.get(CRServo.class, "in");
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "g");
        phone = hwMap.get(Servo.class, "p");

        /*/ Setting the Direction for all Motors (Not Necessary for all motors, specifically DcMotors). /*/
        left.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);
        hang.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(CRServo.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        /*/ Setting the Power for all Motors. (Very Necessary for extension, arm, intake, and hanging. /*/
        
        left.setPower(0);
        left2.setPower(0);
        right.setPower(0);
        right2.setPower(0);
        arm.setPower(0.05);
        ext.setPower(0);
        intake.setPower(0);
        hang.setPower(0);


        /*/ Setting the Power Behavior. Only necessary for the DriveTrain. /*/
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /*/ Encoder being used on the robot, so this line should be placed (for drivetrain). For others, no encoder being used so it is being stated in the code. /*/

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
    /*/ Basic lift method that would lift the arm based on power inputted /*/
    
    public void lift(double p){
        arm.setPower(p);
    }
    
    /*/ Trial method for extension and lift at the same time /*/
    
    public void extension(double p) {
        arm.setPower(p/3);
        ext.setPower(-p);
    }
    
    /*/ Basic extension method that would extend based on power inputted /*/
    
    public void extend(double p) {
        ext.setPower(p);
    }
    
    /*/ Basic hanging method that will lift the hang and lower it based on power inputted /*/
    
    public void hang(double p){
        hang.setPower(p);
    }
    
    /*/ Just easier for our team to call these methods, rather then defining them at the top /*/
    
    private enum Conversions {
        REV_PER_DIS(0.0796),
        TICKS_PER_NEV40(1120),
        TICKS_FOR_MOVE(REV_PER_DIS.get() * TICKS_PER_NEV40.get());

        private double value;

        Conversions(double n) {
            this.value = n;
        }

        public double get() {
            return value;
        }
    }
    /*/ Team marker drop; using the servo. /*/
    
    public void TMDrop() {

    }
    
    /*/ Inside limits or not. /*/
    
    public boolean isExtInLimits(Gamepad g2) {
        ExtMode dir = null;
        boolean inLim = true;
        if (0 >= ext.getCurrentPosition()) {
            dir = ExtMode.FORWARD;
            inLim = false;
        } else if (ext.getCurrentPosition() >= TICKS_EXT_LIMIT) {
            dir = ExtMode.REVERSE;
            inLim = false;
        }
        if (dir != null) {
            if (dir == getControllerDirection(g2)) {
                inLim = true;
            }
        }
        return inLim;
    }

    /*/ Inside limits for Gamepad in /*/
    
    public ExtMode getControllerDirection(Gamepad in) {
        if (-in.right_stick_y > 0) {
            return ExtMode.FORWARD;
        } else if (in.right_stick_y > 0) {
            return ExtMode.REVERSE;
        } else {
            return ExtMode.STATIC;
        }
    }

    /*/ Defining our intake method, via power /*/
    
    public void intake(double p) {
        intake.setPower(p);
    }
    
    /*/ Different modes /*/
    
    private enum ExtMode {
        FORWARD,
        REVERSE,
        STATIC;
    }
}
