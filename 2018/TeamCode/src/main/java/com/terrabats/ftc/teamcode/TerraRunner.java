package org.firstinspires.ftc.teamcode;
/**
 * Enforcing all of our imports, which allows our team to use all these classes inside this class.
 */
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class TerraRunner {
    /**
     * Defining all of the motors/sensors/variables that will be used in this class
     */
    DcMotor l1; /*/ Left front /*/
    DcMotor l2; /*/ Left back /*/
    DcMotor r1; /*/ Right front /*/
    DcMotor r2; /*/ Right back /*/
    DcMotor arm; /*/ Arm /*/
    DcMotor ext; /*/ Extension /*/
    DcMotor hang; /*/ Hanging /*/
    DcMotor intake; /*/ Intake mechanism /*/
    ModernRoboticsI2cGyro gyro; /*/ Gyro /*/
    CRServo tm; /*/ Servo /*/
    HardwareMap hwMap = null;
    ArrayList<double[]> moves = new ArrayList();
    private int stage = 0;
    public boolean isDropping = false;
    private double oldL = 0;
    public double restPower = 0;
    /*/ The following list of doubles/ints are constants for our encoders. /*/
    final double REV_PER_DIS = 0.0796;
    final int TICKS_PER_NEV40 = 1120;
    final double TICKS_FOR_MOVE = REV_PER_DIS*TICKS_PER_NEV40;
    final double MAX_STALL_POWER = 0.25;
    final double     START_ANGLE = 51.27;
    final double ARM_GEAR_RATIO = 2.6666667;
    final double TICKS_PER_NEV60 = 1680;
    final double TICKS_EXT_LIMIT = 5900;
    final double INCHS_PER_ROBOT_TURN = 53.407;
    private ElapsedTime timer = new ElapsedTime(); /*/ Timer /*/
    public void init(HardwareMap Map){
        hwMap = Map;
        /**
         *  Defining all our our motors with the hardware map and defining them with the configuration on the phone.
         */
        l1 = hwMap.get(DcMotor.class, "l"); /*/ Left front being defined as "l" /*/
        l2 = hwMap.get(DcMotor.class, "l2"); /*/ Left back being defined as "l2" /*/
        r1 = hwMap.get(DcMotor.class, "r"); /*/ Right front being defined as "r" /*/
        r2 = hwMap.get(DcMotor.class, "r2"); /*/ Right back being defined as "r2" /*/
        hang = hwMap.get(DcMotor.class,"nr"); /*/ Hang being defined as "nr" /*/
        arm = hwMap.get(DcMotor.class,"ar"); /*/ Arm being defined as "ar" /*/
        ext = hwMap.get(DcMotor.class,"al"); /*/ Extension being defined as "al" /*/
        intake = hwMap.get(DcMotor.class, "in"); /*/ Intake being defined as "in" /*/
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "g"); /*/ Gyro being defined as "g" /*/
        tm = hwMap.get(CRServo.class, "tm"); /*/ Team Marker Drop being defined as "tm" /*/


        /**
         * Setting the direction for all necessary motors. Default: Forward
         */

        l1.setDirection(DcMotor.Direction.FORWARD); /*/ Left front being set to Forward: Specifically for our TileRunner DriveTrain. /*/
        l2.setDirection(DcMotor.Direction.FORWARD); /*/ Left back being set to Forward: Specifically for our TileRunner DriveTrain. /*/
        r1.setDirection(DcMotor.Direction.REVERSE); /*/ Right front being set to Reverse: Specifically for our TileRunner DriveTrain. /*/
        r2.setDirection(DcMotor.Direction.REVERSE); /*/ Right back being set to Reverse: Specifically for our TileRunner DriveTrain. /*/
        hang.setDirection(DcMotor.Direction.REVERSE); /*/ Hang being set to Reverse. /*/
        arm.setDirection(DcMotor.Direction.REVERSE); /*/ Arm being set to Reverse. /*/
        ext.setDirection(DcMotor.Direction.FORWARD); /*/ Extension being set to Forward. /*/
        intake.setDirection(DcMotor.Direction.FORWARD); /*/ Intake being set to Forward. /*/

        /**
         * Setting the power for all necessary motors. Default: 0
         */

        l1.setPower(0); /*/ Left front being set to 0: Specifically for our TileRunner DriveTrain. /*/
        l2.setPower(0); /*/ Left back being set to 0: Specifically for our TileRunner DriveTrain. /*/
        r1.setPower(0); /*/ Right front being set to 0: Specifically for our TileRunner DriveTrain. /*/
        r2.setPower(0); /*/ Right back being set to 0: Specifically for our TileRunner DriveTrain. /*/
        hang.setPower(0); /*/ Hang power being set to 0. /*/
        arm.setPower(0); /*/ Arm power being set to 0. /*/
        ext.setPower(0); /*/ Extension power being set to 0. /*/
        intake.setPower(0); /*/ Intake power being set to 0. /*/

        /**
         * Setting the ZeroPowerBehavior. Check https://github.com/ftctechnh/ftc_app for more information about this.
         */

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); /*/ Intake ZeroPowerBehavior being set to Brake. /*/
        l1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); /*/ Left Front ZeroPowerBehavior being set to Brake. /*/
        l2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); /*/ Left Back ZeroPowerBehavior being set to Brake. /*/
        r1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); /*/ Right Front ZeroPowerBehavior being set to Brake. /*/
        r2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); /*/ Right Back ZeroPowerBehavior being set to Brake. /*/
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); /*/ Hang ZeroPowerBehavior being set to Brake. /*/
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); /*/ Arm ZeroPowerBehavior being set to Brake. /*/
        ext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); /*/ Extension ZeroPowerBehavior being set to Brake. /*/

        /**
         * Setting the Mode. Check https://github.com/ftctechnh/ftc_app for more information about this. (Mainly: Encoder or No Encoder)
         * Specifically for our team: we are using encoders in our DriveTrain, however we do not want to call the encoders while using TeleOp, so
         * we have claimed it as "WITHOUT_ENCODER" so it won't run with it in TeleOp. This will be changed in a method, for our autonomous.
         */

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /*/ Intake being set to run without an encoder) /*/
        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /*/ Left front being set to run without an encoder) /*/
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /*/ Left back being set to run without an encoder) /*/
        r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /*/ Right front being set to run without an encoder) /*/
        r2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /*/ Right back being set to run without an encoder) /*/
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /*/ Hang being set to run without an encoder) /*/
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /*/ Arm being set to run without an encoder) /*/
        ext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /*/ Extension being set to run without an encoder) /*/

        gyro.calibrate(); /*/ Calibration of the gyro (So it works) /*/
    }

    /**
     * This is the basic move function of our robot. Using both power and offset, we created this with trial and error.
     * This is specifically for our TileRunner.
     * @param p represents power.
     * @param o represents offset.
     */

    public void move(double p, double o){
        r1.setPower(p - o);
        r2.setPower(p - o);
        l1.setPower(p + o);
        l2.setPower(p + o);
    }

    /**
     * This is the lift function for our arm, which allows it to tilt back and dump the minerals into the lander, claiming five points
     * for our team with each mineral.
     * @param p represents power.
     */

    public void lift(double p){
        arm.setPower(p); //Setting the power to P defines the power given off by the gamepad controller, which will
                        // allow it only to go the amount needed.
    }

    /**
     * This is the extension method of our robot, allowing the arm to extend outwards and take in the minerals via our intake.
     * @param p represents power.
     */

    public void extend(double p){
        ext.setPower(p); //Setting the power to P defines the power given off by the gamepad controller, which will
                        // allow it only to go the amount needed.
    }

    /**
     * This is the hanging mechanism of our robot, claiming the 30 points in Autonomous and 50 points in TeleOp for our team.
     * @param p
     */

    public void hang(double p){
        hang.setPower(p); //Setting the power to P defines the power given off by the gamepad controller, which will
                         // allow it only to go the amount needed.
    }

    /**
     * This is the move function of our robot, that USES encoders. This allows us to program with just a certain amount of inches and it will
     * move to that position. This really makes coding easier, as you don't have to use a timer for all your movements.
     * @param dis represents the distance that the robot has to move (in inches).
     * @param p represents the power given.
     * @param o defines the name of our LinearOpMode (Nothing special).
     */

    public void moveDis(double dis, double p, LinearOpMode o){
        // Note: Our team only has an encoder on one of our drive train motors, Left front, which is why that's the only motor being defined.
        int d = (int) (dis*TICKS_FOR_MOVE);
        l1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move(p,0);
        if(p>0){
            while (o.opModeIsActive() && (l1.getCurrentPosition() < d)){}
            move(0,0);
        }   else {
            while(o.opModeIsActive() && (l1.getCurrentPosition() > -d)){}
            move(0,0);
        }
    }

    /**
     * This is the TurnDeg for our robot. This allows our robot to move a certain amount of degrees, rather than turning with just a timer.
     * This improves the accuracy and lowers the time needed to code Autonomous.
     * @param deg defines the amount of degrees wanting to be turned.
     * @param p defines the power.
     * @param t is defined to use the gyro.
     * @param o is simply just the LinearOpMode.
     */
    public void turnDeg(double deg ,double p, TurnMode t, LinearOpMode o){
        if(t.equals(TurnMode.ENCODER)) {
            //Via Encoder
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
            //Via Gyro
            gyro.resetZAxisIntegrator();
            timer.reset();
            while (o.opModeIsActive() && timer.seconds() < 0.5){}
            if(deg > 0) {
                move(0,-p);
                gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
                while (o.opModeIsActive() && gyro.getHeading() < deg-2){}
            }else{
                move(0,p);
                gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
                while (o.opModeIsActive() && gyro.getHeading() < (-deg)-2){}
            }
            move(0,0);
        }
    }

    /**
     * Used for our inverse kinematics.
     */

    public void record(){
        double move[] = new double[2];
        double total_L = (r1.getCurrentPosition()+l1.getCurrentPosition())/2;
        double angle = (l1.getCurrentPosition()-r1.getCurrentPosition())/2;
        move[0] = (total_L/TICKS_FOR_MOVE) - oldL;
        move[1] = (angle/(TICKS_FOR_MOVE*INCHS_PER_ROBOT_TURN))*360;
        oldL = move[0];
        moves.add(move);
    }

    /**
     * Used for our inverse kinematics.
     * @param o defines the OpMode
     */

    public void goToStart(OpMode o){
        for(double[]i:moves){
            o.telemetry.addData("Len",i[0]);
            o.telemetry.addData("Angle",i[1]);
        }
        o.telemetry.update();
    }

    /**
     * These are the limits that we have set up for our robot, so that the arm does not break and the string stays on. This is aside from
     * the MANUAL breakers. This is just a programming limit.
     * @param g2 for Gamepad 2, as the arm is only used in Gamepad 2.
     * @return returns a boolean (true or false) and can be used.
     */

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

    /**
     * Used to get the direction of any Gamepad and can therefore be used in TeleOp to transfer the accurate needs.
     * @param in defined as a normal input.
     * @return the direction.
     */

    public ExtMode getControllerDirection(Gamepad in){
        if(-in.right_stick_y > 0){
            return ExtMode.FORWARD;
        }else if(in.right_stick_y > 0){
            return ExtMode.REVERSE;
        }else{
            return ExtMode.STATIC;
        }
    }

    /**
     * Resets the arm position. Used for testing purposes only.
     */

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

    /**
     * Our mechanism for the intake, which allows the Core Hex Motor to grab minerals.
     * @param p defined as power.
     */

    public void intake(double p){
        intake.setPower(p);
    }

    /**
     * This is used for our arm and basically gets the arm's angle at that current moment. This was used to test what angle we should have our
     * arm set at, so that it would not move on INIT.
     * @return returns the arm angle.
     */

    public double getArmAngle(){
        return -((arm.getCurrentPosition()/(TICKS_PER_NEV60*ARM_GEAR_RATIO))*360)+START_ANGLE;
    }

    /**
     * Update is used to constantly replace the current position of the arm and extension. Was used for testing purposes. Now only shown on
     * Driver Station just for a reference to the drivers and programmers.
     */

    public void update(){
        restPower = ((ext.getCurrentPosition()/TICKS_EXT_LIMIT))*Math.sin(Math.toRadians(getArmAngle()))*MAX_STALL_POWER;
        oldL = ext.getCurrentPosition();
    }

    /**
     * Was used for testing.
     */

    public void dropMinerals(){
        isDropping = true;
    }

    /**
     * An enum type is a special data type that enables for a variable to be a set of predefined constants. Allows us to use these constants.
     * This is why we used "ExtMode getControllerDirection."
     */

    private enum ExtMode{
        FORWARD,
        REVERSE,
        STATIC;
    }
    
    /**
     * An enum type is a special data type that enables for a variable to be a set of predefined constants. Allows us to use these constants.
     * This is why we used "TurnMode t."
     */
    public enum TurnMode {
        GYRO,
        ENCODER;
    }

    /**
     * An enum type is a special data type that enables for a variable to be a set of predefined constants. Allows us to use these constants.
     * Was used for testing purposes only.
     */
    
    private enum ArmMode{
        Auto,
        Manual;
    }
}
