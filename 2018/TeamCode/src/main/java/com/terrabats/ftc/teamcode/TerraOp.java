package org.terrabats.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TerraOp: Test")
public class TerraOpTest extends OpMode {
    TerraRunner bot = new TerraRunner();
    ElapsedTime time = new ElapsedTime();
    private boolean record = false;
    @Override
    public void init() {
        bot.init(hardwareMap);
        bot.ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.ext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        time.reset();
    }
    @Override
    public void loop() {
        if(((int)time.milliseconds()) % bot.DELAY_MS == 0){
            bot.oldE = bot.ext.getCurrentPosition();
        }


        bot.move(-gamepad1.right_stick_y,gamepad1.left_stick_x);
        if(bot.isExtInLimits(gamepad2)) {
            bot.extend(-gamepad2.right_stick_y);
        }else {
            bot.extend(0);
        }

        bot.lift(gamepad2.left_stick_y/2);

        if(gamepad2.right_trigger > 0) {
            bot.hang(-gamepad2.right_trigger);
        }else if(gamepad2.left_trigger > 0){
            bot.hang(gamepad2.left_trigger);
        }else{
            bot.hang(0);
        }
        if(gamepad2.right_bumper){
            bot.intake(1);
        }else if(gamepad2.left_bumper){
            bot.intake(-1);
        }else{
            bot.intake(0);
        }
        telemetry.addData("Pos",bot.ext.getCurrentPosition());
        telemetry.addData("Limit", (5*bot.TICKS_PER_REVHD));
    }
    @Override
    public void stop() {
        record = false;
    }
}
