package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TerraOp: Test")
public class TerraOp extends OpMode {
    TerraRunner bot = new TerraRunner();
    ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        bot.init(hardwareMap);
        bot.ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.ext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        time.reset();
    }

    @Override
    public void loop() {
        bot.update();
        bot.move(-gamepad1.right_stick_y, gamepad1.left_stick_x / 2);
        if (bot.isExtInLimits(gamepad2)) {
            bot.extend(-gamepad2.right_stick_y);
        } else {
            bot.extend(0);
        }
        if (gamepad2.y) {
            bot.dropMinerals();
        }
        if (bot.isDropping) {
            if (gamepad2.left_stick_y < 0) {
                bot.lift((gamepad2.left_stick_y / 2) + bot.restPower);
            } else if (gamepad2.left_stick_y > 0) {
                bot.lift((gamepad2.left_stick_y) + bot.restPower);
            } else {
                bot.lift(+bot.restPower);
            }
        }

        if (gamepad1.right_trigger > 0) {
            bot.hang(-gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0) {
            bot.hang(gamepad1.left_trigger);
        } else {
            bot.hang(0);
        }
        if (gamepad2.right_bumper) {
            bot.intake(1);
        } else if (gamepad2.left_bumper) {
            bot.intake(-1);
        } else {
            bot.intake(0);
        }
        telemetry.addData("Angle: ", bot.getArmAngle());
        telemetry.addData("Length: ", bot.ext.getCurrentPosition());
    }

    @Override
    public void stop() {
    }
}

