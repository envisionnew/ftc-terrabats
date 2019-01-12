package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name="TerraAuto: Crater")
public class AutoCrater extends LinearOpMode {
    Terrabot robot = new Terrabot();
    ElapsedTime timer = new ElapsedTime();
    private String goldPos = "c";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AWmKObX/////AAABmYb6oMBM9kDCg3sYoFPoVDBmzHX6yr4hMZ49TFMft1lZNHbrBnNlk1LQt0TnLInh/68lkCxQD6EHxPTOovDxYhjo4CkanvnpIU0HGctTwj9v0S3CSZqTHI0cOlT2SuollZoQkTMrdUY4y3YjtLbiPsmYprc9994qPWPR3iltRb+IkHceIuG2yHxNhiNwbIwfllFtfmhXtDWk4Zw1MSI65lLb1qkAHmUE8vatCiT5QY4y8Fnx+MF0GRLM0W8TXH182agrf+jBbLx2by32HRfh8dmCST0/Lj+g7Qccpsfv2x8B2tg+i95zYmSg0Slmr4hkZ3f4tIKpFUnFgdBWe08BD1Lt/N+jlerdrsdYM8wHx2ra";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        robot.init(hardwareMap);
        initTfod();
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();
        robot.gyro.calibrate();

        while (!isStopRequested() && robot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }
        telemetry.addData("!!!", "Done Calibrating");
        telemetry.update();
        robot.gyro.resetZAxisIntegrator();
        waitForStart();
        /* Hanging code */
        timer.reset();
        while(opModeIsActive()&& timer.seconds() < 2) {
            robot.neverest.setPower(0.5);
        }
        robot.neverest.setPower(0);
        timer.reset();
        robot.move(0,0);
        telemetry.addData("IsGyro",(robot.gyro.getHeading() < 358));
        telemetry.update();
        getMineralPosition();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 3){
            telemetry.addData("GoldPos",goldPos);
            telemetry.update();
        }
        switch (goldPos){
            case "r":
                telemetry.addLine("going to the right");
                timer.reset();
                turnDeg(20, 0.3);
                timer.reset();
                while(opModeIsActive()&&timer.seconds()< 1.5) {
                    robot.move(1, 0);
                }
                robot.move(0,0);
                break;
            case "c":
                telemetry.addLine("going straight");
                timer.reset();
                while(opModeIsActive()&&timer.seconds()< 1.5) {
                    robot.move(1, 0);
                }
                robot.move(0,0);
                break;
            case "l":
                telemetry.addLine("going to the left");
                timer.reset();
                turnDeg(-20, -0.3);
                timer.reset();
                while(opModeIsActive()&&timer.seconds()< 1.5) {
                    robot.move(1, 0);
                }
                robot.move(0,0);
                break;
            default:
                telemetry.addLine("going straight");
                timer.reset();
                while(opModeIsActive()&&timer.seconds()< 1.5) {
                    robot.move(1, 0);
                }
                robot.move(0,0);
                break;
        }
    }
    private void getMineralPosition(){
        int goldX = -1;
        int silverX = -1;
        timer.reset();
        if (opModeIsActive()) {
            tfod.activate();
            while (opModeIsActive() && goldX == -1 && silverX == -1 && timer.seconds() < 10) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if(updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 2) {
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldX = (int)recognition.getLeft();
                            }else{
                                silverX = (int)recognition.getLeft();
                            }
                        }
                        if(goldX == -1){
                            goldPos = "l";
                        }else if(goldX < silverX){
                            goldPos = "r";
                        }else{
                            goldPos = "c";
                        }
                    }
                    telemetry.update();
                }
            }
            tfod.shutdown();
        }
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void turnDeg(double deg, double power) {
        if (deg < 0) {
            robot.gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
            deg = -deg;
        } else {
            robot.gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
        }
        robot.move(0, power);
        while (opModeIsActive() && robot.gyro.getHeading() < deg) {
            telemetry.addData("Gyro: ", robot.gyro.getHeading());
            telemetry.update();
        }
        robot.move(0, 0);
        robot.gyro.resetZAxisIntegrator();
    }
}

