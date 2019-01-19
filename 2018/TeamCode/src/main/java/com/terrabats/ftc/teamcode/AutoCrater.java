package org.terrabats.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="TerraAuto: Test")
public class TestAuto extends LinearOpMode {
    TerraRunner bot = new TerraRunner();
    ElapsedTime timer = new ElapsedTime();
    private String goldPos = "n";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AWmKObX/////AAABmYb6oMBM9kDCg3sYoFPoVDBmzHX6yr4hMZ49TFMft1lZNH" +
            "brBnNlk1LQt0TnLInh/68lkCxQD6EHxPTOovDxYhjo4CkanvnpIU0HGctTwj9v0S3CSZqTHI0cOlT2SuollZoQkTMrdUY4y3YjtLbiPs" +
            "mYprc9994qPWPR3iltRb+IkHceIuG2yHxNhiNwbIwfllFtfmhXtDWk4Zw1MSI65lLb1qkAHmUE8vatCiT5QY4y8Fnx+MF0GRLM0W8TXH182agrf" +
            "+jBbLx2by32HRfh8dmCST0/Lj+g7Qccpsfv2x8B2tg+i95zYmSg0Slmr4hkZ3f4tIKpFUnFgdBWe08BD1Lt/N+jlerdrsdYM8wHx2ra";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    @Override
    public void runOpMode(){
        bot.init(hardwareMap);
       
   
        initTfod();
        initVuforia();
        telemetry.addData("Done Calibrating","Ready to Start");
        telemetry.update();
        waitForStart();
        timer.reset();
        bot.hang.setPower(0.5);
        while (opModeIsActive() && timer.seconds() < 1.6) { }
        bot.hang.setPower(0);
        bot.moveDis(7,0.8,this);
        bot.turnDeg(-20,0.5,TerraRunner.TurnMode.GYRO,this);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.5) { }
        bot.turnDeg(20,0.5,TerraRunner.TurnMode.GYRO,this);
        getMineralPosition();
        telemetry.addData("MineralPos", goldPos);
        telemetry.update();
        switch (goldPos){
            case "r":
                bot.turnDeg(30,0.5,TerraRunner.TurnMode.GYRO,this);
                bot.moveDis(20,0.5,this);
                break;
            case "c":
                bot.moveDis(30,-0.5,this);
                break;
            case "l":
                bot.turnDeg(-30,0.5,TerraRunner.TurnMode.GYRO,this);
                bot.moveDis(20,0.5,this);
                break;
            default:
                bot.moveDis(30,0.5,this);

        }
        timer.reset();
        bot.hang.setPower(-0.5);
        while (opModeIsActive() && timer.seconds() < 1.6) { }
        bot.hang.setPower(0);


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
                            if(goldX == -1){
            goldPos = "l";
        }else if(goldX < silverX){
            goldPos = "c";
        }else{
            goldPos = "r";
        }
                            

                        }
                    }
                    telemetry.update();
                }
            }
            tfod.shutdown();
        }
        CameraDevice.getInstance().setFlashTorchMode(false);
       
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        CameraDevice.getInstance().setFlashTorchMode(true);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfodParameters.useObjectTracker = true;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
