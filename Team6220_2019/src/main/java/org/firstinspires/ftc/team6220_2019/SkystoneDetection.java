package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Skystone Detection")
public class SkystoneDetection extends MasterTeleOp {
    public SkystoneDetectionOpenCV detector;

    @Override
    public void runOpMode() throws InterruptedException {
        detector = new SkystoneDetectionOpenCV();
        detector.init(hardwareMap.appContext, )

        waitForStart();

        while(opModeIsActive()){

        }


    }

}
