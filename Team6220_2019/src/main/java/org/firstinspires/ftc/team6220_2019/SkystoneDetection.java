package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

@Autonomous(name = "Skystone Detection")
public class SkystoneDetection extends MasterTeleOp {
    public SkystoneDetectionOpenCV detector;

    @Override
    public void runOpMode() throws InterruptedException {
        detector = new SkystoneDetectionOpenCV();
        detector.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        detector.enable();

        waitForStart();

        while(opModeIsActive()){
            double m1 = detector.getMean1();
            double m2 = detector.getMean2();
            double m3 = detector.getMean3();

            telemetry.addData("Right stone: ", m1);
            telemetry.addData("Middle stone: ", m2);
            telemetry.addData("Yellow stone: ", m3);

            if(m1 < m2 && m1 < m3){
                telemetry.addData("Right stone is skystone", "");
            }
            else if(m2 < m3){
                telemetry.addData("Middle stone is skystone", "");
            }
            else{
                telemetry.addData("Left stone is skystone", "");
            }

            telemetry.update();
        }

        detector.disable();

    }

}
