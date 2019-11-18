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

        while(opModeIsActive())
        {
            double m1 = detector.getMean1();
            double m2 = detector.getMean2();
            double m3 = detector.getMean3();

            telemetry.addData("Right stone: ", m1);
            telemetry.addData("Middle stone: ", m2);
            telemetry.addData("Left stone: ", m3);

            /*
             *  If the yellow filter value of the right stone is lower than that of the middle or
             *  left stones, then it has more black and is therefore the SkyStone.
             *
             *  Otherwise, if the yellow value of the middle stone is less than that of the left
             *  stone, the middle stone is the SkyStone.
             *
             *  Failing the first two options, the left stone is the SkyStone.
             */
            if(m1 < m2 && m1 < m3)
            {
                telemetry.addData("Right stone is SkyStone", "");
            }
            else if(m2 < m3)
            {
                telemetry.addData("Middle stone is SkyStone", "");
            }
            else
            {
                telemetry.addData("Left stone is SkyStone", "");
            }

            telemetry.update();
        }

        detector.disable();

    }

}
