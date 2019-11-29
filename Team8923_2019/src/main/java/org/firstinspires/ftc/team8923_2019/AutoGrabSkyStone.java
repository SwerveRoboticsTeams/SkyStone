package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;



@Autonomous(name="GrabSkyStone", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutoGrabSkyStone extends MasterAutonomous
{

    public SkystoneDetectionOpenCV detector;

    @Override
    public void runOpMode() throws InterruptedException
    {
        configureAutonomous();
        initAuto();
        telemetry.clear();
        telemetry.update();
        reverseDrive = false;

        detector = new SkystoneDetectionOpenCV();
        detector.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        detector.enable();

        waitForStart();
        telemetry.clear();


        while (opModeIsActive())
        {
            // MoveTowardSkyStone
            moveAuto(300, 0, .5, .1, 3);

            // Track SkyStone
            double m1 = detector.getMean1();
            double m2 = detector.getMean2();
            double m3 = detector.getMean3();

            telemetry.addData("Right stone: ", m1);
            telemetry.addData("Middle stone: ", m2);
            telemetry.addData("Left stone: ", m3);

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


            break;
        }

    }
    private void moveToSkyStone(){
        // Moves robot forward toward skyStone and puts the 3 blocks in frame


    }
    private void collectSkyStone(){
        // Determines which stone is the skyStone then collects it, Backs up, pushes it in and grabs it with the grabber


    }

    private void moveToFoundation(){
        // turn robot so intake is toward foundation, moves to foundation side
    }

    private void pullFoundation(){
        // turn toward foundation and pulls it to foundation park area
    }

    private void placeBlockInFoundation(){
        // place Block in Foundation
    }
    private void parkOnLine(){
        // park on line

    }

}
