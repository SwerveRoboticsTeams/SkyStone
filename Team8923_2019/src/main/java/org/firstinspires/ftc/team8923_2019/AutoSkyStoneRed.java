package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;


//@Disabled
@Autonomous(name = "Auto SkyStone Red", group = "Test")
public class AutoSkyStoneRed extends MasterAutonomous
{
    SkystoneDetectionOpenCV OpenCV_detector;

    public double m1, m2, m3;
    Stone skystonePlacement = Stone.MIDDLE;


    boolean isDetectingSkystone = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initAuto();
        alliance = Alliance.RED;
        double referenceAngle = imu.getAngularOrientation().firstAngle;
        telemetry.clear();
        telemetry.update();

        waitForStart();
        telemetry.clear();

        autoReverseDrive = false;

        OpenCV_detector = new SkystoneDetectionOpenCV();
        OpenCV_detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        OpenCV_detector.enable();

        while (!isStarted())
        {
            telemetry.update();
        }

        waitForStart();

        // run after user presses 'PLAY'
        while (opModeIsActive())
        {
            imuMoveAuto(-14,0,1,.3,3);



            telemetry.addData("Right stone: ", m1);
            telemetry.addData("Middle stone: ", m2);
            telemetry.addData("Left stone: ", m3);

            /*
             *  If the yellow filter value of the right stone is lower than that of the middle or
             *  lef t stones, then it has more black and is therefore the SkyStone.
             *
             *  Otherwise, if the yellow value of the middle stone is less than that of the left
             *  stone, the middle stone is the SkyStone.
             *
             *  Failing the first two options, the left stone is the SkyStone.
             */
            while(isDetectingSkystone){
                m1 = OpenCV_detector.getMean1();
                m2 = OpenCV_detector.getMean2();
                m3 = OpenCV_detector.getMean3();

                if(m1 < m2 && m1 < m3)
                {
                    telemetry.addData("RIGHT stone is SkyStone", "");
                    skystonePlacement = Stone.RIGHT;
                    isDetectingSkystone = false;

                }
                else if(m2 < m3 && m2 < m1)
                {
                    telemetry.addData("MIDDLE stone is SkyStone", "");
                    skystonePlacement = Stone.MIDDLE;
                    isDetectingSkystone = false;
                }
                else if(m3 < m1 && m3 < m2)
                {
                    telemetry.addData("LEFT stone is SkyStone", "");
                    skystonePlacement = Stone.LEFT;
                    isDetectingSkystone = false;
                }
            }


            telemetry.update();

            collectSkystone(skystonePlacement, alliance);

            imuPivot(referenceAngle,-90,.35,1,4);

            imuMoveAuto(0,-15,.3,.3,3);
            grabbersDown();

            sleep(150);
            imuMoveAuto(0,13,.3,.3,3);
            imuPivot(referenceAngle,150,.5,.015,1);
            imuMoveAuto(0 ,23 ,1,.5,3);
            imuPivot(referenceAngle,180,.5,.015,1);
            imuMoveAuto(0 ,-16 ,1,.5,3);
            grabbersUp();
            imuMoveAuto(12 ,0 ,1,.7,3);

            moveLift(-1000, 2);
            sleep(500);
            clawOut();
            sleep(600);

            clawUp();
            sleep(200);
            clawIn();

            moveLift(1000, 2);
            sleep(500);
            imuPivot(referenceAngle,180,.5,.015,1);
            imuMoveAuto(6 ,0 ,1,.7,3);

            imuMoveAuto(0,30,1,.7,3);
            idle();

            break;


        }

    }




}
