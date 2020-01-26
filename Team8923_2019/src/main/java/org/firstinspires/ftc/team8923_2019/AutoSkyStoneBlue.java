package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

//@Disabled
@Autonomous(name = "Auto SkyStone Blue", group = "Test")
public class AutoSkyStoneBlue extends MasterAutonomous
{
    SkystoneDetectionOpenCV OpenCV_detector;

    public double m1, m2, m3;
    Stone skystonePlacement = Stone.MIDDLE;


    boolean isDetectingSkystone = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initAuto();

        alliance = Alliance.BLUE;
        double referenceAngle = imu.getAngularOrientation().firstAngle;
        telemetry.clear();
        telemetry.update();

        waitForStart();
        telemetry.clear();

        autoReverseDrive = true;


        // Initialize the skystoneDetector
        OpenCV_detector = new SkystoneDetectionOpenCV();

        // fullscreen display:
        //   app crashes if screen orientation switches from portrait to landscape
        //   screen goes to sleep, and webcam turns off a few minutes after init, and after play
        OpenCV_detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        OpenCV_detector.enable();
        // loop until user presses 'PLAY'
        // show detected mineral position telemetry
        while (!isStarted())
        {
            telemetry.update();
        }

        // run after user presses 'PLAY'
        while (opModeIsActive())
        {
           imuMoveAuto(12,0,1,.5,3);
            imuMoveAuto(0,9,1,.2,3);



            telemetry.addData("Left stone: ", m1);
            telemetry.addData("Middle stone: ", m2);
            telemetry.addData("Right stone: ", m3);

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
                    telemetry.addData("Right stone is SkyStone", "");
                    skystonePlacement = Stone.RIGHT;
                    isDetectingSkystone = false;

                }
                else if(m2 < m3 && m2 < m1)
                {
                    telemetry.addData("Middle stone is SkyStone", "");
                    skystonePlacement = Stone.MIDDLE;
                    isDetectingSkystone = false;
                }
                else if(m3 < m1 && m3 < m2)
                {
                    telemetry.addData("Left stone is SkyStone", "");
                    skystonePlacement = Stone.LEFT;
                    isDetectingSkystone = false;
                }
            }


            telemetry.update();

            collectSkystone(skystonePlacement,alliance);

            imuPivot(referenceAngle,-90,.35,1,4);

            imuMoveAuto(0,14,.3,.3,3);
            grabbersDown();

            sleep(150);
            imuMoveAuto(0,-10,.3,.3,3);
            imuPivot(referenceAngle,-60,.5,.015,1);
            imuMoveAuto(0 ,-23 ,1,.5,3);
            imuPivot(referenceAngle,0,.5,.015,1);
            imuMoveAuto(0 ,16 ,1,.5,3);
            grabbersUp();
            imuMoveAuto(25 ,0 ,1,.7,3);

            moveLift(-1100, 2);
            sleep(500);
            clawOut();
            sleep(600);

            clawUp();
            sleep(200);
            clawIn();

            moveLift(600, 1.2);
            sleep(500);

            imuMoveAuto(0,-40,1,.7,3);
            idle();


            break;


        }

    }
}
