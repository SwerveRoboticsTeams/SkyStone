package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

@Autonomous(name="OpenCV Test")
public class OpenCVTest extends MasterAutonomous
{
    private OpenCVDetect findSkystone;

    @Override
    public void runOpMode() throws InterruptedException
    {
        findSkystone = new OpenCVDetect();
        // can repl
        // ace with ActivityViewDisplay.getInstance() for fullscreen
        findSkystone.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        findSkystone.setShowCountours(false);
        // start the vision system
        findSkystone.enable();

        telemetry.addData("init", "done");
        telemetry.addData("Skystone Position:", findSkystone.getSksytonePosition());
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            //double goldRectVertices[] = new double[4];
            telemetry.addData("Working", "Yay");


            //int midpoint = goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2;
            telemetry.update();
        }
        findSkystone.disable();
    }
}
