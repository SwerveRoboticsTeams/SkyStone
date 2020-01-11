package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Locale;

@Autonomous(name="Auto Parking", group = "Swerve")
// @Disabled
public class AutoPark extends MasterAutonomous
{
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        double xMove= 0.0;
        double yMove = 0.0;
        double time = 0.0;

        while(!opModeIsActive()){
            // gamepad 2 controls movement
            if(gamepad2.dpad_right){
                xMove += 100;
                sleep(100);
            }
            if(gamepad2.dpad_left){
                xMove -= 100;
                sleep(100);
            }
            if(gamepad2.dpad_up){
                yMove += 100;
                sleep(100);
            }
            if(gamepad2.dpad_down){
                yMove -= 100;
                sleep(100);
            }
            // gamepad 1 controls delay
            if (gamepad1.dpad_up)
            {
                time += 1000;
                sleep(100);
            }
            if (gamepad1.dpad_down)
            {
                time -= 1000;
                sleep(100);
            }
            telemetry.addData("X Movement(mm):", xMove);
            telemetry.addData("Y Movement(mm):", yMove);
            telemetry.addData("Time(ms):", time);
            telemetry.update();
        }

        waitForStart();

        // delay
        sleep((long)time);

        // movement in mm control with robot
        //move(xMove,0, 0.2, 0.8, 3.0);
        //sleep(100);
        //move(0,yMove,0.2,0.8,3.0);
        //moveMaintainHeading(0,-10000,0,0.5,0.8,3.0);
        moveMaintainHeading(500,0,0,0.5,0.8,3.0);
    }
}

