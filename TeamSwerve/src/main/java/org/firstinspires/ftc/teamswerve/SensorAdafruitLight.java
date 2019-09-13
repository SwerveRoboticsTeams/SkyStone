package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;

@TeleOp(name = "Adafruit light sensor", group = "Sensor")
public class SensorAdafruitLight extends LinearOpMode
{
    AdafruitTSL2561LightSensor sensor;

    double lightDetected;
    double rawLightDetected;
    byte sensorState;
    byte sensorID;
    byte timing;

    public void runOpMode() throws InterruptedException
    {
        sensor = hardwareMap.get(AdafruitTSL2561LightSensor.class, "lightsensor");

        waitForStart();

        //sensor.enable();

        while (opModeIsActive())
        {
            //sensor.enable();

            lightDetected = sensor.getLightDetected();
            rawLightDetected = sensor.getLightDetectedRaw();
            sensorState = sensor.getState();
            sensorID = sensor.getDeviceID();
            timing = sensor.getTimingAndGain();

            telemetry.addData("light", lightDetected);
            telemetry.addData("raw", rawLightDetected);
            telemetry.addData("sensor state", sensorState);
            telemetry.addData("sensor ID", sensorID);
            telemetry.addData("timing", timing);

            telemetry.addData("Time", System.currentTimeMillis());
            telemetry.update();
            idle();
        }


    }

}
