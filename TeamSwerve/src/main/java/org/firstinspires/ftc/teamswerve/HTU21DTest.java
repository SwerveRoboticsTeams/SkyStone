package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Created by Dryw Wade
 *
 * OpMode for testing Adafruit's MCP9808 temperature sensor driver
 */
// TODO Needs to be fully modified for HTU21D humidity and temperature sensor.
@TeleOp(name = "HTU21DTest", group = "Tests")
public class HTU21DTest extends LinearOpMode
{
    private AdafruitHTU21D tempSensor;

    public void runOpMode() throws InterruptedException
    {
        tempSensor = hardwareMap.get(AdafruitHTU21D.class, "HTSensor");

        // Uncomment to use parameter version of driver class. This will require you to respecify
        // the sensor type from MCP9808 to MCP9808Params
//        MCP9808Params.Parameters parameters = new MCP9808Params.Parameters();
//        parameters.hysteresis = MCP9808Params.Hysteresis.HYST_1_5;
//        parameters.alertControl = MCP9808Params.AlertControl.ALERT_ENABLE;
//        tempSensor.initialize(parameters);

        //tempSensor.setTemperatureLimit(24, AdafruitHTU21D.Register.T_LIMIT_LOWER);
        //tempSensor.setTemperatureLimit(26, AdafruitHTU21D.Register.T_LIMIT_UPPER);
        //tempSensor.setTemperatureLimit(25, AdafruitHTU21D.Register.T_LIMIT_CRITICAL);

        waitForStart();

        while (opModeIsActive())
        {
            /*telemetry.addData("Temperature", tempSensor.getTemperature());
            telemetry.addData("", "");

            telemetry.addData("Lower Limit", tempSensor.getTemperatureLimit(AdafruitHTU21D.Register.T_LIMIT_LOWER));
            telemetry.addData("Lower Limit Triggered", tempSensor.lowerLimitTriggered());
            telemetry.addData("Upper Limit", tempSensor.getTemperatureLimit(AdafruitHTU21D.Register.T_LIMIT_UPPER));
            telemetry.addData("Upper Limit Triggered", tempSensor.upperLimitTriggered());
            telemetry.addData("Critical Limit", tempSensor.getTemperatureLimit(AdafruitHTU21D.Register.T_LIMIT_CRITICAL));
            telemetry.addData("Critical Limit Triggered", tempSensor.criticalLimitTriggered());
            telemetry.addData("", "");*/

            telemetry.addData("Config", Integer.toHexString(tempSensor.readShort(AdafruitHTU21D.Register.CONFIGURATION)));
            telemetry.addData("Manufacturer ID", tempSensor.getManufacturerIDRaw());

            telemetry.update();
            idle();
        }
    }
}
