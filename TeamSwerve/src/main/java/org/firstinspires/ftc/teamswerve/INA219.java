package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;
import java.util.Locale;

/* Implementation by Dryw Wade */

@SuppressWarnings("WeakerAccess")
@I2cSensor(name = "INA219 Current Sensor", description = "Current Sensor from Adafruit", xmlTag = "INA219")

/*
This is the class for the INA219 Current Sensor, plugged into port 1 of the REV module on
the dynamometer project.  Here's the link to the data sheet:
https://cdn-shop.adafruit.com/datasheets/ina219.pdf.
 */

public class INA219 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements I2cAddrConfig
{

    //----------------------------------------------------------------------------------------------
    // User Methods
    //----------------------------------------------------------------------------------------------

    /*
    This method returns current, which is voltage divided by resistance (0.02 ohm), according to
    Ohm's Law.
     */
    public double current()
    {
        // I=V/R
        //return shuntVoltage() / shuntResistance;
        return shuntVoltage() * 50; // 10 is derived from 3.2 amps divided by 0.32 max shunt voltage
        // 50 is derived from 16 amps divided by 0.32 max shunt voltage
    }

    /*
    This method returns power, which is the product of current and voltage.
     */
    public double power()
    {
        // P=IV
        return current() * busVoltage();
    }

    /*
    This method returns shunt voltage, by reading the raw value from the INA219 current sensor
    and by scaling it to a voltage.
     */
    public double shuntVoltage()
    {
        // Shunt voltage is signed
        return Range.scale(rawShuntVoltage(), -MAX_SHUNT_VOLTAGE_RAW, MAX_SHUNT_VOLTAGE_RAW, -MAX_SHUNT_VOLTAGE, MAX_SHUNT_VOLTAGE);
    }

    /*
    This method returns bus voltage, by reading the raw value from the current sensor and by
    scaling it to a bus voltage.
     */
    public double busVoltage()
    {
        // bus voltage is unsigned (always >= 0)
        //return Range.scale(rawBusVoltage(), 0, MAX_BUS_VOLTAGE_RAW, 0, MAX_BUS_VOLTAGE);
        return rawBusVoltage() * 0.004 /*4 mV, the LSB*/;
    }

    /*
    This method writes to the configuration register to reset the INA219 device.
     */
    public void reset()
    {
        writeShort(Register.CONFIGURATION, (short) 0x8000);
    }

    //----------------------------------------------------------------------------------------------
    // Raw Data
    //----------------------------------------------------------------------------------------------

    /*
    This method reads the raw shunt voltage value from the INA219 device.
     */
    public int rawShuntVoltage()
    {
        return readShort(Register.SHUNT_VOLTAGE);
    }

    /*
    This method reads the raw bus voltage from the INA219 device.
     */
    public int rawBusVoltage()
    {
        // Bus voltage is unsigned (always positive).
        // Unsigned shift right by 3 bits to remove unused bits 2:0; data is in bits 15:3.
        return readShort(Register.BUS_VOLTAGE) >>> 3;
    }

    /*
    This method returns the raw current value from the INA219 device.  The device computes the
    current using the calibration register, but it must be programmed before reading the current
    register.
     */
    public int rawCurrent()
    {
        return readShort(Register.CURRENT);
    }

    /*
    This method returns the raw power from the INA219 device.  The device computes the power using
    the calibration register, but it must be programmed before reading the current register.
     */
    public int rawPower()
    {
        return readShort(Register.POWER);
    }

    /*
    This method reads the configuration register.
     */
    public int rawConfig()
    {
        return readShort(Register.CONFIGURATION);
    }

    //----------------------------------------------------------------------------------------------
    // Registers
    //----------------------------------------------------------------------------------------------

    /*
    This is the list of registers for INA219.
     */
    public enum Register
    {
        FIRST(0),
        CONFIGURATION(0x00),
        SHUNT_VOLTAGE(0x01),
        BUS_VOLTAGE(0x02),
        CURRENT(0x03),
        POWER(0x04),
        CALIBRATION(0X05),
        LAST(CALIBRATION.bVal),
        UNKNOWN(-1);

        public int bVal; // defines an integer "bit value"

        Register(int bVal) // a constructor that allows user to pass in a bit value
        {
            this.bVal = bVal;
        }
    }

    public enum BusVoltageRange
    {
        V16(0x0000),  // 0-16V Range
        V32(0x2000);  // 0-32V Range

        public int bVal;

        BusVoltageRange(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public enum Gain
    {
        GAIN_1_40MV(0x0000),  // Gain 1, 40mV Range
        GAIN_2_80MV(0x0800),  // Gain 2, 80mV Range
        GAIN_4_160MV(0x1000),  // Gain 4, 160mV Range
        GAIN_8_320MV(0x1800);  // Gain 8, 320mV Range

        public int bVal;

        Gain(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public enum BusResolution
    {
        RES_9BIT(0x0080),  // 9-bit bus res = 0..511
        RES_10BIT(0x0100),  // 10-bit bus res = 0..1023
        RES_11BIT(0x0200),  // 11-bit bus res = 0..2047
        RES_12BIT(0x0400);  // 12-bit bus res = 0..4095

        public int bVal;

        BusResolution(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public enum ShuntResolution
    {
        RES_9BIT_1S_84US(0x0000),  // 1 x 9-bit shunt sample
        RES_10BIT_1S_148US(0x0008),  // 1 x 10-bit shunt sample
        RES_11BIT_1S_276US(0x0010),  // 1 x 11-bit shunt sample
        RES_12BIT_1S_532US(0x0018),  // 1 x 12-bit shunt sample
        RES_12BIT_2S_1060US(0x0048),     // 2 x 12-bit shunt samples averaged together
        RES_12BIT_4S_2130US(0x0050),  // 4 x 12-bit shunt samples averaged together
        RES_12BIT_8S_4260US(0x0058),  // 8 x 12-bit shunt samples averaged together
        RES_12BIT_16S_8510US(0x0060),  // 16 x 12-bit shunt samples averaged together
        RES_12BIT_32S_17MS(0x0068),  // 32 x 12-bit shunt samples averaged together
        RES_12BIT_64S_34MS(0x0070),  // 64 x 12-bit shunt samples averaged together
        RES_12BIT_128S_69MS(0x0078);  // 128 x 12-bit shunt samples averaged together

        public int bVal;

        ShuntResolution(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public enum Mode
    {
        POWERDOWN(0x0000),
        SVOLT_TRIGGERED(0x0001),
        BVOLT_TRIGGERED(0x0002),
        SANDBVOLT_TRIGGERED(0x0003),
        ADCOFF(0x0004),
        SVOLT_CONTINUOUS(0x0005),
        BVOLT_CONTINUOUS(0x0006),
        SANDBVOLT_CONTINUOUS(0x0007);

        public int bVal;

        Mode(int bVal)
        {
            this.bVal = bVal;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Variables and Constants
    //----------------------------------------------------------------------------------------------

    public double shuntResistance = 0.1;

    protected static final double MAX_SHUNT_VOLTAGE = 0.320; // 320 mV
    protected static final double MAX_SHUNT_VOLTAGE_RAW = 32000; // 320mV * 100; LSB is 10uV
    protected static final double SHUNT_VOLTAGE_RESOLUTION = MAX_SHUNT_VOLTAGE / MAX_SHUNT_VOLTAGE_RAW;
    protected static final double MAX_BUS_VOLTAGE = 32; // 0V to 32V range
    // 4000 for 16V range, 8000 for 32V range; LSB = 4mV for both ranges
    protected static final double MAX_BUS_VOLTAGE_RAW = 8000;
    protected static final double BUS_VOLTAGE_RESOLUTION = MAX_BUS_VOLTAGE / MAX_BUS_VOLTAGE_RAW;

    //----------------------------------------------------------------------------------------------
    // Communication
    //----------------------------------------------------------------------------------------------

    protected void writeShort(final Register reg, short value)
    {
        write(reg, TypeConversion.shortToByteArray(value, ByteOrder.BIG_ENDIAN));
    }

    public void write(Register reg, byte[] values)
    {
        this.deviceClient.write(reg.bVal, values);
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2), ByteOrder.BIG_ENDIAN);
    }

    //----------------------------------------------------------------------------------------------
    // I2c Address Config
    //----------------------------------------------------------------------------------------------

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x40);

    @Override
    public void setI2cAddress(I2cAddr newAddress)
    {
        this.deviceClient.setI2cAddress(newAddress);
    }

    @Override
    public I2cAddr getI2cAddress()
    {
        return this.deviceClient.getI2cAddress();
    }

    //----------------------------------------------------------------------------------------------
    // Hardware Device Info
    //----------------------------------------------------------------------------------------------

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName()
    {
        return String.format(Locale.getDefault(), "Adafruit INA219 Current Sensor %s",
                new RobotUsbDevice.FirmwareVersion(0/*this.read8(Register.FIRMWARE_REV)*/));
    }

    //----------------------------------------------------------------------------------------------
    // I2C Setup
    //----------------------------------------------------------------------------------------------

    public INA219(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        // Reset sensor to make sure it's in a good state
        reset();

        // TODO: Make a way for user to change these
        // Set the configuration of the sensor
        int config = BusResolution.RES_12BIT.bVal |
                BusVoltageRange.V32.bVal |
                Gain.GAIN_8_320MV.bVal |
                Mode.SANDBVOLT_CONTINUOUS.bVal |
                ShuntResolution.RES_12BIT_128S_69MS.bVal;
        writeShort(Register.CONFIGURATION, (short) config); // break point!!!

        // TODO
        // Should do something with calibration here so the sensor can return current and power
        // calculations, though it's not necessary, because we can do them ourselves

        return true;
    }

    protected void setOptimalReadWindow()
    {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }
}

