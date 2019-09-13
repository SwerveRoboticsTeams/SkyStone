package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteBuffer;
import java.util.Locale;

/**
 * This class implements the AdafruitTSL2561LightSensor interface (i.e., an AdaFruit light sensor module over i2c)
 * https://www.adafruit.com/products/439
 * https://github.com/adafruit/Adafruit_TSL2561
 *
 * Some code borrowed from Adafruit's sample implementation at https://github.com/adafruit/Adafruit_TSL256
 */
@SuppressWarnings("WeakerAccess")
@I2cSensor(name = "Adafruit Light Sensor", description = "Light Sensor from Adafruit", xmlTag = "TSL2561")
public class AdafruitTSL2561LightSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> implements I2cAddrConfig {

    public class Parameters
    {
        //the address at which the sensor resides on the I2C bus.
        //public I2cAddr i2cAddress = I2cAddr.create7bit(I2CADDR.DEFAULT.bVal);

        //default integration time for our implementation
        public INTEGRATION_TIME integrationTime = INTEGRATION_TIME.MS_13;

        //default gain for our implementation
        public GAIN gain = GAIN.GAIN_16;

        public LIGHT_DETECTION_MODE detectionMode = LIGHT_DETECTION_MODE.VISIBLE;

    }

    private Parameters parameters;

    //----------------------------------------------------------------------------------------------
    // Construction and initialization
    //----------------------------------------------------------------------------------------------

    public AdafruitTSL2561LightSensor(I2cDeviceSynch deviceClient)
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
        Parameters params = new Parameters(); //use the defaults

        return initialize(params);
    }

    protected synchronized boolean initialize(Parameters parameters) {
        // Remember the parameters for future use
        this.parameters = parameters;

        boolean armed = this.deviceClient.isArmed();

        byte id = this.getDeviceID();

        if ((id != ADAFRUIT_TSL2561_ID)) {
            throw new IllegalArgumentException(String.format("unexpected chip: expected=%d found=%d", ADAFRUIT_TSL2561_ID, id));
        }

        // Set the integration time and gain
        setIntegrationTimeAndGain(parameters.integrationTime, parameters.gain);

        //wait x milliseconds for first integration to complete
        waitForIntegrationToComplete();

        enable();

        return true;
    }


    protected synchronized void setIntegrationTimeAndGain(INTEGRATION_TIME time, GAIN gain) {
        write8(REGISTER.TIMING, time.byteVal | gain.byteVal);
    }

    protected synchronized void waitForIntegrationToComplete() {
        //wait x milliseconds for integration to complete
        if (parameters.integrationTime == INTEGRATION_TIME.MS_13)
            delay(TSL2561_INTEGRATION_DELAY_13MS);
        else if (parameters.integrationTime == INTEGRATION_TIME.MS_101)
            delay(TSL2561_INTEGRATION_DELAY_101MS);
        else /*if (parameters.integrationTime == INTEGRATION_TIME.MS_402)*/
            delay(TSL2561_INTEGRATION_DELAY_402MS);
    }

    public synchronized byte getDeviceID() {
        byte b = this.read8(REGISTER.ID);
        return b;
    }

    private synchronized void enable() {
        write8(REGISTER.CONTROL, TSL2561_CONTROL_POWERON);
    }

    /* Turn the device off to save power */
    private synchronized void disable() { write8(REGISTER.CONTROL, TSL2561_CONTROL_POWEROFF); }

    public synchronized byte getState() {
        byte b = this.read8(REGISTER.CONTROL);
        return b;
    }

    public synchronized byte getTimingAndGain() {
        byte b = this.read8(REGISTER.TIMING);
        return b;
    }


    //----------------------------------------------------------------------------------------------
    // Light sensor methods
    //----------------------------------------------------------------------------------------------


    /*
     * Get the amount of light detected by the sensor.
     * Returns amount of light in LUX.
     *
     *   From the datasheet, page 23
     *   How to calculate LUX from the sensor light values
         CS Package
            For 0 < CH1/CH0  0.52 Lux = (0.0315 * CH0) − (0.0593 * CH0 * ((CH1/CH0)^1.4))
            For 0.52 < CH1/CH0  0.65 Lux = (0.0229 * CH0) − (0.0291 * CH1)
            For 0.65 < CH1/CH0  0.80 Lux = (0.0157 * CH0) − (0.0180 * CH1)
            For 0.80 < CH1/CH0  1.30 Lux = (0.00338 * CH0) − (0.00260 * CH1)
            For CH1/CH0 > 1.30 Lux = 0

         T, FN, and CL Package
            For 0 < CH1/CH0  0.50 Lux = (0.0304 * CH0) − (0.062 * CH0 * ((CH1/CH0)^1.4))
            For 0.50 < CH1/CH0  0.61 Lux = (0.0224 * CH0) − (0.031 * CH1)
            For 0.61 < CH1/CH0  0.80 Lux = (0.0128 * CH0) − (0.0153 * CH1)
            For 0.80 < CH1/CH0  1.30 Lux = (0.00146 * CH0) − (0.00112 * CH1)
            For CH1/CH0 > 1.30 Lux = 0

            The formulas shown above were obtained by optical testing with fluorescent and incandescent light sources,
            and apply only to open-air applications. Optical apertures (e.g. light pipes) will
            affect the incident light on the device

     */
    /*
    public synchronized double getLUX() {
        double result = 0;

        //add the above math here...
        //open Q: do we need to scale the register values by the integration time? I think so.

        return result;
    }
    */

    /*
     * Get the amount of light detected by the sensor. 1.0 is max possible light, 0.0 is least possible light.
     * Returns amount of light, on a scale of 0 to 1
     */
    public synchronized double getLightDetected() {
        double raw = getLightDetectedRaw(); //get this as a double so the division below will use double math

        double result = 0;

        if (parameters.integrationTime == INTEGRATION_TIME.MS_13)
            result = (raw / TSL2561_MAX_RAW_VALUE_13MS);
        else if (parameters.integrationTime == INTEGRATION_TIME.MS_101)
            result = (raw / TSL2561_MAX_RAW_VALUE_101MS);
        else if (parameters.integrationTime == INTEGRATION_TIME.MS_402)
            result = (raw / TSL2561_MAX_RAW_VALUE_402MS);

        return result;
    }


    //return the raw value of the sensor, considering which light detection mode the user has asked for.
    public synchronized int getLightDetectedRaw() {
        if (parameters.detectionMode == LIGHT_DETECTION_MODE.BROADBAND) {
            return getRawBroadbandLight();
        } else if (parameters.detectionMode == LIGHT_DETECTION_MODE.INFRARED) {
            return getRawIRSpectrumLight();
        } else /* if (parameters.detectionMode == LIGHT_DETECTION_MODE.VISIBLE) */ {
            /* Note: even though "broadband" is supposed to include visible + IR,
             * this calculation can return a negative number.
             * This can happen because there are actually 2 sensors (one capturing broadband,
             * and one capturing IR), and they don't have exactly the same responsiveness to signals.
             * If this value is less than zero, it means that the infrared portion might be the vast majority
             * of the resulting signal. So, I'm clamping the value to be no less than zero.
             */
            int result = getRawBroadbandLight() - getRawIRSpectrumLight();
            if (result < 0) result = 0;
            return result;
        }
    }


    //----------------------------------------------------------------------------------------------
    // Low level sensor methods
    //----------------------------------------------------------------------------------------------

    private synchronized int getRawBroadbandLight() {
        //todo In AdaFruit's implementation, they always enable, read, disable. Should we?
        //No for now.
            //enable();
            //waitForIntegrationToComplete();
            //int returnValue = readUnsignedShort(REGISTER.CHAN0_LOW);
            //disable();
            //return returnValue;
        int low = read8(REGISTER.CHAN0_LOW);
        int high = read8(REGISTER.CHAN0_HIGH);

        return (high << 8) + low;
    }

    private synchronized int getRawIRSpectrumLight() {
        //todo In AdaFruit's implementation, they always enable, read, disable. Should we?
        // No for now.
            //enable();
            //waitForIntegrationToComplete();
            //int returnValue = readUnsignedShort(REGISTER.CHAN0_LOW);
            //disable();
            //return returnValue;
        int low = read8(REGISTER.CHAN1_LOW);
        int high = read8(REGISTER.CHAN1_HIGH);

        return (high << 8) + low;
    }

    //----------------------------------------------------------------------------------------------
    // Communication
    //----------------------------------------------------------------------------------------------


    public synchronized byte read8(final REGISTER reg) {
        //this device likes the COMMAND bit to be set when specifying registers
        return deviceClient.read8(reg.bVal | TSL2561_COMMAND_BIT);
    }

    public synchronized byte[] read(final REGISTER reg, final int cb) {
        //this device likes the COMMAND bit to be set when specifying registers,
        return deviceClient.read(reg.bVal | TSL2561_COMMAND_BIT, cb);
    }

    public synchronized void write8(REGISTER reg, int data) {
        //this device likes the COMMAND bit to be set when specifying registers
        this.deviceClient.write8(reg.bVal | TSL2561_COMMAND_BIT, data, I2cWaitControl.WRITTEN);
    }

    public synchronized void write(REGISTER reg, byte[] data) {
        //this device likes the COMMAND bit to be set when specifying registers,
        this.deviceClient.write(reg.bVal | TSL2561_COMMAND_BIT, data, I2cWaitControl.WRITTEN);
    }

    public synchronized int readUnsignedByte(REGISTER register)
    {
        return TypeConversion.unsignedByteToInt(read8(register));
    }

    public synchronized int readUnsignedShort(REGISTER register)
    {
        return TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(this.deviceClient.read(register.bVal, 2)));
    }

    public synchronized int readSignedShort(REGISTER register)
    {
        return TypeConversion.byteArrayToShort(this.deviceClient.read(register.bVal, 2));
    }

    protected synchronized void waitForWriteCompletions()
    {
        this.deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
    }

    protected synchronized void setOptimalReadWindow()
    {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                REGISTER.FIRST.bVal,
                REGISTER.LAST.bVal - REGISTER.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    //----------------------------------------------------------------------------------------------
    // I2c Address Config
    //----------------------------------------------------------------------------------------------

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(I2CADDR.DEFAULT.bVal);

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
        return String.format(Locale.getDefault(), "Adafruit TSL2561 Light Sensor %s",
                new RobotUsbDevice.FirmwareVersion(0/*this.read8(Register.FIRMWARE_REV)*/));
    }


    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    //wait a number of milliseconds
    protected synchronized void delay(int ms)
    {
        try
        {
            // delays are usually relative to preceding writes, so make sure they're all out to the controller
            this.waitForWriteCompletions();
            Thread.sleep((int)(ms));
        }
        catch (InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
    }

    //----------------------------------------------------------------------------------------------
    // Hardware constants
    //----------------------------------------------------------------------------------------------

    enum REGISTER {
        FIRST (0),
        CONTROL(0x00),
        TIMING(0x01),
        THRESHHOLDL_LOW(0x02),
        THRESHHOLDL_HIGH(0x03),
        THRESHHOLDH_LOW(0x04),
        THRESHHOLDH_HIGH(0x05),
        INTERRUPT(0x06),
        CRC(0x08),
        ID(0x0A),
        CHAN0_LOW(0x0C),
        CHAN0_HIGH(0x0D),
        CHAN1_LOW(0x0E),
        CHAN1_HIGH(0x0F),
        LAST(CHAN1_HIGH.bVal);

        public byte bVal;

        REGISTER(int i) {
            this.bVal = (byte) i;
        }
    }

    //This board/chip allows 3 possible i2c addresses.
    enum I2CADDR {
        UNSPECIFIED(-1), DEFAULT(0x39), ADDR_29(0x29), ADDR_49(0x49);
        public final byte bVal;

        I2CADDR(int i) {
            bVal = (byte) i;
        }
    }

    enum GAIN {
        GAIN_1(0x00),  //no gain
        GAIN_16(0x10); //16x gain

        public final byte byteVal;

        GAIN(int i) {
            this.byteVal = (byte) i;
        }
    }

    enum INTEGRATION_TIME {
        MS_13(0x00), //fast but low resolution
        MS_101(0x01), //medium speed and resolution
        MS_402(0x02); //16-bit data but slowest conversions

        public final byte byteVal;

        INTEGRATION_TIME(int i) {
            this.byteVal = (byte) i;
        }
    }


    enum LIGHT_DETECTION_MODE {
        BROADBAND(0x00), //detect both visible and IR light
        INFRARED(0x01),  //detect only IR light
        VISIBLE(0x02);   //detect only visible light

        public final byte byteVal;

        LIGHT_DETECTION_MODE(int i) {
            this.byteVal = (byte) i;
        }
    }


    //------------------------------------------------------------------------------------------
    // Register descriptions
    //------------------------------------------------------------------------------------------
    /*
     ADDRESS     REGISTER NAME     REGISTER FUNCTION
     −−         COMMAND           Specifies register address
     0h          CONTROL           Control of basic functions
     1h          TIMING            Integration time/gain control
     2h          THRESHLOWLOW      Low byte of low interrupt threshold
     3h          THRESHLOWHIGH     High byte of low interrupt threshold
     4h          THRESHHIGHLOW     Low byte of high interrupt threshold
     5h          THRESHHIGHHIGH    High byte of high interrupt threshold
     6h          INTERRUPT         Interrupt control
     7h          −−               Reserved
     8h          CRC               Factory test — not a user register
     9h          −−               Reserved
     Ah          ID                Part number/ Rev ID
     Bh          −−               Reserved
     Ch          DATA0LOW          Low byte of ADC channel 0
     Dh          DATA0HIGH         High byte of ADC channel 0
     Eh          DATA1LOW          Low byte of ADC channel 1
     Fh          DATA1HIGH         High byte of ADC channel 1

    */
    //----------------------------------------------------------------------------------------------

    //byte TSL2561_VISIBLE = 2;                   // channel 0 - channel 1
    //byte TSL2561_INFRARED = 1;                  // channel 1
    //byte TSL2561_FULLSPECTRUM = 0;              // channel 0

    // I2C address options
    //int TSL2561_ADDR_LOW         = (0x29);
    //int TSL2561_ADDR_FLOAT       = (0x39);    // Default address (pin left floating)
    //int TSL2561_ADDR_HIGH        = (0x49);

    byte ADAFRUIT_TSL2561_ID     = (0x50);  //this doesn't match the datasheet I read, but matches actual device?!

    int TSL2561_MAX_RAW_VALUE_13MS                = (0x000013B7);
    int TSL2561_MAX_RAW_VALUE_101MS                = (0x00009139);
    int TSL2561_MAX_RAW_VALUE_402MS                = (0x0000FFFF);

    int TSL2561_INTEGRATION_DELAY_13MS       =  (15);    // These values come from AdaFruit's implementation
    int TSL2561_INTEGRATION_DELAY_101MS      = (120);    // These values come from AdaFruit's implementation
    int TSL2561_INTEGRATION_DELAY_402MS      = (450);    // These values come from AdaFruit's implementation

    // Lux calculations differ slightly for CS package
    //Package options are:
    //TSL2561_PACKAGE_CS
    //TSL2561_PACKAGE_T_FN_CL //this is the default package in the adafruit library

    int TSL2561_COMMAND_BIT      = (0x80);    // Must be 1
    int TSL2561_CLEAR_BIT        = (0x40);    // Clears any pending interrupt (write 1 to clear)
    int TSL2561_WORD_BIT         = (0x20);    // 1 = read/write word (rather than byte)
    int TSL2561_BLOCK_BIT        = (0x10);    // 1 = using block read/write

    int TSL2561_CONTROL_POWERON  = (0x03);
    int TSL2561_CONTROL_POWEROFF = (0x00);

    int TSL2561_LUX_LUXSCALE     = (14);      // Scale by 2^14
    int TSL2561_LUX_RATIOSCALE   = (9);       // Scale ratio by 2^9
    int TSL2561_LUX_CHSCALE      = (10);      // Scale channel values by 2^10
    int TSL2561_LUX_CHSCALE_TINT0 = (0x7517);  // 322/11 * 2^TSL2561_LUX_CHSCALE
    int TSL2561_LUX_CHSCALE_TINT1 = (0x0FE7);  // 322/81 * 2^TSL2561_LUX_CHSCALE

    // T, FN and CL package values
    int TSL2561_LUX_K1T          = (0x0040);  // 0.125 * 2^RATIO_SCALE
    int TSL2561_LUX_B1T          = (0x01f2);  // 0.0304 * 2^LUX_SCALE
    int TSL2561_LUX_M1T          = (0x01be);  // 0.0272 * 2^LUX_SCALE
    int TSL2561_LUX_K2T          = (0x0080);  // 0.250 * 2^RATIO_SCALE
    int TSL2561_LUX_B2T          = (0x0214);  // 0.0325 * 2^LUX_SCALE
    int TSL2561_LUX_M2T          = (0x02d1);  // 0.0440 * 2^LUX_SCALE
    int TSL2561_LUX_K3T          = (0x00c0);  // 0.375 * 2^RATIO_SCALE
    int TSL2561_LUX_B3T          = (0x023f);  // 0.0351 * 2^LUX_SCALE
    int TSL2561_LUX_M3T          = (0x037b);  // 0.0544 * 2^LUX_SCALE
    int TSL2561_LUX_K4T          = (0x0100);  // 0.50 * 2^RATIO_SCALE
    int TSL2561_LUX_B4T          = (0x0270);  // 0.0381 * 2^LUX_SCALE
    int TSL2561_LUX_M4T          = (0x03fe);  // 0.0624 * 2^LUX_SCALE
    int TSL2561_LUX_K5T          = (0x0138);  // 0.61 * 2^RATIO_SCALE
    int TSL2561_LUX_B5T          = (0x016f);  // 0.0224 * 2^LUX_SCALE
    int TSL2561_LUX_M5T          = (0x01fc);  // 0.0310 * 2^LUX_SCALE
    int TSL2561_LUX_K6T          = (0x019a);  // 0.80 * 2^RATIO_SCALE
    int TSL2561_LUX_B6T          = (0x00d2);  // 0.0128 * 2^LUX_SCALE
    int TSL2561_LUX_M6T          = (0x00fb);  // 0.0153 * 2^LUX_SCALE
    int TSL2561_LUX_K7T          = (0x029a);  // 1.3 * 2^RATIO_SCALE
    int TSL2561_LUX_B7T          = (0x0018);  // 0.00146 * 2^LUX_SCALE
    int TSL2561_LUX_M7T          = (0x0012);  // 0.00112 * 2^LUX_SCALE
    int TSL2561_LUX_K8T          = (0x029a);  // 1.3 * 2^RATIO_SCALE
    int TSL2561_LUX_B8T          = (0x0000);  // 0.000 * 2^LUX_SCALE
    int TSL2561_LUX_M8T          = (0x0000);  // 0.000 * 2^LUX_SCALE

    // CS package values
    int TSL2561_LUX_K1C          = (0x0043);  // 0.130 * 2^RATIO_SCALE
    int TSL2561_LUX_B1C          = (0x0204);  // 0.0315 * 2^LUX_SCALE
    int TSL2561_LUX_M1C          = (0x01ad);  // 0.0262 * 2^LUX_SCALE
    int TSL2561_LUX_K2C          = (0x0085);  // 0.260 * 2^RATIO_SCALE
    int TSL2561_LUX_B2C          = (0x0228);  // 0.0337 * 2^LUX_SCALE
    int TSL2561_LUX_M2C          = (0x02c1);  // 0.0430 * 2^LUX_SCALE
    int TSL2561_LUX_K3C          = (0x00c8);  // 0.390 * 2^RATIO_SCALE
    int TSL2561_LUX_B3C          = (0x0253);  // 0.0363 * 2^LUX_SCALE
    int TSL2561_LUX_M3C          = (0x0363);  // 0.0529 * 2^LUX_SCALE
    int TSL2561_LUX_K4C          = (0x010a);  // 0.520 * 2^RATIO_SCALE
    int TSL2561_LUX_B4C          = (0x0282);  // 0.0392 * 2^LUX_SCALE
    int TSL2561_LUX_M4C          = (0x03df);  // 0.0605 * 2^LUX_SCALE
    int TSL2561_LUX_K5C          = (0x014d);  // 0.65 * 2^RATIO_SCALE
    int TSL2561_LUX_B5C          = (0x0177);  // 0.0229 * 2^LUX_SCALE
    int TSL2561_LUX_M5C          = (0x01dd);  // 0.0291 * 2^LUX_SCALE
    int TSL2561_LUX_K6C          = (0x019a);  // 0.80 * 2^RATIO_SCALE
    int TSL2561_LUX_B6C          = (0x0101);  // 0.0157 * 2^LUX_SCALE
    int TSL2561_LUX_M6C          = (0x0127);  // 0.0180 * 2^LUX_SCALE
    int TSL2561_LUX_K7C          = (0x029a);  // 1.3 * 2^RATIO_SCALE
    int TSL2561_LUX_B7C          = (0x0037);  // 0.00338 * 2^LUX_SCALE
    int TSL2561_LUX_M7C          = (0x002b);  // 0.00260 * 2^LUX_SCALE
    int TSL2561_LUX_K8C          = (0x029a);  // 1.3 * 2^RATIO_SCALE
    int TSL2561_LUX_B8C          = (0x0000);  // 0.000 * 2^LUX_SCALE
    int TSL2561_LUX_M8C          = (0x0000);  // 0.000 * 2^LUX_SCALE

    // Auto-gain thresholds
    int TSL2561_AGC_THI_13MS     = (4850);    // Max value at Ti 13ms = 5047
    int TSL2561_AGC_TLO_13MS     = (100);
    int TSL2561_AGC_THI_101MS    = (36000);   // Max value at Ti 101ms = 37177
    int TSL2561_AGC_TLO_101MS    = (200);
    int TSL2561_AGC_THI_402MS    = (63000);   // Max value at Ti 402ms = 65535
    int TSL2561_AGC_TLO_402MS    = (500);

    // Clipping thresholds
    int TSL2561_CLIPPING_13MS    = (4900);
    int TSL2561_CLIPPING_101MS   = (37000);
    int TSL2561_CLIPPING_402MS   = (65000);


}


