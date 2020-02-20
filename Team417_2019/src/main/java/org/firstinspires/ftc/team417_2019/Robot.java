package org.firstinspires.ftc.team417_2019;

public class Robot {

    double curAngle = 0;
    double currentX = 0;
    double currentY = 0;
    private double initialHeading;

    MasterOpMode master;

    // instantiated masterOpMode to resolve static issues because we are making a new robot object
    public Robot(MasterOpMode masterOpMode){
        master = masterOpMode;
    }

    // correct our initial angle depending on where we are starting by the field
    public double getCorrectedHeading(){
        return initialHeading + master.imu.getAngularOrientation().firstAngle;
    }
    public void setCorrectedHeading(double initialHeading) {
        this.initialHeading = initialHeading;
    }

    public void updatePosition() {
        // divide by 4 because we are averaging them (scaling)
        // plug in motor values to check the equations below
        currentX = 0.85 * ( (float) (master.motorFL.getCurrentPosition() - master.motorBL.getCurrentPosition() - master.motorFR.getCurrentPosition() + master.motorBR.getCurrentPosition()) ) / ( 4 * master.COUNTS_PER_INCH );
        currentY = ( (float) (master.motorFL.getCurrentPosition() + master.motorBL.getCurrentPosition() + master.motorFR.getCurrentPosition() + master.motorBR.getCurrentPosition()) ) / ( 4 * master.COUNTS_PER_INCH );

        curAngle = getCorrectedHeading();
    }

}
