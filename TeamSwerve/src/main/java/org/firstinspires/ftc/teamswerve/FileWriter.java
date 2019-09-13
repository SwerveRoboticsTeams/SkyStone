package org.firstinspires.ftc.teamswerve;

import android.os.Environment;

import java.io.File;
import java.io.PrintWriter;

/*
 * This class is used to write messages and data to a file, typically to analyze data.
 */
public class FileWriter
{
    private PrintWriter outputFile;

    /*
     * Creates a new file to store messages
     * Each file should be given a different name
     *
     * The file will appear on the robot phone in the folder storage/legacy/emulated
     */
    public FileWriter(String filename)
    {
        openPublicFileForWriting(filename);
    }

    /*
     * Use this to add messages to the file
     */
    public void print(String message)
    {
        if (outputFile!=null)
        {
            try
            {
                outputFile.print(message);
                outputFile.flush();
            }
            catch (Exception e)
            {
                // TODO: Something else should be done here
                //telemetry.log.add("Exception writing to file: " + e.toString());
            }
        }
    }

    /*
     * Writes message on a new line of the file
     */
    public void println(String message)
    {
        print("\r\n" + message);
    }

    /*
     * This should be called when the file is no longer needed
     */
    public void closeFile()
    {
        if (outputFile != null)
        {
            try
            {
                outputFile.flush();
                outputFile.close();
            }
            catch (Exception e)
            {
                // TODO: Something else should be done here
                //telemetry.log.add("Exception closing file: " + e.toString());
            }
        }
    }

    /*
     * Actually creates the new file
     */
    private void openPublicFileForWriting(String filename)
    {
        String fullpath = Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + filename;

        try
        {
            File file = new File(fullpath);
            if (!file.exists())
            {
                file.createNewFile();
            }
            outputFile = new PrintWriter(file);
        }
        catch (Exception e)
        {
            // TODO: Something else should be done here
            //telemetry.log.add("Exception opening file: " + e.toString());
        }
    }
}