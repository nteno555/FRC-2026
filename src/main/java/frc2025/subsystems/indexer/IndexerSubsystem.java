/* 
 * Aaron's Note
 * 
 * The new ubot might be linear indexer like "Jack in the bot" does. 
 * It abandon the original spinning indexer, and no tower.
 * 
 * The new indexer might be one motor.
 * (I don't know if the build team would like to do two motors)
 * 
 * Use the voltage to control the speed of the indexer and 
 * compatitive to the planning bot as much as possible. 
 * 
 * Todo:
 * (done) 1. View the code and delete/add new functions/variable to it
 * (done) 2. Try to make a double motors system to spare.
 * 
 */

package frc2025.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import frc2025.config.SubsystemConstants;


public class IndexerSubsystem extends SubsystemBase {
    //create instance of IndexerIO called io
    //create a constructor of this class that takes in io and sets it to this instance
    @Override
    public void periodic(){
        //update inputs of io instace
    }
    //create new method setSpeed that returns a Command
        //return a lambda setting the speed in the io to the indexerSpeed
    //create new method setVoltage that also returns a Command
        //return a lambda setting the voltage of the io to indexerVoltage
    //make a function getCurrent() returning a double that just returns totalCurrent from io instace
}