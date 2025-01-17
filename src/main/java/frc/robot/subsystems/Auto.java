package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.CoralScoringLevel;

public class Auto {

    private String autoString;
    private boolean isAutoValid;
    public SequentialCommandGroup autonomousCommand;

    

    public void setupAutoDashboard() {
        //TODO:

    }

    public void setFeedback(String s){
        //TODO:
    }

    public Command pathCommand(String pathString){
        return Commands.runOnce(() -> {});
    }

    /* ArrayLists to hold the values of various waypoints */
    private ArrayList<String> startingLocations = new ArrayList<String>(
        Arrays.asList("S1","S2","S3","S4","S5")
    );
    private ArrayList<String> scoringLocations = new ArrayList<String>(
        Arrays.asList("A","a","B","b","C","c","D","d","E","e","F","f")
    );
    private ArrayList<String> pickupLocations = new ArrayList<String>(
        Arrays.asList("1","2","3","4","5","6")
    );
    private ArrayList<String> coralLevels = new ArrayList<String>(
        Arrays.asList("1","2","3","4")
    );

    public boolean isAutoStringValid() {

        /* Boolean to store if auto is valid or not */
        isAutoValid = true;

        /* Checks if there is a valid starting location */
        if(!startingLocations.contains(autoString.substring(0,2))) {
            isAutoValid = false;
            setFeedback("Invalid starting location");
        }


        /* Starts at 2 because you don't need to loop through the starting location */
        for(int i=2; i<autoString.length(); i+=3){ //Loops every 3 

            /* Creates strings to store each part of the 3 character interval */
            String first = autoString.substring(i, i + 1);
            String second = i<autoString.length()-1 ? autoString.substring(i+1,i+2) : null;
            String third = i<autoString.length()-2 ? autoString.substring(i+2,i+3) : null;

            /* Checks if we are going to a valid scoring location */
            if(!scoringLocations.contains(first)) {
                isAutoValid = false;
                setFeedback("Character " + (i+1) + " is not a scoring location, when it should be!");
            }
            /* Checks if the coral level we want to score at is valid */
            if(second != null && !coralLevels.contains(second)) {
                isAutoValid = false;
                setFeedback("Character " + (i+2) + " is not a valid coral level to score at, when it should be!");
            }
            /* Checks if the pickup location we want to score at is valid */
            if(third != null && !pickupLocations.contains(third)) {
                isAutoValid = false;
                setFeedback("Character " + (i+3) + " is not a valid pickup location, when it should be!");
            }
            
        }

        return isAutoValid;  

    }
        
    
    public Command getAutoCommand() {
        if(!isAutoStringValid()) {
            return Commands.runOnce(() -> {});
            //Possibly make this return just a simple path so we get the leave bonus
        }

        //TODO: write the rest of this
        for(int i=1; i<autoString.length(); i+=3) {

            /* Creates strings to store each part of the 3 character interval */
            String first = autoString.substring(i,i+1); //pickup location
            String second = i<autoString.length()-1 ? autoString.substring(i+1,i+2) : null; //scoring location
            String third = i<autoString.length()-2 ? autoString.substring(i+2,i+3) : null; //coral level
            String fourth = i<autoString.length()-3? autoString.substring(i+3,i+4) : null; //next pickup location
            
            Command pathcmd;
            String pathName;

            /* adds command to from pickup location to scoring location */
            try {
                // Load the path you want to follow using its name in the GUI
                pathName = i == 1? "S" + first + "-" + second : first + "-" + second; //makes sure it accounts for starting path having an "S"
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                // Create a path following command using AutoBuilder. This will also trigger event markers.
                pathcmd = AutoBuilder.followPath(path);
            } catch (Exception e) {
                DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                pathcmd = Commands.none();
            }
            autonomousCommand.addCommands(pathcmd);

            /* ADDS SCORING COMMAND */
            autonomousCommand.addCommands(Commands.runOnce(() -> {})); //change this to score whatever level you want to score

            /* adds command form scoring location to next pickup location */
            try {
                // Load the path you want to follow using its name in the GUI
                pathName = second + "-" + fourth;
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                // Create a path following command using AutoBuilder. This will also trigger event markers.
                pathcmd = AutoBuilder.followPath(path);
            } catch (Exception e) {
                DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                pathcmd = Commands.none();
            }
            autonomousCommand.addCommands(pathcmd);

            /* ADDS AN INTAKING COMMAND */
            autonomousCommand.addCommands(Commands.runOnce(() -> {})); //change this to intaking command

        }

        return autonomousCommand;
    }
    

}
