package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Auto {

    private String autoString;
    private boolean isAutoValid;
    public Command autonomousCommand;

    

    public void setupAutoDashboard() {
        //TODO:

    }

    public void setFeedback(String s){
        //TODO:
    }

    /* ArrayLists to hold the values of various waypoints */
    private ArrayList<String> startingLocations = new ArrayList<String>(
        Arrays.asList("S1","S2","S3","S4","S5")
    );
    private ArrayList<String> scoringLocations = new ArrayList<String>(
        Arrays.asList("A","A'","B","B'","C","C'","D","D'","E","E'","F","F'")
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
        for(int i=2; i<autoString.length(); i+=3){

            /* Creates strings to store each part of the 3 character interval */
            String first = autoString.substring(i,i+1);
            String second = i<autoString.length()-1 ? autoString.substring(i+1,i+2) : null;
            String third = i<autoString.length()-2 ? autoString.substring(i+2,i+3) : null;

            /* Checks if we are going to a valid scoring location */
            if(!scoringLocations.contains(first)) {
                isAutoValid = false;
                setFeedback("Character " + (i+1) + " is not a scoring location, when it should be!");
            }
            /* Checks if the coral level we want to score at is valid */
            if(!coralLevels.contains(second) && second != null) {
                isAutoValid = false;
                setFeedback("Character " + (i+2) + " is not a valid coral level to score at, when it should be!");
            }
            /* Checks if the pickup location we want to score at is valid */
            if(!pickupLocations.contains(third) && third != null) {
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
        return Commands.runOnce(() -> {});
    }
    
    public void parseAutoString(String str) {
        for(int i = 0; i < str.length()-1; i++) {
            // if(str.charAt(i) 
        }
    }

}
