package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

public class Auto {

    private String autoString;
    private boolean isAutoValid;

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

    public void setupAutoDashboard() {
        //TODO:

    }

    public void setFeedback(String s){
        //TODO:
    }

    public void isA(String[] arr){
        for(int i=0; i<arr.length; i++){

        }
    }

    public void validateAutoString() {
        //TODO:

        isAutoValid = true;

        /* Starts at 2 because you don't need to loop through the starting location */
        for(int i=2; i<autoString.length(); i++){

            String firstTwo = autoString.substring(0,2);
            String current = autoString.substring(i,i+1);
            String currentTwo = i<autoString.length()-1 ? autoString.substring(i,i+2) : "";
            String currentThree = i<autoString.length()-2 ? autoString.substring(i,i+3) : "";
            String next = i<autoString.length()-1 ? autoString.substring(i+1,i+2) : "";
            String nextTwo = i<autoString.length()-3 ?autoString.substring(i+2,i+4) : "";

            /*TO CLARIFY ->
             * 
             * If autoString = "hello!" and i=2:
             * first two = "he"
             * 
             * current = "f"
             * next "a"
             * 
             * currentTwo = "fa"
             * nextTwo = "el"
             * 
             * currentThree = "fae"
             */


            /* Checks if there is a valid starting location */
            if(!startingLocations.contains(firstTwo)) {
                isAutoValid = false;
                setFeedback("Invalid starting location");
            }

            







            }
        }
    
    public void buildAutoCommand() {
        //TODO:
    }
    
    public void parseAutoString(String str) {
        for(int i = 0; i < str.length()-1; i++) {
            // if(str.charAt(i) 
        }
    }

}
