import uk.ac.warwick.dcs.maze.logic.IRobot;
import java.util.ArrayList;

public class GrandFinale{

	/*This grandFinale works by storing the movements made in the first run of the maze, overwriting when it revisits
	square. So that means we have an array at the end of the first run with the directions used by the robot to get
	to the exit from each square. So this means we can use this data in order to pick the directions to move in, in
	the second run of the maze. Although this will not produce an opitmal solution it will trim out the loops it has
	to go through while in the first run.*/

	/*Using the same Tremaux implemenation used in exercise 3 in order to solve loopy maze. Full comments in Ex3.*/

	private int pollRun = 0;
	private RobotData robotData;
	private int explorerMode;

	/*Data structure is a 2d array, 200*200 to ensure it will still work even for the maximum dimensions of the maze*/

	private int[][] dirFinalArray = new int[200][200];

	public void controlRobot(IRobot robot){

		/*Control method, takes in robot object and will initalise new robotData if first run, then using the
		explorerMode varible to control the flow between explorer and backtrack.
		If facing towards a wall when initalising the explorer mode will set to 0 after first move so to ensure
		the robot begins by exploring the maze if in the second move (ie. pollRun is 1) then also set explorer
		mode to be 1.*/
	
		if((robot.getRuns() == 0) && (pollRun == 0 || pollRun == 1)){ 
			robotData = new RobotData();
			explorerMode = 1;
		}
		pollRun += 1;

		/*Must switch to the explorerMode 2 when maze has been completed so robot can follow the final direction
		used in the first run. Once in explorerMode 2 it will not need to come out of it.*/

		if(robot.getRuns() > 0){
			explorerMode = 2;
		}

		/*As usual use explorer mode to decide which way to move the robot. If explorerMode = 1 then explore, if 0 then
		backtrack, and if 2 then run the finalRun method to use the array to move the robot through the maze*/

		if(explorerMode == 1){
			exploreControl(robot);
		}
		else if(explorerMode == 0){
			backtrackControl(robot);
		}
		else{
			finalRun(robot);
		}
	}

	public void finalRun(IRobot robot){

		/*finalRun takes in the robot object, and is called when on the second run through the maze. It will set the
		heading by getting the direction the robot finally moved in the first run, and moving the robot ahead.*/
		robot.setHeading(dirFinalArray[robot.getLocation().x][robot.getLocation().y]);
		robot.face(IRobot.AHEAD);
	}

	public void exploreControl(IRobot robot){

		/*This method takes in the robot object, assesses the local area and moves according to Tremaux algorithm
		No return type as move executed from within the method*/ 

		int direction;

		switch(nonwallExits(robot)){
		case 1: direction = deadEnd(robot); explorerMode = 0; break; 	//Reverse and change to backtrack.
		case 2: direction = corrider(robot); break;
		default:
			if(beenbeforeExits(robot) == 1){    //Register new junction and try to move down unexplored if not go down random
				robotData.recordJunction(robot.getLocation().x,robot.getLocation().y,robot.getHeading());
				direction = crossroad(robot);
			}
			else{ 	//At a previous visited exit, then reverse from direction and switch to backtrack
				direction = IRobot.BEHIND;
				explorerMode = 0;
			}
		}
		robot.face(direction);	//Move the robot in the direction returned from decision methods.
		dirFinalArray[robot.getLocation().x][robot.getLocation().y] = robot.getHeading();
	}

	public void backtrackControl(IRobot robot){

		/*Implimentation of algorithm described, takes in robot object and executes the move from within
		the method, so no return type. Will first check if in corrider or deadend and move according to their
		respective methods. If at a junction (i.e 3 non-walls) then if the junction is not fully explored it will
		explore a passage exits and return to explorer mode, or if it is fully explored then reverse out from the
		direction it came in, using the robotData object. If at a crossroad, the robot will choose a passage exit
		if possible and if not reverse out out of the crossroad from the way it came.*/

		/*Set default to AHEAD, because robotData will store absolute directions, and will need to use the set
		Heading function to direct the robot before moving it by using face.(IRobot.AHEAD)*/

		int direction = IRobot.AHEAD;

		switch(nonwallExits(robot)){
			case 1: direction = deadEnd(robot); break;
			case 2: direction = corrider(robot); break;
			default:
			if (passageExits(robot) > 0){		//At a junction with unexplored exits then explore one at random.
				direction = crossroad(robot);
				explorerMode = 1;
			}
			else{		//At a fully explored junction, so reverse out and leave in backtrack. 
				int dirEntered = robotData.searchJunction(robot.getLocation().x,robot.getLocation().y);	
				robot.setHeading(((dirEntered+2)%4) + IRobot.NORTH);	//Using mod arithmetic such as in Ex1 to reverse direction.
			}
		}
		robot.face(direction);
		dirFinalArray[robot.getLocation().x][robot.getLocation().y] = robot.getHeading();
	}

	/*-------------------------------------------------------------------------------------------------------
	These methods are exactly the same as Ex1, and full comments can be found there the methods used, and
	the robotData object which is exactly the same.
	---------------------------------------------------------------------------------------------------------*/

	private int nonwallExits (IRobot robot){

		//For loop to check each different direction, by using that AHEAD+1 = RIGHT

		int countOfExits = 0;
		for(int i = 0; i < 4; i++){
			if (robot.look((IRobot.AHEAD)+i) != IRobot.WALL)
				countOfExits += 1;
		}
		return countOfExits;
	}

	private int deadEnd(IRobot robot){

		//Run a loop, when you find the direction without a wall return this direction.
		//Will not break as in order to call method must be one non-wall space.

		for(int i = 0; i < 4; i++){
			if (robot.look((IRobot.AHEAD)+i) != IRobot.WALL)
				return ((IRobot.AHEAD)+i);
		}
		return -1; //As a default case incase it's enclosed, otherwise error from no return.
	}

	private int corrider(IRobot robot){

		//As we do not want to move behind, start with LEFT, then LEFT + 1 = AHEAD, then
		//RIGHT, as these are the only 3 possible moves, checking each one doesn't hit a wall
		//There will only be one correct move so do not need to move randomly. 

		if (robot.look(IRobot.LEFT) != IRobot.WALL)
			return IRobot.LEFT;
		if (robot.look(IRobot.AHEAD) != IRobot.WALL)
			return IRobot.AHEAD;
		if (robot.look(IRobot.RIGHT) != IRobot.WALL)
			return IRobot.RIGHT;
		return -1;
		
	}

	private int junction(IRobot robot){

		//Junction the same as cross road so just refer to the crossroad method.
		return crossroad(robot);
	
	}

	private int crossroad(IRobot robot){

		/*Takes in robot object, and will return an int representing the direction the robot should move in.
		Creates arrayList to store direction that are passages. Loops through directions to check which are
		passages. If arrayList has elements, then select one at random. If no passages then call random move method*/

		ArrayList<Integer> passageChoices = new ArrayList<Integer>();

		for (int i = 0; i < 4; i++){
			if (robot.look(IRobot.AHEAD + i) == IRobot.PASSAGE)
				passageChoices.add(IRobot.AHEAD + i);
		}

		//Using passage exits as an example of modularisation.

	    if (passageExits(robot) != 0){
			int randomnum = (int) (Math.random()*passageChoices.size());
	        return passageChoices.get(randomnum);
	    }
	    else{
	    	return moveRandomly(robot);
	    }   
	}

	private int moveRandomly(IRobot robot){

		int[] dirArray = {IRobot.AHEAD,IRobot.BEHIND,IRobot.LEFT,IRobot.RIGHT};
		int direction;

		do{
			int randno = (int) (Math.random()*4);   //Can't generate 4.
			direction = dirArray[randno];
		}while(robot.look(direction) == IRobot.WALL);

		//When found a direction that works, loop will break and return it
		return direction;
	}

	private int passageExits(IRobot robot){

		int passages = 0;

		for(int i = 0; i < 4; i++){
			if (robot.look((IRobot.AHEAD)+i) == IRobot.PASSAGE)
				passages += 1;
		}
		return passages;
	}

	private int beenbeforeExits(IRobot robot){

		int beenbefore = 0;

		for(int i = 0; i < 4; i++){
			if (robot.look((IRobot.AHEAD)+i) == IRobot.BEENBEFORE)
				beenbefore += 1;
		}
		return beenbefore; 
	}

	public void reset(){
		robotData.resetJunctionCounter();
		explorerMode = 1;
	}

}

//No public modifier as only one public modifier per java file, same name
//As the java file. 

class RobotData{

	/*RobotData class to store data about the junctions, no public modifier as only one public class per java file.
	Creates an array of JunctionRecorder objects of size 10000, to be used to store data on all junctions in maze.*/

	private static int maxJunctions = 40000;
	private static int junctionCounter;
	private JunctionRecorder[] junctions = new JunctionRecorder[maxJunctions];

	public void resetJunctionCounter(){
		junctionCounter = 0;
	}

	public void recordJunction(int x, int y, int heading){

		/*Takes in 3 varibles in order to record junction and no return type. Method will create JunctionRecorder 
		object with information given and assign the object to the index of the array which we are curretly at
	 	before printing it out and incrementing counter.*/

		JunctionRecorder J = new JunctionRecorder(x,y,heading);
		junctions[junctionCounter] = J;

		printJunction();
		junctionCounter++;
	}

	public void printJunction(){

		//Switch statement to covert int values of IRobot.East etc to 'EAST' to be outputted.

		String OutDir = "";

		switch(junctions[junctionCounter].arrived){
			case IRobot.NORTH: OutDir = "NORTH"; break;
			case IRobot.EAST: OutDir = "EAST"; break; 
			case IRobot.SOUTH: OutDir = "SOUTH"; break; 
			case IRobot.WEST: OutDir = "WEST"; break;
		}

		System.out.println("Junction " + (junctionCounter+1) + " (x=" + junctions[junctionCounter].juncX + ",y=" +
			junctions[junctionCounter].juncY + ") heading " + OutDir);
	}

	public int searchJunction(int x, int y){

		/*Takes in an x and y co-ordinate and searches through the array of Junction objects, till the x and y 
		given match those of a junction. Then returns the direction that robot arrived at that junction*/

		for(int i = 0; i < maxJunctions; i++){
			if(x == junctions[i].juncX && y == junctions[i].juncY)
				return junctions[i].arrived;
		}
		//If not been to the junction before then return -1.
		return 0; 
	}	
}

class JunctionRecorder{

	/*Class created to treat a junction as an object, with 3 varibles to store the x, y and direction which the robot
	entered the junction from, only method will construct a instance of the class assigning each of the instance 
	varibles with parameters given*/

	public int juncX;
	public int juncY; 
	public int arrived;

	public JunctionRecorder(int x, int y, int dirArrived){
		juncX = x;
		juncY = y;
		arrived = dirArrived;
	}
}










