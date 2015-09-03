package comp3702_a1;

import java.awt.Graphics2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import problem.*;
import tester.Tester;

public class path_planner {
	
	
	private static ProblemSpec problem;
	
	public static void main(String[] args) {
		
		
		ProblemSpec problem = new ProblemSpec();
		
		
		try {
			problem.loadProblem(args[0]);
		} catch (Exception e) {
			System.out.println("Error loading problem file");
			e.printStackTrace();
		}
		
		System.out.println(problem.getObstacles());
		//construct new empty search tree using root node
		
		// do rrt while we have not found goal
		RRTLoop(problem);
		
		//Smooth Path - create shorcuts between nodes if possible
//		smoothPath(problem);
		
		//Now have Path from initial to Goal 
		// Need to Break Path down to appropriate step sizes
//		completePath(problem);
		
		
		//Output 
		try {
			problem.saveSolution(args[1]);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		smoothPath(problem);
		
		try {
			problem.saveSolution("/home/ryan/Documents/COMP3702/outFiles/sol-0_joints_obs_smooth.txt");
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		
	}
	
	public static void RRTLoop(ProblemSpec problem) {
		boolean completedPath = false; //has a complete path been found
		int numSamples = 10;
		List<Obstacle> obstacles = problem.getObstacles();
		
		Tester tests = new Tester();
		Tree tree = new Tree();
		
		//Add initial config to tree
		new TreeNode(tree, null, problem.getInitialState());
		
		//For Drawing the  tree
		Graphics2D graphics = null;
		
		List<Double> jointAngles = new ArrayList<Double>();	//starting jointAngled -------- currently no joints
		
		while(!completedPath){
			
			//Try Connect Goal State 
			TreeNode parent = tree.nearestNeighbour(problem.getGoalState());
			if (!tests.pathCollision(parent.getConfig(), problem.getGoalState(), obstacles)) {
				//Goal State Connected
				
				
				//add to tree
				TreeNode goal = new TreeNode(tree, parent, problem.getGoalState());
				
				//Retrieve path back to Initial Config
				problem.setPath(goal.getPath());
				
				completedPath = true;	
				System.out.println("Goal State Connected");
			}
			
			for(int i = 0; i < numSamples; i++) {
				// get a random coordinate
				ArmConfig sample = new ArmConfig(getNewSampleBase(), jointAngles);
						

				// check if the sample lies within an obstacle							
				if(!tests.hasCollision(sample, obstacles)) {
						
					System.out.println(sample.getBase());
					
					//nearest Node in Tree to sample
					parent = tree.nearestNeighbour(sample);
					
					// Check No Collision 
					if(!tests.pathCollision(parent.getConfig(), sample, obstacles)) {
						//No Collision add to tree
						new TreeNode(tree, parent, sample);
						
						//Draw on tree																VISUALISATION					
						Line2D line = new Line2D.Double(parent.getConfig().getBase(), sample.getBase());
//						graphics.draw(line);
						
						System.out.println("Added New Node");
					}			
				} else {
					System.out.println("Collision Detected");
				}
			}
		}
	}
	
	/*
	 * Returns a randomly chosen x,y position between
	 * 0 and 1. 
	 */
	public static Point2D getNewSampleBase() {
		Double randomx = Math.random();
		Double randomy = Math.random();
		
		return new Point2D.Double(randomx, randomy);
	}
	
	/**
	 * Smooth the given path to remove non essential nodes
	 * @param problem
	 * 
	 */
	public static void smoothPath(ProblemSpec problem) {
		
		ArrayList<ArmConfig> path = (ArrayList<ArmConfig>) problem.getPath();
		List<Obstacle> o = problem.getObstacles();
		boolean updating = true;
		Tester tests = new Tester();
		
		System.out.println("Smoothing Path");
		int origSize = path.size();
		
		while (updating) {
			//Start at initial position
			for (int i = 0; i < path.size() - 2; i ++) {
				
				//set updating false
				updating = false;
				
				if(!tests.pathCollision(path.get(i), path.get(i+2), o)){
					//no collision
					//remove middle node
					path.remove(i+1);
					
					//reduce i so we attempt to increase shortcut
					i --;
					
					//will keep on updating until it stops smoothing
					updating = true;
				}
			}
		}
		//Update solution
		problem.setPath(path);
		
		
		System.out.print("Removed ");
		System.out.print(origSize-path.size());
		System.out.print(" Nodes from path\n");
		
	}
	
	/**
	 * Completes the found path by breaking it down into primitive step sizes
	 * max base movement is 0.001 and max joint angle is 0.1 degrees
	 */
	public static void completePath(ProblemSpec problem) {
		
		ArrayList<ArmConfig> path = (ArrayList<ArmConfig>) problem.getPath();	
				
		Tester tests = new Tester();
		tests.ps = problem;

		//Get List of Steps which are not correct distance
		List<Integer> badSteps = tests.getInvalidSteps();								//HOW DOES TEST HAVE CORRECT PROBLEM SPEC??
		
		int addIdx = 0;
		
		for (Integer i : badSteps) {
			
			ArmConfig cfg1 = path.get(i + addIdx);
			ArmConfig cfg2 = path.get(i + addIdx + 1);
			
			//Break edge up into multiple increments
			double dist = cfg1.getBase().distance(cfg2.getBase());
			int numSteps = (int) Math.ceil(dist/0.001);	//number of steps required between cfg1 & cfg2
			
			//step distance
			double dx = (cfg2.getBase().getX() - cfg1.getBase().getX())/numSteps;
			double dy = (cfg2.getBase().getY() - cfg1.getBase().getY())/numSteps;
			
					
			for (int j = 1; j < numSteps; j ++) {	
				ArmConfig newCfg = new ArmConfig(path.get(addIdx+i+j-1));
				newCfg.addBaseIncrement(dx, dy);
				path.add(addIdx+i+j, newCfg);
			}
			
			addIdx +=numSteps-1;	//Increase the Idex Counter

		}
		problem.setPath(path);
	}
	
	
}
