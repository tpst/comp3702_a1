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
	
	
	private ProblemSpec problem;
	
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
		
		//Now have Path from initial to Goal 
		// Need to Break Path down to appropriate step sizes
		completePath(problem);
		
		
		//Output 
		try {
			problem.saveSolution(args[1]);
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
		
		
		while(!completedPath){
			
			for(int i = 0; i < numSamples; i++) {
				// get a random coordinate
				ArmConfig sample = new ArmConfig(getNewSampleBase());

				// check if the sample lies within an obstacle							
				if(!tests.hasCollision(sample, obstacles)) {
						
					System.out.println(sample.getBase());
					
					//nearest Node in Tree to sample
					TreeNode parent = tree.nearestNeighbour(sample);
					
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
	 * Completes the found path by breaking it down into primative step sizes
	 * max base movement is 0.001 and max joint angle is 0.1 degrees
	 */
	public static void completePath(ProblemSpec problem) {
		
		List<ArmConfig> path = problem.getPath();
				
		Tester tests = new Tester();

		//Get List of Steps which are not correct distance
		List<Integer> badSteps = tests.getInvalidSteps();								//HOW DOES TEST HAVE CORRECT PROBLEM SPEC??
		
		int addIdx = 0;
		
		for (Integer i : badSteps) {
			
			ArmConfig cfg1 = path.get(i + addIdx);
			ArmConfig cfg2 = path.get(i + addIdx + 1);
			
			//Break edge up into multiple incriments
			double dist = cfg1.getBase().distance(cfg2.getBase());
			int numSteps = (int) Math.ceil(dist/0.001);	//number of steps required between cfg1 & cfg2
			
			//step distance
			double dx = (cfg2.getBase().getX() - cfg1.getBase().getX())/numSteps;
			double dy = (cfg2.getBase().getY() - cfg1.getBase().getY())/numSteps;
			
			ArmConfig newCfg = cfg1;
			
			for (int j = 1; j <= numSteps; j ++) {
				newCfg = newCfg.addBaseIncrement(dx, dy);
				
				path.add(addIdx +i+ j, newCfg);
			}
			
			addIdx +=numSteps;	//Increase the Idex Counter

		}
		problem.setPath(path);
	}
}
