package comp3702_a1;

import java.awt.Graphics2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import problem.*;
import tester.Tester;

public class path_planner {
	
	
	ProblemSpec problem;
	
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
		
		//Output 
		/*
		try {
			problem.saveSolution(args[1]);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		*/
	}
	
	public static void RRTLoop(ProblemSpec problem) {
		boolean completedPath = false; //has a complete path been found
		int numSamples = 100;
		List<Obstacle> obstacles = problem.getObstacles();
		
		Tester tests = new Tester();
		Tree tree = new Tree();
		
		//Add initial config to tree
		new TreeNode(tree, null, problem.getInitialState());
		
		//For Drawing the  tree
		Graphics2D graphics = null;
		
		for(int i = 0; i < numSamples; i++) {
			// get a random coordinate
			ArmConfig sample = new ArmConfig(getNewSampleBase(), null);
			
			// check if the sample lies within an obstacle							
			if(!tests.hasCollision(sample, obstacles)) {
					
				System.out.println(sample.getBase());
				
				//nearest Node in Tree to sample
				TreeNode parent = tree.nearestNeighbour(sample);
				
				// Check No Collision 
				if(tests.pathCollision(parent.getConfig(), sample, obstacles)) {
					//No Collision add to tree
					new TreeNode(tree, parent, sample);
					
					//Draw on tree																VISUALISATION
					
					Line2D line = new Line2D.Double(parent.getConfig().getBase(), sample.getBase());
					graphics.draw(line);
					
					System.out.println("Added New Node");
				}			
			}
			System.out.println("Collision Detected");
			}
					
			
			

			//draw line
			
			
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
}
