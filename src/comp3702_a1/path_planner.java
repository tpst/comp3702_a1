package comp3702_a1;

import java.awt.geom.Point2D;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import problem.*;

public class path_planner {
	
	Tree tree;
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
		boolean complete = false; //has a complete path been found
		int numSamples = 100;
		List<Obstacle> obstacles = problem.getObstacles();
		
		for(int i = 0; i < numSamples; i++) {
			// get a random coordinate
			Point2D sample = getNewSample(); 
			
			// check if the sample lies within an obstacle
			for(Obstacle o : obstacles) {
				if(o.getRect().contains(sample)) {
					System.out.println("Collision Detected");
				}
			}
			
			System.out.println(sample);
			//nearestPointInTree(sample)
			//draw line
			
			
		}
		
		
		
		
	}
	
	/*
	 * Returns a randomly chosen x,y position between
	 * 0 and 1. 
	 */
	public static Point2D getNewSample() {
		Double randomx = Math.random() * (1-0);
		Double randomy = Math.random() * (1-0);
		
		return new Point2D.Double(randomx, randomy);
	}
	
	public void nearestPointInTree() {
		// finds nearest point in the search tree
	}
	
	public void checkForCollision() {
		// checks for obstacles between two nodes
	}
}
