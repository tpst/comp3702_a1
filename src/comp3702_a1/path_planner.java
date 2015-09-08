package comp3702_a1;

import java.awt.Graphics2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
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
		
		Tester tests = new Tester();
		tests.ps = problem;
		
		//construct new empty search tree using root node
		
		// do rrt while we have not found goal
		ArrayList<ArmConfig> path = RRTLoop(problem.getInitialState(), problem.getGoalState(), tests);
		
//		//Smooth Path - create shortcuts between nodes if possible
		if(path.size() > 3) {
		path = smoothPath(path, tests);
		}
		
		int size = path.size();
		double jDiff = calcTotJointDiff(path);
		
		
		System.out.println("Completing Path - Interpolation");
		path = interpolatePathFull(tests, path);
//		path = completePath(path, tests);
		  
		problem.setPath(path);
		//Output 
		try {
			problem.saveSolution(args[1]);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		

		
		
		System.out.print("Path size smooth ");
		System.out.println(size);
		System.out.print("Joint diff");
		System.out.println(jDiff);
		
		System.out.println("Challenge Complete");
	}
	
	/**
	 * Rapidly Exploring Random Tree
	 * 
	 * Finds a path between initial and goal state
	 * 
	 * @param problem
	 * 		Problem Spec object containing problem
	 */
	public static ArrayList<ArmConfig> RRTLoop(ArmConfig initState, ArmConfig goalState, Tester tests, Rectangle2D bounds) {
//		boolean completedPath = false; //has a complete path been found
		int numSamples = 10;

		List<Obstacle> obstacles = tests.ps.getObstacles();

		Tree tree = new Tree();

		//initial state base
		Point2D initBase = initState.getBase();
		
		System.out.println("Finding Tree");
		
		
		//Add initial config to tree
		new TreeNode(tree, null, initState);
			
		while(true){

			for(int i = 0; i < numSamples; i++) {
				// get a random coordinate
				ArmConfig sample = getValidSampleCfg(tests, bounds);
//					ArmConfig sample = getValidSampleCfg(tests);

				//nearest Node in Tree to sample
				TreeNode parent = tree.nearestNeighbour(sample);
				
				// Check No Collision 
				if(!tests.pathCollisionAllCfg(parent.getConfig(), sample, obstacles)) {
					//No Collision add to tree
					new TreeNode(tree, parent, sample);						
//					System.out.println("Added New Node");
				}			
			}
			//Try Connect Goal State 
			TreeNode parent = tree.nearestNeighbour(goalState);
			if (!tests.pathCollision(parent.getConfig(), goalState, obstacles)) {
				//Goal State Connected

				//add to tree
				TreeNode goal = new TreeNode(tree, parent, goalState);
	
//				completedPath = true;	
				System.out.println("Goal State Connected");

				//Retrieve path back to Initial Config
				return (ArrayList<ArmConfig>) goal.getPath();	
			}
		}
//		return null;
	}
	
	/**
	 * Rapidly Exploring Random Tree
	 * 
	 * Finds a path between initial and goal state
	 * 
	 * @param problem
	 * 		Problem Spec object containing problem
	 */
	public static ArrayList<ArmConfig> RRTLoop(ArmConfig initState, ArmConfig goalState, Tester tests) {
		boolean completedPath = false; //has a complete path been found
		int numSamples = 50;
		double sqrSideLen = 0.1;
		boolean incSample = true;	//increase sampling bounding box
		Rectangle2D bounds = new Rectangle2D.Double();
		
		List<Obstacle> obstacles = tests.ps.getObstacles();

		Tree tree = new Tree();
		
		
		//initial state base
		Point2D initBase = initState.getBase();
		
		System.out.println("Finding Tree");
		
		
		//Add initial config to tree
		new TreeNode(tree, null, initState);
			
		while(true){
			
			//Try Connect Goal State 
			TreeNode parent = tree.nearestNeighbour(goalState);
			if (!tests.pathCollisionAllCfg(parent.getConfig(), goalState, obstacles)) {
				//Goal State Connected

				//add to tree
				TreeNode goal = new TreeNode(tree, parent, goalState);

				completedPath = true;	
				System.out.println("Goal State Connected");
				
				
//				//save tree to plot in matlab																NOT REQURED TEST ONLY
//				try {
//					tree.getMatlabTree();
//				} catch (IOException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
				
				//Retrieve path back to Initial Config
				return (ArrayList<ArmConfig>) goal.getPath();	
			}
			
//			for(int k = 1; k <= 2/sqrSideLen; k ++) {
//				//increment a bounding box to sample configs
//				if (incSample) {
//					bounds = getBoundingRect(initBase, sqrSideLen*k);
//				} else {
//					//default bounds 
//					bounds = tests.BOUNDS;
//					System.out.println("default bounds");
//				}

				for(int i = 0; i < numSamples; i++) {
					// get a random coordinate
//					ArmConfig sample = getValidSampleCfg(tests, bounds);
					ArmConfig sample = getValidSampleCfg(tests);
	
					//nearest Node in Tree to sample
					parent = tree.nearestNeighbour(sample);
					
					// Check No Collision 
					if(!tests.pathCollision(parent.getConfig(), sample, obstacles)) {
						//No Collision add to tree
						new TreeNode(tree, parent, sample);						
//						System.out.println("Added New Node");
					}			
				}
//			}
//			incSample = false;
		}
	}
	
	
	/**
	 * Return a bounding square that is constrained within workspace
	 * @param center
	 * 		center of square
	 * @param sLength
	 * 		side length
	 * @return
	 * 		square
	 */
	public static Rectangle2D getBoundingRect(Point2D center, double sLength) {
		double cX = center.getX();
		double cY = center.getY();
		
		//corner values
		double x_left = cX - sLength/2;
		double x_right = cX + sLength/2;
		double y_bot = cY - sLength/2;
		double y_top = cY + sLength/2;
		
		//Can't be be outside workspace (0,0) to (1,1) diagonal of rect
		if (x_left < 0) x_left = 0;
		if(x_right > 1) x_right = 1;
		if(y_bot < 0) y_bot =0;
		if(y_top > 1) y_top = 1;
		
		return new Rectangle2D.Double(x_left, y_bot, x_right-x_left, y_top-y_bot);
	
	}
	
	/**
	 * Returns a bounding rect given 2 corner points
	 * @param crn1
	 * 			one corner of the bounding box
	 * @param crn2
	 * 			second corner of bounding box
	 * @param grow
	 * 			grow size for rect
	 * @return
	 * 			bounding box from crn1 to crn2
	 */
	public static Rectangle2D getBoundingRect(Point2D crn1, Point2D crn2, double grow) {
				
		double x1 = crn1.getX();
		double x2 = crn2.getX();
		double y1 = crn1.getY();
		double y2 = crn2.getY();
		
		double w = Math.abs(x1-x2);
		double h = Math.abs(y1-y2);
		
		Rectangle2D rect = new Rectangle2D.Double(Math.min(x1, x2), Math.min(y1, y2), w, h);
		
		return Tester.grow(rect, grow);
	
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
	
	/*
	 * Returns a randomly chosen x,y position between
	 * the rectangle bounds
	 */
	public static Point2D getNewSampleBase(Rectangle2D bounds) {
		
		Double randomx = Math.random()*bounds.getWidth() + bounds.getMinX();
		Double randomy = Math.random()*bounds.getHeight() + bounds.getMinY();
		
		return new Point2D.Double(randomx, randomy);
	}
	
	/**
	 * Return a new sample arm configuration
	 * @param numJoints
	 * 		Number of joints required
	 * @return
	 * 		Randomly Sampled Arm Config
	 */
	public static ArmConfig getSampleConfig(int numJoints) {
		
		Point2D base = getNewSampleBase();
		
		List<Double> joints = new ArrayList<Double>();
		Tester test = new Tester();
		double maxJointAng = test.MAX_JOINT_ANGLE;
		
		for (int i = 0; i < numJoints; i ++) {
			joints.add(Math.random()*2*maxJointAng - maxJointAng);
		}
		
		return new ArmConfig(base, joints);	
	}
	
	/**
	 * Return a new sample arm configuration
	 * @param numJoints
	 * 		Number of joints required
	 * @param bounds
	 * 		Bounding rectangle to choose base point from 
	 * @return
	 * 		Randomly Sampled Arm Config
	 */
	public static ArmConfig getSampleConfig(int numJoints, Rectangle2D bounds) {
		
		Point2D base = getNewSampleBase(bounds);
		
		List<Double> joints = new ArrayList<Double>();
		Tester test = new Tester();
		double maxJointAng = test.MAX_JOINT_ANGLE;
		
		for (int i = 0; i < numJoints; i ++) {
			joints.add(Math.random()*2*maxJointAng - maxJointAng);
		}
		
		return new ArmConfig(base, joints);	
	}

	/**
	 * Return a new sample arm configuration
	 * @param numJoints
	 * 		Number of joints required
	 * @param base
	 * 		base point
	 * @return
	 * 		Randomly Sampled Arm Config
	 */
	public static ArmConfig getSampleConfig(int numJoints, Point2D base) {
		
		List<Double> joints = new ArrayList<Double>();
		Tester test = new Tester();
		double maxJointAng = test.MAX_JOINT_ANGLE;
		
		for (int i = 0; i < numJoints; i ++) {
			joints.add(Math.random()*2*maxJointAng - maxJointAng);
		}
		
		return new ArmConfig(base, joints);	
	}
	
	
	/**
	 * Get a Valid sample config
	 * @param ps
	 * 		problem specifications
	 * @return
	 * 		a valid sample configuration
	 */
	public static ArmConfig getValidSampleCfg(Tester tests) {

		int numJoints = tests.ps.getJointCount();
		
		for(;;) {
			ArmConfig cfg = getSampleConfig(numJoints);
			
			if (tests.isValidConfig(cfg)) {
				return cfg;
//			} else {
//				System.out.println("Invalid Sample");
			}
		}	
	}
	
	/**
	 * Get a Valid sample config
	 * @param ps
	 * 		problem specifications
	 * @param bounds
	 * 		bounding rectangle to choose base point from
	 * @return
	 * 		a valid sample configuration
	 */
	public static ArmConfig getValidSampleCfg(Tester tests, Rectangle2D bounds) {

		int numJoints = tests.ps.getJointCount();
		
		for(;;) {
			ArmConfig cfg = getSampleConfig(numJoints, bounds);
			
			if (tests.isValidConfig(cfg)) {
				return cfg;
//			} else {
//				System.out.println("Invalid Sample");
			}
		}	
	}
	
	/**
	 * Get a Valid sample config
	 * @param tests
	 * 		tester class
	 * @param base
	 * 		Base position of config
	 * @return
	 * 		a valid sample configuration
	 */
	public static ArmConfig getValidSampleCfg(Tester tests, Point2D base) {

		int numJoints = tests.ps.getJointCount();
		
		for(;;) {
			ArmConfig cfg = getSampleConfig(numJoints, base);
			
			if (tests.isValidConfig(cfg)) {
				return cfg;
			} else {
				System.out.println("Invalid Sample");
			}
		}	
	}
	
	
	
	/**
	 * Smooth the given path to remove non essential nodes
	 * @param problem
	 * 
	 */
	public static ArrayList<ArmConfig> smoothPath(ArrayList<ArmConfig> path, Tester tests) {
		
		List<Obstacle> o = tests.ps.getObstacles();
		boolean updating = true;
		
		
		System.out.println("Smoothing Path");
		int origSize = path.size();
		
		while (updating) {
			if(path.size() == 2) {
				break;
			}
			//Start at initial position
			for (int i = 0; i < path.size() - 2; i ++) {
				
				//set updating false
				updating = false;
				
				if(!tests.pathCollisionAllCfg(path.get(i), path.get(i+2), o)){
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
		//Return new path
		return path;
	}
	
	/*
	 * Takes the current solution path and corrects it such that base movement is <= 0.001 per step, 
	 * and max joint rotation is <= 0.1 degrees.
	 */
	public static ArrayList<ArmConfig> interpolatePath(Tester tests, ArrayList<ArmConfig> path) {
		tests.ps.setPath(path);
		List<Integer> badSteps = tests.getInvalidSteps();
				
		int tmp = 0;
		
//		while(badSteps.size()>0) {

			int addIdx = 0;
	
			for (Integer i : badSteps) {
				
				ArmConfig cfg1 = path.get(i + addIdx);
				ArmConfig cfg2 = path.get(i+1 + addIdx);
							
				double dist = cfg1.getBase().distance(cfg2.getBase());
				ArrayList<Double> jointAngleDifferences = calcDeltaJoints(cfg1, cfg2);
				
				//number of steps required
//				int numSteps = getNumSteps(dist, jointAngleDifferences, tests);
				int numSteps = 2;
				
				//step distance
				double dx = (cfg2.getBase().getX() - cfg1.getBase().getX())/numSteps;
				double dy = (cfg2.getBase().getY() - cfg1.getBase().getY())/numSteps;
				ArrayList<Double> jointIncs = calcJointStep (jointAngleDifferences, numSteps);
				
			ArmConfig prevCfg = new ArmConfig(cfg1);	
			int prevSize = path.size();
			
			for (int j = 1; j < numSteps; j ++) {	
			ArmConfig newCfg = new ArmConfig(prevCfg);
			newCfg.addIncrement(dx, dy, jointIncs);
			
			//check cfg is valid
//			if(!tests.isValidConfig(newCfg)) {
////				System.out.println("Config not valid");
//				//not valid get a valid config
//				newCfg = getPathConfig(newCfg.getBase(), prevCfg, tests);
//			}
			
			if(tests.isValidConfig(newCfg)) {
				path.add(addIdx+i+1, newCfg);
				addIdx +=1;
			}
			
				
			prevCfg = newCfg;
		}
		
//		addIdx +=path.size()-prevSize;	//Increase the Idex Counter
	}

			//update badSteps
			tests.ps.setPath(path);
			badSteps = tests.getInvalidSteps();
				
			System.out.print(" interp2 BadSteps: ");
			System.out.println(badSteps.size());
//			System.out.println(badSteps);
			System.out.print("path size ");
			System.out.println(path.size());
//		}	
			return path;	
	}
	
	
	/*
	 * Takes the current solution path and corrects it such that base movement is <= 0.001 per step, 
	 * and max joint rotation is <= 0.1 degrees.
	 */
	public static ArrayList<ArmConfig> interpolatePathFull(Tester tests, ArrayList<ArmConfig> path) {
		tests.ps.setPath(path);
		List<Integer> badSteps = tests.getInvalidSteps();

		while(badSteps.size()>0) {

			int addIdx = 0;
	
			for (Integer i : badSteps) {
				
				ArmConfig cfg1 = path.get(i + addIdx);
				ArmConfig cfg2 = path.get(i+1 + addIdx);
							
				double dist = cfg1.getBase().distance(cfg2.getBase());
				ArrayList<Double> jointAngleDifferences = calcDeltaJoints(cfg1, cfg2);
				
				//number of steps required
//				int numSteps = getNumSteps(dist, jointAngleDifferences, tests);
				int numSteps = 2;
				
				//step distance
				double dx = (cfg2.getBase().getX() - cfg1.getBase().getX())/numSteps;
				double dy = (cfg2.getBase().getY() - cfg1.getBase().getY())/numSteps;
				ArrayList<Double> jointIncs = calcJointStep (jointAngleDifferences, numSteps);
				
			ArmConfig prevCfg = new ArmConfig(cfg1);	
			int prevSize = path.size();
			
			for (int j = 1; j < numSteps; j ++) {	
			ArmConfig newCfg = new ArmConfig(prevCfg);
			newCfg.addIncrement(dx, dy, jointIncs);
			
			//check cfg is valid
			if(!tests.isValidConfig(newCfg)) {
//				System.out.println("Config not valid");
				//not valid get a valid config
				newCfg = getPathConfig(newCfg.getBase(), prevCfg, tests);
			}

				path.add(addIdx+i+1, newCfg);
				addIdx +=1;
		
			prevCfg = newCfg;
		}
	}

			//update badSteps
			tests.ps.setPath(path);
			badSteps = tests.getInvalidSteps();
				
			System.out.print(" interp2 BadSteps: ");
			System.out.println(badSteps.size());
//			System.out.println(badSteps);
			System.out.print("path size ");
			System.out.println(path.size());
		}	
			return path;	
	}
	
	public static ArrayList<ArmConfig> completePath(ArrayList<ArmConfig> path, Tester tests) {
		
		tests.ps.setPath(path);
		
		//interpolate path
		path = interpolatePath(tests, path);
		tests.ps.setPath(path);
		//now will have a few bad steps
		int badStep = tests.getFirstInvalidStep();
	
		while(badStep >= 0) {
			
			
				ArmConfig cfgInit = path.get(badStep);
				ArmConfig cfgGoal = path.get(badStep+1);
				
				
				//perform RRT between two points
					//setBounding for sample points
				
				Rectangle2D bounds = getBoundingRect(cfgInit.getBase(), cfgGoal.getBase(), 0.01);
				ArrayList<ArmConfig> miniPath = RRTLoop(cfgInit, cfgGoal, tests, bounds);
				
				if(miniPath.size() > 3) {
					miniPath = smoothPath(miniPath, tests);
				}
				//interpolate this path
				miniPath = interpolatePathFull(tests, miniPath);
				//repeat until no more bad steps
				
				tests.ps.setPath(miniPath);
				System.out.print("miniPath BadSteps - ");
				System.out.println(tests.getInvalidSteps().size());
	
				//MiniPath now correct add into actual path
				for(int j = 1; j < miniPath.size()-2; j++) {
					path.add(badStep+j, miniPath.get(j));
				}

			tests.ps.setPath(path);
			badStep = tests.getFirstInvalidStep();		
		}
		
		System.out.println("Path complete");

		return path;
		
	}
	
	/**
	 * get the number of steps required to linearly move from one config to another
	 * within the specifications (0.001 for base and 0.1 for angles)
	 * @param baseDist
	 * 			Distance between base points
	 * @param jointDiff
	 * 		 	distance between corresponding joints
	 * @return
	 * 			the required steps (ie max number of steps)
	 */
	public static int getNumSteps(double baseDist, ArrayList<Double> jointDiff, Tester tests) {
		
		int numSteps = (int) Math.ceil(baseDist/tests.MAX_BASE_STEP);
		int tmpNumSteps = 0;
		
		for(double j : jointDiff){
			tmpNumSteps = (int) Math.ceil(Math.abs(j)/tests.MAX_JOINT_STEP);
			if(tmpNumSteps > numSteps) {
				numSteps = tmpNumSteps;
			}
		}
		return numSteps;
	}
	
	/**
	 * returns a Arm configuration which is valid and close to the previous config on 
	 * the path. To be used when a path config is invalid but the base is known  
	 * @param base
	 * 				base point of robot
	 * @return
	 * 			A valid and good configuration at the given base
	 */
	public static ArmConfig getPathConfig(Point2D base, ArmConfig prevCfg, Tester tests) {
		List<ArmConfig> samples = new ArrayList<ArmConfig>();
		int numSamples = 15;
		int numJoints = tests.ps.getJointCount();
		double sqrSideLen = 0.01;
		//growing sample bounds
		int itTilGrow = 5;
		double growDelta = 0.001;
		
		Rectangle2D bounds = getBoundingRect(base, sqrSideLen);
			
		//sample configs - try for the correct base
		for(int i = 0; i < numSamples; i ++) {
			//get valid sample
			
			ArmConfig cfg = getSampleConfig(numJoints, base);
			if (tests.isValidConfig(cfg)) {
				samples.add(cfg);
//			} else {
//				System.out.println("Invalid Sample - fixed base");
			}
		}
			
		int iter = 0;
		while(samples.size() == 0){
			//repeat until have at least one valid config
			//increase bounding box
			if(iter % itTilGrow ==0) {
				bounds = Tester.grow(bounds, growDelta*(iter/itTilGrow));
			}
			ArmConfig cfg = getSampleConfig(numJoints, bounds);
			if (tests.isValidConfig(cfg)) {
				samples.add(cfg);
			}
			iter +=1;
		}
		
		
		if (samples.size() > 1) {
			return chooseBestCfg(prevCfg, samples);
		} else {
			return samples.get(0);
		}
	}

	/**
	 * Chooses best config in relation to previous configuration wrt joint angles
	 * @param prevCfg
	 * 			previous configuration
	 * @param samples
	 * 			list of configurations to choose from
	 * @return
	 * 			the best configuration (lowest average difference between prev and best configs)
	 */
	public static ArmConfig chooseBestCfg(ArmConfig prevCfg, List<ArmConfig> samples) {
		
		double lowAvg = Double.POSITIVE_INFINITY;	//lowest difference average between joint angles
		int lowIdx = 0;		//index of lowest difference average
		List<Double> prevJoints = prevCfg.getJointAngles();
		
		int numJoints = prevJoints.size();
		
		for(int i = 0; i < samples.size(); i ++) {
			double currDiff = 0;
			List<Double> sampleJoints = samples.get(i).getJointAngles();
			for(int j = 0; j < numJoints; j ++) {
				currDiff += Math.abs(prevJoints.get(j) - sampleJoints.get(j));
			}
			
			double currAvg = currDiff/numJoints;
			if(currAvg < lowAvg) {
				lowAvg = currAvg;
				lowIdx = i;
			}
		}
		return samples.get(lowIdx);
	}
	
	/*
	 * Takes two configurations and calculates the difference between joint angles
	 */
	public static ArrayList<Double> calcDeltaJoints(ArmConfig cfg1, ArmConfig cfg2) {
		ArrayList<Double> deltaJoints = new ArrayList<Double>();
		ArrayList<Double> jointAngles1 = (ArrayList<Double>)cfg1.getJointAngles();
		ArrayList<Double> jointAngles2 = (ArrayList<Double>)cfg2.getJointAngles();
	
		for(int i = 0; i < jointAngles1.size(); i++) {
			double angle = jointAngles2.get(i)-jointAngles1.get(i);
			deltaJoints.add(angle);
		}
		
		return deltaJoints;
	}
	/**
	 * Calculate the distance for each joint to step
	 * @param angDist
	 * 		distance between 2 configurations
	 * @param numSteps
	 * 		number of steps between the 2 configurations
	 * @return
	 * 		a list of the joint step sizes
	 */
	public static ArrayList<Double> calcJointStep (ArrayList<Double> angDist, int numSteps) {
		
		ArrayList<Double> angleSteps = new ArrayList<Double>();
		
		for(Double dist : angDist) {
			angleSteps.add(dist/numSteps);
		}
		return angleSteps;
	}
	
	

	public static double calcTotJointDiff(ArrayList<ArmConfig> path) {
		
		double totDiff = 0;
		
		ArmConfig cfg = path.get(0);
		for(int i = 1; i <path.size(); i ++) {
			ArmConfig nextCfg = path.get(i);
			
			ArrayList<Double> diffs = calcDeltaJoints(cfg, nextCfg);
			
			for(Double d : diffs) {
				totDiff+= Math.abs(d);
			}
			
			cfg = nextCfg;
		}
		
		return totDiff;
	}
	
	
//	/**
//	 * Completes the found path by breaking it down into primitive step sizes
//	 * max base movement is 0.001 and max joint angle is 0.1 degrees
//	 */
//	public static void completePath(ProblemSpec problem) {
//		
//		ArrayList<ArmConfig> path = (ArrayList<ArmConfig>) problem.getPath();	
//				
//		Tester tests = new Tester();
//		tests.ps = problem;
//
//		//Get List of Steps which are not correct distance
//		List<Integer> badSteps = tests.getInvalidSteps();								//HOW DOES TEST HAVE CORRECT PROBLEM SPEC??
//		
//		int addIdx = 0;
//		
//		for (Integer i : badSteps) {
//			
//			ArmConfig cfg1 = path.get(i + addIdx);
//			ArmConfig cfg2 = path.get(i + addIdx + 1);
//			
//			//Break edge up into multiple increments
//			double dist = cfg1.getBase().distance(cfg2.getBase());
//			int numSteps = (int) Math.ceil(dist/0.001);	//number of steps required between cfg1 & cfg2
//			
//			//step distance
//			double dx = (cfg2.getBase().getX() - cfg1.getBase().getX())/numSteps;
//			double dy = (cfg2.getBase().getY() - cfg1.getBase().getY())/numSteps;
//			
//					
//			for (int j = 1; j < numSteps; j ++) {	
//				ArmConfig newCfg = new ArmConfig(path.get(addIdx+i+j-1));
//				newCfg.addBaseIncrement(dx, dy);
//				path.add(addIdx+i+j, newCfg);
//			}
//			
//			addIdx +=numSteps-1;	//Increase the Idex Counter
//		}
//		problem.setPath(path);
//	}
}
