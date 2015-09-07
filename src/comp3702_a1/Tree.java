package comp3702_a1;
import java.awt.geom.Point2D;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import problem.ArmConfig;


public class Tree {
	
	List<TreeNode> tree;
	
	public Tree(){
		tree = new ArrayList<TreeNode>();
		
	}
	
	/**
	 * Adds a tree Node to the tree
	 * 
	 * @param node 
	 * 		node to add to tree
	 */
	public void addNode(TreeNode node) {
		tree.add(node);
	}
	
	/*
	 * Return nearest neighbour node currently in tree to the given node
	 */
	public TreeNode nearestNeighbour(ArmConfig cfg) {
		
		Point2D point = cfg.getBase();
		double nbDistSq, distSq;
		TreeNode checkNode, nbNode = null;
		
		
		nbDistSq = Double.POSITIVE_INFINITY;
		
		//Loop through Tree
		for(int i =0; i < tree.size(); i++) {
			
			//Find distance to checknode
			checkNode = tree.get(i);
			
			distSq = point.distanceSq(checkNode.getConfig().getBase());
			
			if (distSq < nbDistSq) {
				//Closer neighbour 
				nbNode = checkNode;
				nbDistSq = distSq;
			}
			
		}

		//return that neighbour
		return nbNode;
		
	}
	/**
	 * print out tree and parent data to plot tree in matlab
	 * @throws IOException 
	 */
	public void getMatlabTree() throws IOException {
		String filename = "/home/ryan/Documents/COMP3702/outFiles/sol-MATLAB.txt";
		
		String ls = System.getProperty("line.separator");
		FileWriter output = new FileWriter(filename);
		
		for (int i = 1; i < tree.size(); i ++) {
			//Node coords
			String x = String.valueOf(tree.get(i).getConfig().getBase().getX());
			String y = String.valueOf(tree.get(i).getConfig().getBase().getY());		
			
			//parent coords
			String px = String.valueOf(tree.get(i).getParent().getConfig().getBase().getX());
			String py = String.valueOf(tree.get(i).getParent().getConfig().getBase().getY());
			
			output.write(x + " " + y + " " + px + " "+ py + " " + ls);


		}
		output.close();
		}


}
