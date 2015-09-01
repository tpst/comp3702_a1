package comp3702_a1;
import java.util.ArrayList;
import java.util.List;

import problem.ArmConfig;


public class TreeNode {
	
	private TreeNode parent;
	private ArmConfig config;
	
	public TreeNode(Tree tree, TreeNode rent, ArmConfig data) {
		parent = rent;
		this.config = data;
		tree.addNode(this);
		
		}
		
	/*
	 * Return Config of this tree node
	 */
	public ArmConfig getConfig(){
		return config;
		
	}
	
	/*
	 * Return Parent
	 */
	public TreeNode getParent(){
		return parent;
	}

	/*
	 * Return the path from end point to root (or start point) - highest parent
	 * note parent of root is NULL
	 */
	public ArrayList<TreeNode> getPath() {
		
		ArrayList<TreeNode> path = new ArrayList<TreeNode>();
		//Get parent
		TreeNode lastParent = parent;
		path.add(lastParent);
		
		while (lastParent != null) {
			lastParent = lastParent.getParent();
			path.add(lastParent);
			
		}
		
		
		return path;
		
	}

}
