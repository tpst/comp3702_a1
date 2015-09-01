package comp3702_a1;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;


public class Tree {
	
	List<TreeNode> tree;
	
	public Tree(){
		tree = new ArrayList<TreeNode>();
		
	}
	
	public void addNode(TreeNode node) {
		tree.add(node);
	}
	
	/*
	 * Return nearest neighbour node currently in tree to the given node
	 */
	public TreeNode nearestNeighbour(TreeNode node) {
		
		Point2D point = node.getConfig().getBase();
		double nbDistSq, distSq;
		TreeNode checkNode, nbNode;
		
		
		nbDistSq = Double.POSITIVE_INFINITY;
		
		//Loop through Tree
		for(int i =0; i < tree.size(); i++) {
			
			//Find distance to checknode
			checkNode = tree.get(i);
			
			distSq = point.distanceSq(checkNode.getConfig().getBase());
			
			if (distSq < nbDistSq) {
				//Closer neighbour 
				nbNode = checkNode;
			}
			
		}
		

		
		//return that neighbour
		return nbNode;
		
	}

}
