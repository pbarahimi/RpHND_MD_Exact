package model;

import java.util.ArrayList;

public class RoutingTreeNode {
	public final int ind;
	public RoutingTree routingTree;
	public final ArrayList<Route> availableRoutes;
	public boolean prune = false;
	
	public RoutingTreeNode (int ind, RoutingTree rt, ArrayList<Route> availableRoutes, int L){
		this.ind = ind;
		this.routingTree = rt;
		this.availableRoutes = availableRoutes;
		
		if (routingTree.complete)
			prune = true;
	}
}
