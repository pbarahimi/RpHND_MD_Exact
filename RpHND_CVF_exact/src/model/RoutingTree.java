package model;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.PriorityQueue;

public class RoutingTree {
	public Route[] routes ;
	public HashMap<Integer,ArrayList<Node>> usedHubs = new HashMap<Integer, ArrayList<Node>>();
	public HashMap<Integer,ArrayList<Route>> availableRoutes = new HashMap<Integer, ArrayList<Route>>();
	public ArrayList<Integer> unexploredNodes = new ArrayList<Integer>(); 
	public double value;
	public boolean pruned = false;  // if the value of the tree is worse than the upper bound, the tree will be pruned eventhough it might not be complete.
	
	// Constructor 0
	public RoutingTree(int size){
		this.routes = new Route[size];
	}
	
	// Constructor 1
	@SuppressWarnings("unchecked")
	public RoutingTree (Route[] routes, Route newRoute, ArrayList<Route> avlRoutes, int L, double UB) {
		this.routes = routes.clone();
		this.unexploredNodes.add(0);
		this.availableRoutes.put(0, (ArrayList<Route>) avlRoutes.clone() );
		this.usedHubs.put( 0, new ArrayList<Node>() );
		addRoute(newRoute, L);
		updateValue();
		prune(UB);
	};
	
	// Constructor 2
	public RoutingTree (RoutingTree rt, int newNodeInd, Route newRoute, ArrayList<Route> avlRoutes, int L, double UB){
		this.routes = rt.routes.clone();
		this.usedHubs.putAll(rt.usedHubs);
		this.availableRoutes.putAll(rt.availableRoutes);
		this.unexploredNodes.addAll(rt.unexploredNodes);	
		this.availableRoutes.put(newNodeInd, new ArrayList<Route>(avlRoutes) );
		updateUsedHubs(newNodeInd);
		addRoute(newRoute, L);
		updateValue();
		prune(UB);
			
	}
	
	
	private <T> void prune(T upperBound){
		if ((double) upperBound < this.value)
			this.pruned = true;
	}
	
	private void updateUsedHubs(int newNodeInd){
		// Adding parent node's used hubs list to the new node.
		int parentNodeInd = (int) Math.floor((newNodeInd - 1)/2);
		this.usedHubs.put(newNodeInd, new ArrayList<Node>(this.usedHubs.get(parentNodeInd)) );
		
		if (newNodeInd % 2 == 0) // if new node is a right child
			// add the second hub of the parent node
			this.usedHubs.get(newNodeInd).add(this.routes[parentNodeInd].m);
		else // if new node is a left node
			// add the first hub of the parent node
			this.usedHubs.get(newNodeInd).add(this.routes[parentNodeInd].k);
	}
	
	private void addRoute(Route r, int L){
		int index = unexploredNodes.get(0);
		unexploredNodes.remove(0);
		availableRoutes.get(index).remove(r);
		this.routes[index] = r;
				
		// See the index to find the left and right child indices and add to the unexplored list. 		
		// check whether the new route needs backups
		if (index < Math.pow(2, L)-1){  // checks make sure we are not exceeding depth L
			if ( !r.k.equals(r.m) ) {
				if ( r.i.equals(r.k) && !r.j.equals(r.m)) //iimj
					this.unexploredNodes.add(2*index+2);
				else if ( !r.i.equals(r.k) && r.j.equals(r.m) )  //ikjj
					this.unexploredNodes.add(2*index+1);
				else if ( !r.i.equals(r.k) && !r.j.equals(r.m) ){ //ikmj
					this.unexploredNodes.add(2*index+1);
					this.unexploredNodes.add(2*index+2);
				}
				else {} //iijj
				
			} else {
				if ( !r.i.equals(r.k) && !r.j.equals(r.m) ) //ikkj
					this.unexploredNodes.add(2*index+1);
				else {} //iiij or ijjj
			}
		}
	}
	
	public RoutingTree (RoutingTree other) {
		this.routes = other.routes;
		this.usedHubs = other.usedHubs;
		this.value = other.value;
		this.availableRoutes = other.availableRoutes;
//		this.complete = other.complete;
	};
	
	public boolean isComplete(){
		if (this.unexploredNodes.size() == 0)
			return true;
		else
			return false;
	}
	public void updateValue(){	
		this.value = this.routes[0].expCost;
		for (int i = 1 ; i < routes.length ; i++){
			if (this.routes[i] != null)
				this.value += this.routes[i].cost * getFailureProb(i);
		}	
	}
	
	private double getFailureProb(int i ){
		double output = 1;
		for ( Node n : this.usedHubs.get(i) )
			output *= n.failure;
		return output;
	}
	
	public static ArrayList<Route> getFeasibleRoutes(
			int i, 
			int j, 
			int[] hList,
			Node[] nodes,
			Route[][][][] routes,
			double[][] distances,
			double alpha
			){
		// generating list of feasible routes between the origin and the
				// destination.
				ArrayList<Route> feasibleRoutes = new ArrayList<Route>();
				if (nodes[i].isHub && nodes[j].isHub) {
					if (routes[i][i][j][j] == null)
						routes[i][i][j][j] = new Route(nodes[i], nodes[i], nodes[j], nodes[j],
								distances, alpha);
					feasibleRoutes.add(routes[i][i][j][j]);
				} else if (nodes[i].isHub) {
					for (Integer n : hList) {
						if (routes[i][i][n][j] == null)
							routes[i][i][n][j] = new Route(nodes[i], nodes[i], nodes[n], nodes[j],
									distances, alpha);
						feasibleRoutes.add(routes[i][i][n][j]);
					}
				} else if (nodes[j].isHub) {
					for (Integer n : hList) {
						if (routes[i][n][j][j] == null)
							routes[i][n][j][j] = new Route(nodes[i], nodes[n], nodes[j], nodes[j],
									distances, alpha);
						feasibleRoutes.add(routes[i][n][j][j]);
					}
				} else {
					for (int u = 0; u < hList.length; u++) {
						for (int v = u; v < hList.length; v++) {
							if (routes[i][hList[u]][hList[v]][j] == null
									&& routes[i][hList[v]][hList[u]][j] == null) {
								Route r1 = new Route(nodes[i], nodes[hList[u]], nodes[hList[v]], nodes[j],
										distances, alpha);
								Route r2 = new Route(nodes[i], nodes[hList[v]], nodes[hList[u]], nodes[j],
										distances, alpha);
								if (r1.expCost <= r2.expCost) {
									routes[r1.i.ID][r1.k.ID][r1.m.ID][r1.j.ID] = r1;
									feasibleRoutes.add(r1);
								} else {
									routes[r2.i.ID][r2.k.ID][r2.m.ID][r2.j.ID] = r2;
									feasibleRoutes.add(r2);
								}
							} else if (routes[i][hList[u]][hList[v]][j] != null) {
								feasibleRoutes.add(routes[i][hList[u]][hList[v]][j]);
							} else {
								feasibleRoutes.add(routes[i][hList[v]][hList[u]][j]);
							}
						}
					}
				}
				return feasibleRoutes;
	}
	
	public static PriorityQueue<Route> getFeasibleRoutes2(
			int i, 
			int j, 
			int[] hList,
			Node[] nodes,
			Route[][][][] routes,
			double[][] distances,
			double alpha
			){
		// generating list of feasible routes between the origin and the
				// destination.
		PriorityQueue<Route> feasibleRoutes = new PriorityQueue<Route>();
				if (nodes[i].isHub && nodes[j].isHub) {
					if (routes[i][i][j][j] == null)
						routes[i][i][j][j] = new Route(nodes[i], nodes[i], nodes[j], nodes[j],
								distances, alpha);
					feasibleRoutes.add(routes[i][i][j][j]);
				} else if (nodes[i].isHub) {
					for (Integer n : hList) {
						if (routes[i][i][n][j] == null)
							routes[i][i][n][j] = new Route(nodes[i], nodes[i], nodes[n], nodes[j],
									distances, alpha);
						feasibleRoutes.add(routes[i][i][n][j]);
					}
				} else if (nodes[j].isHub) {
					for (Integer n : hList) {
						if (routes[i][n][j][j] == null)
							routes[i][n][j][j] = new Route(nodes[i], nodes[n], nodes[j], nodes[j],
									distances, alpha);
						feasibleRoutes.add(routes[i][n][j][j]);
					}
				} else {
					for (int u = 0; u < hList.length; u++) {
						for (int v = u; v < hList.length; v++) {
							if (routes[i][hList[u]][hList[v]][j] == null
									&& routes[i][hList[v]][hList[u]][j] == null) {
								Route r1 = new Route(nodes[i], nodes[hList[u]], nodes[hList[v]], nodes[j],
										distances, alpha);
								Route r2 = new Route(nodes[i], nodes[hList[v]],nodes[hList[u]], nodes[j],
										distances, alpha);
								if (r1.expCost <= r2.expCost) {
									routes[r1.i.ID][r1.k.ID][r1.m.ID][r1.j.ID] = r1;
									feasibleRoutes.add(r1);
								} else {
									routes[r2.i.ID][r2.k.ID][r2.m.ID][r2.j.ID] = r2;
									feasibleRoutes.add(r2);
								}
							} else if (routes[i][hList[u]][hList[v]][j] != null) {
								feasibleRoutes.add(routes[i][hList[u]][hList[v]][j]);
							} else {
								feasibleRoutes.add(routes[i][hList[v]][hList[u]][j]);
							}
						}
					}
				}
				return feasibleRoutes;
	}
	/**
	 * returns a list of routes that do not contain node h as hub.
	 * @param avlRoutes - initial list of routes.
	 * @param h - the node to be removed from the routes.
	 * @return
	 */
	public static ArrayList<Route> getAvlRoutes(ArrayList<Route> avlRoutes, Node h){
		ArrayList<Route> output = new ArrayList<Route>();
		for (Route r : avlRoutes)
			if (!r.k.equals(h) && !r.m.equals(h))
				output.add(r);
		return output;
	}
	
	public String toString(){
		String output = "";
		for (int i = 0 ; i < routes.length ; i++){
			if (routes[i] != null )
				output = output.concat(i + ") " + routes[i] + "\r\n");
		}
		output = output.concat("value: " + this.value);
		return output;
	}
}
