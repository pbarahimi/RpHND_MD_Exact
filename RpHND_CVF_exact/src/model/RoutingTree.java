package model;

import java.util.ArrayList;
import java.util.List;

public class RoutingTree {
	public ArrayList<Route> routes = new ArrayList<Route>();
	public ArrayList<NodeList> usedHubs;
	public ArrayList<Route> availableRoutes;
	public double value;
	public boolean complete = false;
	
	public RoutingTree (ArrayList<Route> routes, Route newRoute, ArrayList<Route> availableRoutes, int L) {
		this.routes.add(newRoute);
		this.availableRoutes = availableRoutes;
		updateValue();
		// check if the tree is completed, according to the required depth.
		if ( routes.size() >= Math.pow(2, L+1)-2 )
			this.complete = true;
	};
	
	public RoutingTree (RoutingTree other) {
		this.routes = other.routes;
		this.usedHubs = other.usedHubs;
		this.value = other.value;
		this.availableRoutes = other.availableRoutes;
		this.complete = other.complete;
	};
	
	public void updateValue(){		
		this.value = this.routes.get(0).expCost;
		for (int i = 1 ; i < routes.size() ; i++){
			if (this.routes.get(i) != null)
				this.value += this.routes.get(i).cost * getFailureProb(i);
		}	
	}
	
	private double getFailureProb(int i ){
		double output = 1;
		for ( Node n : this.usedHubs.get(i).list )
			output *= n.failure;
		return output;
	}
	
	public static ArrayList<Route> getFeasibleRoutes(
			Node i, 
			Node j, 
			List<Node> hList,
			Route[][][][] routes,
			double[][] distances,
			double alpha
			){
		// generating list of feasible routes between the origin and the
				// destination.
				ArrayList<Route> feasibleRoutes = new ArrayList<Route>();
				if (i.isHub && j.isHub) {
					if (routes[i.ID][i.ID][j.ID][j.ID] == null)
						routes[i.ID][i.ID][j.ID][j.ID] = new Route(i, i, j, j,
								distances, alpha);
					feasibleRoutes.add(routes[i.ID][i.ID][j.ID][j.ID]);
				} else if (i.isHub) {
					for (Node n : hList) {
						if (routes[i.ID][i.ID][n.ID][j.ID] == null)
							routes[i.ID][i.ID][n.ID][j.ID] = new Route(i, i, n, j,
									distances, alpha);
						feasibleRoutes.add(routes[i.ID][i.ID][n.ID][j.ID]);
					}
				} else if (j.isHub) {
					for (Node n : hList) {
						if (routes[i.ID][n.ID][j.ID][j.ID] == null)
							routes[i.ID][n.ID][j.ID][j.ID] = new Route(i, n, j, j,
									distances, alpha);
						feasibleRoutes.add(routes[i.ID][n.ID][j.ID][j.ID]);
					}
				} else {
					for (int u = 0; u < hList.size(); u++) {
						for (int v = u; v < hList.size(); v++) {
							if (routes[i.ID][hList.get(u).ID][hList.get(v).ID][j.ID] == null
									&& routes[i.ID][hList.get(v).ID][hList.get(u).ID][j.ID] == null) {
								Route r1 = new Route(i, hList.get(u), hList.get(v), j,
										distances, alpha);
								Route r2 = new Route(i, hList.get(v), hList.get(u), j,
										distances, alpha);
								if (r1.value <= r2.value) {
									routes[r1.i.ID][r1.k.ID][r1.m.ID][r1.j.ID] = r1;
									feasibleRoutes.add(r1);
								} else {
									routes[r2.i.ID][r2.k.ID][r2.m.ID][r2.j.ID] = r2;
									feasibleRoutes.add(r2);
								}
							} else if (routes[i.ID][hList.get(u).ID][hList.get(v).ID][j.ID] != null) {
								feasibleRoutes.add(routes[i.ID][hList.get(u).ID][hList
										.get(v).ID][j.ID]);
							} else /*
									 * if (
									 * !routes[i.ID][hList.get(v).ID][hList.get(u).ID
									 * ][j.ID].equals(null) )
									 */{
								feasibleRoutes.add(routes[i.ID][hList.get(v).ID][hList
										.get(u).ID][j.ID]);
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
		for (int i = 0 ; i < routes.size() ; i++){
			if (routes.get(i) != null )
				output = output.concat(i + ") " + routes.get(i) + "\r\n");
		}
		output = output.concat("value: " + this.value);
		return output;
	}
}
