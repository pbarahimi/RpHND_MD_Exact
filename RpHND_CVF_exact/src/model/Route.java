package model;

import gurobi.GRBVar;

public class Route implements Comparable<Route>{
	public final Node i;
	public final Node k;
	public final Node m;
	public final Node j;
	public final double cost; // the route-cost regardless of failure probabilities
	public final double expCost; // expected cost of the route
//	public final double value;	// the route-cost considering the failure probability of the hubs
	public GRBVar var;
	
	/**
	 * Constructor
	 * 
	 * @param h1
	 * @param h2
	 * @param load
	 * @param alpha
	 */
	public Route ( Node i, Node k, Node m, Node j, double[][] distances, double alpha ){
		this.i = i;
		this.k = k;
		this.m = m;
		this.j = j;
		this.cost = getRouteCost ( distances, alpha );
		this.expCost = getExpCost ( alpha );
	}
	
	/**
	 * Copy constructor
	 * @param other
	 */
	public Route ( Route other ){
		this.i = other.i;
		this.j = other.j;
		this.k = other.k;
		this.m = other.m;
		this.cost = other.cost;
		this.expCost = other.expCost;
//		this.value = other.value;		
	}
	
	/**
	 * calculates the distance of a route given a load,
	 * origin, destination and a discount factor. 
	 * 
	 * @param load
	 * @param alpha
	 * @return
	 */
	private double getRouteCost( double[][] distances, double alpha ){
		double output = getDistance(i, k, distances, alpha);
		output += getDistance(k, m, distances, alpha);
		output += getDistance (m, j, distances, alpha);	
//		System.out.println(load.origin + "_" + hub1 + "_" + hub2 + "_" + load.destination + ": " + output);
		return output;		
	}
	
	private double getExpCost ( double alpha ){
		double output = this.cost;
		if ( !k.equals(m) ) {
			if ( i.equals(k) && !j.equals(m)) //iimj
				output *= 1 - m.failure;
			else if ( !i.equals(k) && j.equals(m) )  //ikjj
				output *= 1 - k.failure;
			else if ( !i.equals(k) && !j.equals(m) ) //ikmj
				output *= 1 - k.failure - m.failure;
			else //iijj
				output *= 1;
		} else {
			if ( !i.equals(k) && !j.equals(m) ) //ikkj
				output *= 1 - k.failure;
			else //iiij or ijjj
				output *= 1;
		}
		return output;
	}
	
	/**
	 * returns the Euclidean distance between the two nodes considering
	 * the discount factor if the two nodes are hubs.
	 * @param n1
	 * @param n2
	 * @return
	 */
	private double getDistance (Node n1, Node n2, double[][] distances, double alpha ){
		double coefficient = 1;
		if (n1.isHub && n2.isHub) coefficient = 1 - alpha;
		return coefficient * distances[n1.ID][n2.ID];//(Math.sqrt(Math.pow(n1.x - n2.x, 2) + Math.pow(n1.y - n2.y, 2)));
	}
	
	@Override
	public String toString(){
		return "(" + this.i + "," + this.k + "," + this.m + "," + this.j + ") - " + this.cost +/* "-" + this.value + */"-" +this.expCost;
	}
	
	@Override
	public int compareTo (Route other){
		if ( this.expCost < other.expCost )
			return -1;
		else if ( this.expCost > other.expCost )
			return 1;
		else 
			return 0;
	}
}
