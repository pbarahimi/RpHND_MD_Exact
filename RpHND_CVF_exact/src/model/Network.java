package model;

public class Network{
	public final int[] hubs;
	public final RoutingTree[][] routingTrees;
	public final double cost;
	public String results;
	
	public Network(int[] hubs, RoutingTree[][] RT, double[][] flows){
		this.hubs = hubs;
		this.routingTrees = RT;
		int nodesNum = RT.length;
		double cost = 0;
		for ( int i = 0 ; i < nodesNum ; i++ )
			for ( int j = i + 1 ; j < nodesNum ; j++ )
				cost += routingTrees[i][j].value * flows[i][j];
		this.cost = cost;
	}
}