package model;


public class RoutingTree {
	public Route[] routes;
	public NodeList[] usedHubs;
	public double value;
	
//	public RoutingTree () {};
	
	public RoutingTree (int size){
		routes = new Route[size];
		usedHubs = new NodeList[size];
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
		for ( Node n : this.usedHubs[i].list )
			output *= n.failure;
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
