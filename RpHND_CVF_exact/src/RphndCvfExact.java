import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectInputStream.GetField;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

import org.apache.commons.math3.util.CombinatoricsUtils;

import gurobi.GRB;
import gurobi.GRBConstr;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBVar;
import model.HubComb;
import model.Node;
import model.NodeList;
import model.Route;
import model.RoutingTree;

public class RphndCvfExact{
	// private static final double[][] coordinates =
	// MyArray.read("coordinates.txt");
	// private static final double[][] tmpFlows = MyArray.read("w.txt");
	private static double[][] failures;
	private static double[][] distances;
	private static int nVar;
	private static double[][] flows;
	private static  double[][] fixedCosts;
	private static int P;
	private static int L; // maximum number of failures
	private static double alpha ;
	private static long M ;
	private static ArrayList<Node> nodes;
	private static Route[][][][] routes;
	private static List<HubComb> hubCombs;

	public static void main(String[] args) throws GRBException, IOException {
//		String path = "C:/Gurobi_Results/120416/";
//		File file = new File(path + "results.txt");
//		FileWriter fw = new FileWriter(file);
		
		/*int[] Instance = {0};
		int[] N = {20};
		int[] hub = {5};
		double[] discount = {0.2};
		String[] failure = {
				"01-05",
				"05-10",
				"10-15",
				"15-20",
				"20-25",
				"01-10",
				"01-15",
				"01-20",
				"01-25"				
		};
		int[] L = {2}; 
		
		for (int l : L){
			for(int n : N){
				for (int h : hub){
					for (double d : discount){
						for (String s:failure){
							for (int i:Instance){
								if (l<h)
									fw.append(run(n, h, d, s, l, i) + "\r\n");	
							}
						}
					}
				}
			}
		}*/
		run(15, 3, 0.2, "20-25", 2, 4);
//		fw.append(run(15, 3, 0.2, "20-25", 0, 4) + "\r\n");
//		fw.close();
	}
	
	
	private static String run(int N, int hub, double discount, String failure, int l, int instance) throws GRBException, IOException {
		double startTime = System.currentTimeMillis();
		failures = MyArray.read("Failures.txt");
		distances = MyArray.read("Distances.txt");
		nVar = distances.length;
		flows = MyArray.read("Flows.txt");
		fixedCosts = MyArray.read("Fixedcharge.txt");
		P = hub;
		L = l; // maximum number of failures
		alpha = discount;
		M = CombinatoricsUtils.binomialCoefficient(
				nVar - 1, P - 1); // big M
		nodes = new ArrayList<Node>();
		routes = new Route[nVar][nVar][nVar][nVar];
		hubCombs = new ArrayList<HubComb>();
		
		// build Gurobi model and environment.
		GRBEnv env = new GRBEnv(null);
		
		
		//env.set(	GRB.IntParam.OutputFlag , 0);
		GRBModel model = new GRBModel(env);
		
		// initializing node objects.
		GRBVar[] y = new GRBVar[nVar];
		for (int i = 0; i < nVar; i++) {
			nodes.add(new Node(i, /* coordinates[i][0], coordinates[i][1], */
					failures[i][0]));
			y[i] = model.addVar(0, 1, 0, GRB.BINARY, "y" + i);
		}

		// initializing hub combinations.
		generateHubCombs(nodes, P, hubCombs, fixedCosts);

		
		// mapping node indexes to hub combinations
		HashMap<Integer, ArrayList<HubComb>> map = new HashMap<Integer, ArrayList<HubComb>>();
		for (int i = 0; i < nVar; i++)
			map.put(i, new ArrayList<HubComb>());
		for (HubComb h : hubCombs) {
			for (Node n : h.hubs)
				map.get(n.ID).add(h);
		}
		int ctr = 0;
		// initializing routing tree variables.
		GRBVar[][][] x = new GRBVar[nVar][nVar][hubCombs.size()];
		RoutingTree[][][] routingTrees = new RoutingTree[nVar][nVar][hubCombs
				.size()];
		for (int i = 0; i < nVar; i++) {
			for (int j = i + 1; j < nVar; j++) {
				for (int k = 0; k < hubCombs.size(); k++) {
					routingTrees[i][j][k] = getRoutingTree2(nodes.get(i),
							nodes.get(j), hubCombs.get(k).hubs, L);
					System.out.println(i+"_"+j+"_"+k+" : " + routingTrees[i][j][k].value);
					x[i][j][k] = model.addVar(0, 1, flows[i][j]
							* routingTrees[i][j][k].value, GRB.CONTINUOUS, "x"
							+ i + "_" + j + "_" + k);ctr++;
				}
			}
		}
		System.out.println("Routing trees generated! " + ctr);
		System.out.println(routingTrees[8][9][116]);
		model.update();

		// Adding constrains
		GRBLinExpr expr;

		// constraints (2)
		GRBConstr[][] c2 = new GRBConstr[nVar][nVar];
		for (int i = 0; i < nVar; i++) {
			for (int j = i + 1; j < nVar; j++) {
				expr = new GRBLinExpr();
				for (int k = 0; k < hubCombs.size(); k++) {
					expr.addTerm(1, x[i][j][k]);
				}
				c2[i][j] = model.addConstr(expr, GRB.EQUAL, 1, "c2" + i + "_"
						+ j);
			}
		}

		// constraint (3)
		expr = new GRBLinExpr();
		for (int i = 0; i < nVar; i++) {
			expr.addTerm(1, y[i]);
		}
		GRBConstr c3 = model.addConstr(expr, GRB.EQUAL, P, "c3");

		// constraints (4)
		GRBConstr[][][] c4 = new GRBConstr[nVar][nVar][hubCombs.size()];
		for (int i = 0; i < nVar; i++) {
			for (int j = i + 1; j < nVar; j++) {
				for (int k = 0; k < nVar; k++) {
					expr = new GRBLinExpr();
					for (HubComb h : map.get(k)) {
						expr.addTerm(1, x[i][j][h.ID]);
					}
					expr.addTerm(-M, y[k]);
					c4[i][j][k] = model.addConstr(expr, GRB.LESS_EQUAL, 0, "c4"
							+ i + "_" + j + "_" + k);
				}
			}
		}
		double buildTime = System.currentTimeMillis() - startTime;
		startTime = System.currentTimeMillis();
		// solve model
		model.optimize();
		
		// writing solution to file
		File file = new File ("solution_"+N+"_"+P+"_"+discount+"_"+failure+"_"+l+"_"+instance+".sol");
		FileWriter fw = new FileWriter(file);
		fw.append(getSol(model));
		fw.close();
		
		// printing general info on the solution on to the console.
		System.out.print(N+","+P+","+discount+","+failure+","+l);
		System.out.print(",Obj:,");
		System.out.print(model.get(GRB.DoubleAttr.ObjVal)+",hubs:,");
		for (GRBVar var : model.getVars()){
			if (var.get(GRB.DoubleAttr.X) != 0 && var.get(GRB.StringAttr.VarName).contains("y")){
				System.out.print(var.get(GRB.StringAttr.VarName)+" ");
			}			
		}
		
		// saving the results into file
		String output = "";
		output += N+","+P+","+discount+","+failure+","+l+"," + instance + ",Obj:,"+model.get(GRB.DoubleAttr.ObjVal)+",hubs:,";
		for (GRBVar var : model.getVars()){
			if (var.get(GRB.DoubleAttr.X)!=0 && var.get(GRB.StringAttr.VarName).contains("y")){
				output += var.get(GRB.StringAttr.VarName)+ " ";
			}
		}
		System.out.print(",Solution Time:,"
				+ (System.currentTimeMillis() - startTime));
		System.out.println(",Build Time:," + buildTime);
		output += ",Solution Time:,"
				+ (System.currentTimeMillis() - startTime) + ",Build Time:," + buildTime;
		model.dispose();
		env.dispose();
		return output;		
	}

	private static RoutingTree getRoutingTree2(Node i, Node j, List<Node> hList,
			int l) {
		double bestRTValue = GRB.INFINITY;
		RoutingTree bestRT = new RoutingTree((int) Math.pow(2, l + 1) - 1);
		
		// Update the nodes in the hubsList by setting the isHub to true
		for (Node n : hList)
			n.isHub = true;

		// generating list of feasible routes between the origin and the destination.
		ArrayList<Route> feasibleRoutes = RoutingTree.getFeasibleRoutes(i, j, hList, routes, distances, alpha);
		
		// instantiating unexplored RoutingTreeNodes
		ArrayList<RoutingTree> unexploredNodes = new ArrayList<RoutingTree>();

		// initializing a RoutingTreeNode for each feasible route.
		for (Route r : feasibleRoutes){
			RoutingTree rt = new RoutingTree(new Route[(int) Math.pow(2, l + 1) - 1], r, feasibleRoutes, L);
			if (rt.complete ){
				if ( rt.value < bestRTValue ){
					bestRT = rt;
					bestRTValue = bestRT.value;
				}
			}else if (!rt.pruned)
				unexploredNodes.add(rt);
		}
		
		while (!unexploredNodes.isEmpty()) {
			RoutingTree currentNode = unexploredNodes.get(unexploredNodes.size()-1);
			unexploredNodes.remove(unexploredNodes.size()-1);
			
			if (!currentNode.complete){
				int childNodeInd = currentNode.unexploredNodes.get(0); 
				int parentNodeInd = (int) Math.floor((childNodeInd-1)/2);
				
				String flag = "left";
				if ( childNodeInd % 2 == 0)
					flag = "right";
				
				ArrayList<Route> avlRoutes = currentNode.availableRoutes.get(parentNodeInd);
				if ( flag.equals("left") ){
					avlRoutes = RoutingTree.getAvlRoutes(avlRoutes, currentNode.routes[parentNodeInd].k );
				}else {
					avlRoutes = RoutingTree.getAvlRoutes(avlRoutes, currentNode.routes[parentNodeInd].m );
				}
				for (Route r : avlRoutes){
					RoutingTree rt = new RoutingTree(currentNode,childNodeInd,r,avlRoutes,L);
					// Check for the best value and the best routing-tree 
					if (rt.complete ){
						if ( rt.value < bestRTValue ){
							bestRT = rt;
							bestRTValue = bestRT.value;
						}
					}else if (!rt.pruned)
						unexploredNodes.add(rt);
				}
			}
			
		}
		// switching node is hubList back to spokes
		for (Node n : hList)
			n.isHub = false;
		
		return bestRT;
	}

	private static RoutingTree getRoutingTree(Node i, Node j, List<Node> hList,
			int l) {
		// Update the nodes in the hubsList by setting the isHub to true
		for (Node n : hList)
			n.isHub = true;

		// instantiating the output
		RoutingTree output = new RoutingTree((int) Math.pow(2, l + 1) - 1);

		// generating list of feasible routes between the origin and the
		// destination.
		PriorityQueue<Route> feasibleRoutes = RoutingTree.getFeasibleRoutes2(i, j, hList, routes, distances, alpha);

		int cntr = 0; // counter
		int lastIndex = (int) (Math.pow(2, l) - 2); // the last index of the
													// second last level of the
													// tree

		Route selectedRoute = feasibleRoutes.poll();
		output.routes[0] = selectedRoute;
		output.usedHubs.put(0, new ArrayList<Node>()); 

		while (!feasibleRoutes.isEmpty() && cntr <= lastIndex) {
			PriorityQueue<Route> feasibleRoutes1 = new PriorityQueue<Route>(
					feasibleRoutes); // List of feasible routes to select left
										// child node from.
			PriorityQueue<Route> feasibleRoutes2 = new PriorityQueue<Route>(
					feasibleRoutes); // List of feasible routes to select right
										// child node from.

			if (output.routes[cntr] != null
					&& !output.routes[cntr].i.equals(output.routes[cntr].k)
					&& !isFinal(output.routes[cntr])) {
				ArrayList<Node> usedHubs1 = new ArrayList<Node>(
						output.usedHubs.get(cntr)); // make a list of parent
														// node's used hubs.
				usedHubs1.add(output.routes[cntr].k); // adding parent node's
														// first hub to the list
				int leftNodeIndex = 2 * cntr + 1;
				while (output.routes[leftNodeIndex] == null) {
					selectedRoute = feasibleRoutes1.poll();

					if (!usedHubs1.contains(selectedRoute.k)
							&& !usedHubs1.contains(selectedRoute.m)) {
						output.routes[2 * cntr + 1] = selectedRoute;
						output.usedHubs.put(2 * cntr + 1,usedHubs1);
					}
				}
			}

			if (output.routes[cntr] != null
					&& !output.routes[cntr].j.equals(output.routes[cntr].m)					
					&& !isFinal(output.routes[cntr])
					&& !output.routes[cntr].k.equals(output.routes[cntr].m)) {
				ArrayList<Node> usedHubs2 = new ArrayList<Node>(
						output.usedHubs.get(cntr)); // make a list of parent
														// node's used hubs.
				usedHubs2.add(output.routes[cntr].m); // adding parent node's
														// first hub to the list
				int rightNodeIndex = 2 * cntr + 2;
				while (output.routes[rightNodeIndex] == null) {
					selectedRoute = feasibleRoutes2.poll();
					if (!usedHubs2.contains(selectedRoute.k)
							&& !usedHubs2.contains(selectedRoute.m)) {
						output.routes[2 * cntr + 2] = selectedRoute;
						output.usedHubs.put(2 * cntr + 2, usedHubs2);
					}
				}
			}
			cntr++;
		}

		// switching node is hubList back to spokes
		for (Node n : hList)
			n.isHub = false;

		// updating the value of the tree
		output.updateValue();

		return output;
	}

	
	private static boolean isFinal(Route r) {
		boolean output = false;
		if (r.k.equals(r.m)) {
			if (r.k.equals(r.i) || r.m.equals(r.j))
				output = true;
		} else if (r.i.equals(r.k) && r.j.equals(r.m)) {
			output = true;
		}
		return output;
	}

	/**
	 * Generates all possible hub combinations of size k from the set of nodes.
	 * 
	 * @param nodes
	 * @param k
	 * @param hubCombs
	 * @param fixedCosts
	 */
	static void generateHubCombs(List<Node> nodes, int k,
			List<HubComb> hubCombs, double[][] fixedCosts) {
		int[] set = new int[nodes.size()];
		for (int i = 0; i < nodes.size(); i++)
			set[i] = i;
		int[] subset = new int[k];
		processLargerSubsets(set, subset, 0, 0, hubCombs, fixedCosts);
	}

	/**
	 * Sub-procedure used to generate hub combinations.
	 * 
	 * @param set
	 * @param subset
	 * @param subsetSize
	 * @param nextIndex
	 * @param hubCombs
	 * @param fixedCosts
	 */
	static void processLargerSubsets(int[] set, int[] subset, int subsetSize,
			int nextIndex, List<HubComb> hubCombs, double[][] fixedCosts) {
		if (subsetSize == subset.length) {
			hubCombs.add(new HubComb(hubCombs.size(), subset, nodes, fixedCosts));

		} else {
			for (int j = nextIndex; j < set.length; j++) {
				subset[subsetSize] = set[j];
				processLargerSubsets(set, subset, subsetSize + 1, j + 1,
						hubCombs, fixedCosts);
			}
		}
	}

	/**
	 * Prints the basic variables.s
	 * 
	 * @param model
	 * @throws GRBException
	 */
	static void printSol(GRBModel model) throws GRBException {
		for (GRBVar var : model.getVars()) {
			double value = var.get(GRB.DoubleAttr.X);
			if (value != 0)
				System.out.println(var.get(GRB.StringAttr.VarName) + ": "
						+ value + ": " + var.get(GRB.DoubleAttr.Obj));
		}
	}
	
	static String getSol(GRBModel model) throws GRBException {
		String output = "Obj value: " + model.get(GRB.DoubleAttr.ObjVal) + "\n";
		for (GRBVar var : model.getVars()) {
			double value = var.get(GRB.DoubleAttr.X);
			if (value != 0)
				output += (var.get(GRB.StringAttr.VarName) + ": "
						+ value + ": " + var.get(GRB.DoubleAttr.Obj) + "\n");
		}
		return output;
	}
}
