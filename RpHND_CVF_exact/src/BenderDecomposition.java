import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.util.CombinatoricsUtils;

import gurobi.GRB;
import gurobi.GRBColumn;
import gurobi.GRBConstr;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBVar;
import model.HubComb;
import model.HubCombGenerator;
import model.KMedoids;
import model.Node;
import model.Route;
import model.RoutingTree;

public class BenderDecomposition {
	public static double[][] failures, distances, fixedCosts, flows;
	public static int nVar,nConst,P,L,M;
	public static double alpha;
	public static List<Node> nodes;
	public static Route[][][][] routes;
	public static List<HubComb> hubCombs;
	public static List<Integer> potentialHubs;
	public static GRBEnv env;
	
	
	public static void main ( String[] args) 
			throws IOException, InterruptedException, GRBException{
		failures = MyArray.read("Datasets/CAB/Failures/01-05/failures.txt");
		distances = MyArray.read("Datasets/CAB/CAB10/Distances.txt");
		nVar = distances.length;
		nConst = (nVar*(nVar-1)/2)*(1 + nVar) + 1; 
		flows = MyArray.read("Datasets/CAB/CAB10/Flows.txt");
		fixedCosts = MyArray.read("Datasets/CAB/CAB10/fixedcharge.txt");
		P = 3;
		L = 2; // maximum number of failures
		alpha = 0.2;
		M = (int) CombinatoricsUtils.binomialCoefficient(
				nVar - 1, P - 1); // big M
		nodes = new ArrayList<Node>();
		routes = new Route[nVar][nVar][nVar][nVar];
		hubCombs = new ArrayList<HubComb>();
		potentialHubs = KMedoids.run("Datasets/CAB/CAB10/Distances.txt",P);
		
		env = new GRBEnv();
		env.set( GRB.IntParam.OutputFlag , 1);
		
		GRBModel MP1 = new GRBModel(env);
		
		// -------------- initializing node objects & adding variables to the MasterProblem 1-------------
		GRBVar z = MP1.addVar(-1*GRB.INFINITY, GRB.INFINITY, 0, GRB.CONTINUOUS, "z");
		GRBVar[] y = new GRBVar[nVar];
		for (int i = 0; i < nVar; i++) {
			nodes.add(new Node(i, failures[i][0]));
			y[i] = MP1.addVar(0, 1, 0, GRB.BINARY, "y" + i);
		}
		MP1.update();
		
		// -------------- adding constraints to MasterProblem 1: sum(y_i) = P ---------------
		GRBLinExpr expr = new GRBLinExpr();
		for (int i : potentialHubs)
			expr.addTerm(1, y[i]);
		GRBConstr temp = MP1.addConstr(expr, GRB.EQUAL, P, null);			
		
		expr = new GRBLinExpr();
		for ( GRBVar var : y )
			expr.addTerm(1, var);
		MP1.addConstr(expr, GRB.EQUAL, P, null); // sum(y_i) = P
		MP1.optimize();
		printSol(MP1, "all");
		
		// ---------------- solving sub sub problem (routing problem) for 
		GRBModel SP1 = solveSP1(potentialHubs, y);
//		printSol(SP1, "nonzeros");
		SP1.write("moyModel22.lp");
//		solvePP();
//		for (GRBConstr c : SP1.getConstrs())
//			System.out.println(c.get(GRB.StringAttr.ConstrName) + ": " + c.get(GRB.DoubleAttr.Pi));
		
		Array2DRowRealMatrix b = getRHS(SP1, nConst);
		Array2DRowRealMatrix u = getDuals(SP1, nConst);
		Array2DRowRealMatrix B = getCoeffs(SP1, nConst, nVar);	
		RealMatrix c = b.transpose().multiply(u);
		System.out.println("");
	}
	
	private static GRBModel solveSP1(List<Integer> potentialHubs, GRBVar[] y) throws GRBException{
		// take initial hubs
		// enumerate hub combinations
		// build up a restricted master problem
		// solve the pricing problem
		// either add new combinations based on the result of the previous step or terminate the column generation process
		
		GRBModel SP1 = new GRBModel(env);
		// -----------    initializing hub combinations based on current list of potential hubs -------------
		List<HubComb> hubCombs = getHubCombs(potentialHubs, P);
		
		// ------   mapping node indices to hub combinations   ----------
		HashMap<Integer, ArrayList<HubComb>> map = getMap(hubCombs);
		
		// ---------  initializing location variables variables   ----------
		GRBVar[] l = new GRBVar[nVar];
		for (int i = 0 ; i < nVar ; i++)
			l[i] = SP1.addVar(0, 1, 0, GRB.CONTINUOUS, "z"+i);
		
		// ---------  initializing routing tree variables   ----------  
		GRBVar[][][] x = new GRBVar[nVar][nVar][hubCombs.size()];
		RoutingTree[][][] routingTrees = new RoutingTree[nVar][nVar][hubCombs
				.size()];
		for (int i = 0; i < nVar; i++) 
			for (int j = i + 1; j < nVar; j++) 
				for (int k = 0; k < hubCombs.size(); k++) {
					double RTUB = getUpperbound(nodes.get(i),
							nodes.get(j), hubCombs.get(k).hubs, L);  // the upper bound obtained by the greedy algorithm
					routingTrees[i][j][k] = getRoutingTree(nodes.get(i),
							nodes.get(j), hubCombs.get(k).hubs, L, RTUB);
					x[i][j][k] = SP1.addVar(0, 1, flows[i][j]
							* routingTrees[i][j][k].value, GRB.CONTINUOUS, "x"
							+ i + "_" + j + "_" + k);
				}
		SP1.update();
		
		// Adding constrains
		GRBLinExpr expr;
		int constCount = 0;
		// constraints (2)
		GRBConstr[][] c2 = new GRBConstr[nVar][nVar];
		for (int i = 0; i < nVar; i++) {
			for (int j = i + 1; j < nVar; j++) {
				expr = new GRBLinExpr();
				for (int k = 0; k < hubCombs.size(); k++) {
					expr.addTerm(1, x[i][j][k]);
				}
				c2[i][j] = SP1.addConstr(expr, GRB.EQUAL, 1, constCount++ + "");
			}
		}
		
		// constraint (3)
		expr = new GRBLinExpr();
		for (int i = 0 ; i < nVar ; i++)
			expr.addTerm(1, l[i]);		
		GRBConstr c3 = SP1.addConstr(expr, GRB.EQUAL, P, constCount++ + "");
		
		// constraints (4)
		GRBConstr[][][] c4 = new GRBConstr[nVar][nVar][nVar];
		for (int i = 0; i < nVar; i++) 
			for (int j = i + 1; j < nVar; j++) 
				for (int k = 0; k < nVar; k++) {
					expr = new GRBLinExpr();
					if (potentialHubs.contains(k)){
						for (HubComb h : map.get(k))
							expr.addTerm(1, x[i][j][h.ID]);
						expr.addTerm(-1 * M * y[k].get(GRB.DoubleAttr.X),l[k]);
					c4[i][j][k] = SP1.addConstr(expr, GRB.LESS_EQUAL, 0, constCount++ + "");
					}else{
						expr.addTerm(-1*M, l[k]);
						c4[i][j][k] = SP1.addConstr(expr, GRB.LESS_EQUAL, 0, constCount++ + "");
					}
				}
		
		// fixing hubs
		GRBConstr[] temp = new GRBConstr[P];
		for (int i = 0 ; i < nVar ; i++)
			if ( y[i].get(GRB.DoubleAttr.X) > 0 )
				SP1.addConstr(l[i], GRB.EQUAL, 1, constCount++ + "");
		
//		for (GRBConstr c : temp)
//			SP1.remove(c);
		SP1.optimize();	
		return SP1;
	}
	
	private static void solvePP() throws GRBException{
		// to be implemented
	}
	
	/**
	 * Returns all the possible hub combinations of size P from hubsList
	 * @param hubsList
	 * @param P
	 * @return List<HubComb>
	 */
	private static List<HubComb> getHubCombs(List<Integer> hubsList, int P){
		List<HubComb> output = new ArrayList<HubComb>();
		List<Set<Integer>> subsets = HubCombGenerator.getSubsets(hubsList, P);
		ArrayList<Node> hubs = new ArrayList<>();
		int counter = 0;
		for (Set<Integer> s : subsets){
			hubs = new ArrayList<>();
			for (int i : s)
				hubs.add(nodes.get(i));
			output.add(new HubComb(counter++, hubs, nodes, fixedCosts));
		}
		return output;
	}
	
	/**
	 * Maps each node to all the hub combinations that use the node as a hub
	 * 
	 * @param hubCombinations
	 * @return HashMap<Integer, ArrayList<HubComb>>
	 */
	private static HashMap<Integer, ArrayList<HubComb>> getMap(List<HubComb> hubCombinations){
		HashMap<Integer, ArrayList<HubComb>> map = new HashMap<Integer, ArrayList<HubComb>>();
		for (int i = 0; i < nVar; i++)
			map.put(i, new ArrayList<HubComb>());
		for (HubComb h : hubCombinations)
			for (Node n : h.hubs)
				map.get(n.ID).add(h);
		return map;
	}
	
	/**
	 * Prints the basic variables.s
	 * 
	 * @param model
	 * @throws GRBException
	 */
	public static void printSol(GRBModel model, String flag) throws GRBException {
		if (flag.equals("nonzeros")){
			for (GRBVar var : model.getVars()) {
				double value = var.get(GRB.DoubleAttr.X);
				if (value != 0)
					System.out.println(var.get(GRB.StringAttr.VarName) + ": "
							+ value + ": " + var.get(GRB.DoubleAttr.Obj));
			}
		} else if (flag.equals("all")){
			for (GRBVar var : model.getVars()) 
					System.out.println(var.get(GRB.StringAttr.VarName) + ": "
							+ var.get(GRB.DoubleAttr.X) + ": " + var.get(GRB.DoubleAttr.Obj));			
		} 		
	}
	
	/**
	 * returns an upper bound on the best routing tree of depth l from i to j using hub combination hList
	 * @param i
	 * @param j
	 * @param hList
	 * @param l
	 * @return double
	 */
	public static double getUpperbound(Node i, Node j, List<Node> hList,
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

		return output.value;
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
	 * Returns the optimal routing tree of depth l from i to j using hub combination hList
	 * @param i
	 * @param j
	 * @param hList
	 * @param l
	 * @param UB
	 * @return RoutingTree
	 */
	public static RoutingTree getRoutingTree(Node i, Node j, List<Node> hList,
			int l, double UB) {
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
			RoutingTree rt = new RoutingTree(new Route[(int) Math.pow(2, l + 1) - 1], r, feasibleRoutes, L, UB);
			if ( rt.isComplete() ){
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
			
			if (!currentNode.isComplete()){
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
					RoutingTree rt = new RoutingTree(currentNode,childNodeInd,r,avlRoutes,L, UB);
					// Check for the best value and the best routing-tree 
					if (rt.isComplete() ){
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
	
	private static Array2DRowRealMatrix getRHS(GRBModel model, int nConst ) throws GRBException{
		double[] temp = new double[nConst];
		for ( int i = 0 ; i < nConst ; i++)
			temp[i] = model.getConstr(i).get(GRB.DoubleAttr.RHS); 
		return new Array2DRowRealMatrix(temp);
	}
	
	private static Array2DRowRealMatrix getDuals(GRBModel model, int nConst ) throws GRBException{
		double[] temp = new double[nConst];
		for ( int i = 0 ; i < nConst ; i++)
			temp[i] = model.getConstr(i).get(GRB.DoubleAttr.Pi); 
		return new Array2DRowRealMatrix(temp);
	}
	
	private static Array2DRowRealMatrix getCoeffs(GRBModel model, int nConst, int nVar) throws GRBException{
		
		double[][] temp = new double[nConst][nVar];
		for (int i = 0 ; i < nVar ; i++){
			GRBColumn col = model.getCol(model.getVar(i));
			for (int l = 0 ; l < col.size() ; l++){
				try{
					temp[Integer.parseInt(col.getConstr(l).get(GRB.StringAttr.ConstrName))][i] = col.getCoeff(l);
				}catch(ArrayIndexOutOfBoundsException e){
					System.out.println(e);
				}
			}
		}
		return new Array2DRowRealMatrix(temp);
	}
}
