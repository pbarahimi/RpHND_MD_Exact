package model;

import gurobi.GRB;
import gurobi.GRB.IntParam;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBVar;

public class HND {
	private final double[][] distances;
	private final double[][] flows;
	private final int p;
	private final double alpha;
	public double objFunVal;
	public int[] hubs;

	public HND(double[][] distances, double[][] flows, int p, double alpha)
			throws GRBException {
		this.distances = distances;
		this.flows = flows;
		this.p = p;
		this.alpha = alpha;
		this.hubs = new int[p];
		solve();
	}

	private void solve() throws GRBException {
		int nVar = this.flows.length;
		GRBEnv env = new GRBEnv();
		env.set(IntParam.OutputFlag, 1);
		GRBModel model = new GRBModel(env);		
		
		// Create variables
		GRBVar[][][][] x = new GRBVar[nVar][nVar][nVar][nVar];
		GRBVar[] y = new GRBVar[nVar];
		
		// ------------ Routing Variables ---------------
		for (int i = 0; i < nVar; i++)
			for (int j = i + 1; j < nVar; j++)
				for (int k = 0; k < nVar; k++)
					for (int m = 0; m < nVar; m++) {
						String varName = "x" + i + "_" + k + "_" + m + "_" + j
								+ "_0";
						double coefficient = this.flows[i][j]
								* HND.getRouteCost(i, k, m, j, this.distances,
										this.alpha);
						x[i][k][m][j] = model.addVar(0.0, 1.0, coefficient,
								GRB.CONTINUOUS, varName);
					}

		// --------------- Location Variables -------------
		for (int i = 0; i < nVar; i++)
			y[i] = model.addVar(0, 1, 0, GRB.BINARY, "y" + i);
		
		
		model.update();
		
		// ---------------Adding constraints ---------------
		// constraint 1
		GRBLinExpr expr = new GRBLinExpr();
		for (GRBVar var : y)
			expr.addTerm(1, var);
		model.addConstr(expr, GRB.EQUAL, this.p, "c1");
		
		// constraint 2
		for (int i = 0 ; i < nVar ; i++ )
			for (int j = i+1 ; j < nVar ; j++ ){
				expr = new GRBLinExpr();
				for (int k = 0 ; k < nVar ; k++ )
					for (int m = 0 ; m < nVar ; m++ )
						expr.addTerm(1,x[i][k][m][j]);
				model.addConstr(expr, GRB.EQUAL, 1, null);
					
			}
		
		// constraint 3
		for (int i = 0 ; i < nVar ; i++ )
			for (int j = i+1 ; j < nVar ; j++ )
				for (int k = 0 ; k < nVar ; k++ )
					for (int m = 0 ; m < nVar ; m++ ){
						expr = new GRBLinExpr();
						expr.addTerm(1,x[i][k][m][j]);
						expr.addTerm(-1, y[k]);
						model.addConstr(expr, GRB.LESS_EQUAL, 0, null);
						
						expr = new GRBLinExpr();
						expr.addTerm(1,x[i][k][m][j]);
						expr.addTerm(-1, y[m]);
						model.addConstr(expr, GRB.LESS_EQUAL, 0, null);
					}
						
		// ------------- solving model ---------------
		model.optimize();
		
		// ------------- updating solution attributes---------------
		this.objFunVal = model.get(GRB.DoubleAttr.ObjVal);
		int cntr = 0;
		for ( int i = 0 ; i < nVar ; i++ )
			if ( y[i].get(GRB.DoubleAttr.X) != 0 )
					this.hubs[cntr++] = i;
							
		// ------------- disposing model and environment ------------
		model.dispose();
		env.dispose();

	}

	private static double getRouteCost(int i, int k, int m, int j,
			double[][] distances, double alpha) {
		return distances[i][k] + (1 - alpha) * distances[k][m]
				+ distances[m][j];
	}
	
	private static void printSol(GRBModel model, String flag) throws GRBException {
		if (flag.equals("nonzeros")) {
			for (GRBVar var : model.getVars()) {
				double value = var.get(GRB.DoubleAttr.X);
				if (value != 0)
					System.out.println(
							var.get(GRB.StringAttr.VarName) + ": " + value + ": " + var.get(GRB.DoubleAttr.Obj));
			}
		} else if (flag.equals("all")) {
			for (GRBVar var : model.getVars())
				System.out.println(var.get(GRB.StringAttr.VarName) + ": " + var.get(GRB.DoubleAttr.X) + ": "
						+ var.get(GRB.DoubleAttr.Obj));
		}
	}
}
