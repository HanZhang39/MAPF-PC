#include "CBSHeuristic.h"
#include "CBS.h"
// #include <ilcplex/ilocplex.h>

/*
// integer linear programming for weighted vertex cover
// SCIP
int CBSHeuristic::ILPForWMVC(const vector<int>& CG, const vector<int>& node_max_value)
{
	int N = (int) node_max_value.size();

	// initialize SCIP environment 
	SCIP* scip = NULL;
	SCIP_CALL(SCIPcreate(&scip));

	// Version information
	// SCIPprintVersion(scip, NULL);
	// SCIPinfoMessage(scip, NULL, "\n");

	// include default plugins 
	SCIP_CALL(SCIPincludeDefaultPlugins(scip));

	// set verbosity parameter 
	SCIP_CALL(SCIPsetIntParam(scip, "display/verblevel", 5));
	// SCIP_CALL( SCIPsetBoolParam(scip, "display/lpinfo", TRUE) );

	// disable scip output to stdout
	SCIPmessagehdlrSetQuiet(SCIPgetMessagehdlr(scip), TRUE);

	// set runtime limit
	double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
	SCIP_CALL(SCIPsetRealParam(scip, "limits/time", time_limit - runtime));

	// create empty problem 
	SCIP_CALL(SCIPcreateProb(scip, "WDG", NULL, NULL, NULL, NULL, NULL, NULL, NULL));

	// set the objective senseif necessary, default is minimize
	// SCIP_CALL(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));

	// add variables 
	vector<SCIP_VAR*> vars(N);
	for (int i = 0; i < N; ++i)
	{
		std::string s = std::to_string(i);
		char const* pchar = s.c_str();  //use char const* as target type
		SCIP_CALL(SCIPcreateVar(scip,
								&vars[i],                   // returns new index
								pchar,               // name
								0,                    // lower bound
								node_max_value[i] + 1,                    // upper bound
								1,             // objective
								SCIP_VARTYPE_INTEGER,   // variable type
								true,                   // initial
								false,                  // forget the rest ...
								NULL, NULL, NULL, NULL, NULL));
		SCIP_CALL(SCIPaddVar(scip, vars[i]));
	}

	// add constraints 
	list<SCIP_CONS*> cons;
	for (int i = 0; i < N; i++)
	{
		for (int j = i + 1; j < N; j++)
		{
			if (CG[i * N + j] > 0)
			{
				SCIP_CONS* con;
				SCIP_VAR* var[2] = { vars[i], vars[j] };
				double coeff[2] = { 1, 1 };
				std::string s = std::to_string(i * N + j);
				char const* pchar = s.c_str();  //use char const* as target type
				SCIP_CALL(SCIPcreateConsLinear(scip, &con,
											   pchar,                    // name of the constraint
											   2,                            // number of variables to be added to the constraint
											   var,                        // array of SCIP_VAR pointers to variables
											   coeff,                    // array of values of the coeffcients
											   CG[i * N + j],    // lhs
											   SCIPinfinity(scip),                    // rhs
											   true,                   // initial: set this to TRUE if you want the constraint to occur in the root problem
											   true,                  // separate: set this to TRUE if you would like the handler to separate, e.g. generate cuts
											   true,                   // enforce: set this to TRUE if you would like the handler to enforce solutions. This means that when the handler declares an LP or pseudo solution as infeasible, it can resolve infeasibility by adding cuts, reducing the domain of a variable, performing a branching, etc.
											   true,                   // check: set this to TRUE if the constraint handler should check solutions
											   true,                   // propagate: set this to TRUE if you want to propagate solutions, this means tighten variables domains based on constraint information
											   false,                  // local: set this to TRUE if the constraint is only locally valid, e.g., generated in a branch and bound node
											   false,                   // modifiable: set this to TRUE if the constraint may be modified during solution process, e.g. new variables may be added (colum generation)
											   false,                  // dynamic: set this to TRUE if this constraint is subject to aging, this means it will be removed after being inactive for a while (you should also say TRUE to removable in that case) removable set this to TRUE to allow the deletion of the relaxation of the constraint from the LP
											   false,                  // removable
											   false));               // stickingatnode: set this to TRUE if you want the constraint to be kept at the node it was added

				// add the vars belonging to field in this row to the constraint
				// SCIP_CALL(SCIPaddCoefLinear(scip, con, vars[i], 1));
				// SCIP_CALL(SCIPaddCoefLinear(scip, con, vars[j], 1));
				// add the constraint to scip
				SCIP_CALL(SCIPaddCons(scip, con));
				cons.push_back(con);
			}
		}
	}

	// Solve 

	SCIP_CALL(SCIPsolve(scip));

	// Statistics
	//SCIP_CALL(SCIPprintStatistics(scip, NULL));
	//SCIP_CALL(SCIPprintBestSol(scip, NULL, FALSE));
	// get the best found solution from scip
	SCIP_SOL* sol = SCIPgetBestSol(scip);
	int rst = -1;
	if (sol != NULL) // solved successfully
	{
		rst = SCIPgetSolOrigObj(scip, sol);
	}

	// Deinitialization

	// release variables
	for (auto var: vars)
	{
		SCIP_CALL(SCIPreleaseVar(scip, &var));
	}
	for (auto con : cons)
	{
		SCIP_CALL(SCIPreleaseCons(scip, &con));
	}

	SCIP_CALL(SCIPfree(&scip));

	BMScheckEmptyMemory();
	return rst;
}
*/

// CPLEX
int CBSHeuristic::ILPForWMVC(const vector<int>& CG, const vector<int>& node_max_value)
{
// 	int N = (int)node_max_value.size();
// 	IloEnv env = IloEnv();
// 	IloModel model = IloModel(env);
// 	IloExpr sum_obj = IloExpr(env);
// 	IloNumVarArray var(env);
// 	IloRangeArray con(env);
// 	for (int i = 0; i < N; i++)
// 	{
// 		var.add(IloNumVar(env, 0, node_max_value[i] + 1, ILOINT));
// 		sum_obj += var[i];
// 	}
// 	model.add(IloMinimize(env, sum_obj));
// 	for (int i = 0; i < N; i++)
// 	{
// 		for (int j = i + 1; j < N; j++)
// 		{
// 			if (CG[i * N + j] > 0)
// 			{
// 				con.add(var[i] + var[j] >= CG[i * N + j]);
// 			}
// 		}
// 	}
// 	model.add(con);
// 	IloCplex cplex(env);
// 	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
// 	cplex.setParam(IloCplex::TiLim, time_limit - runtime);
// 	int solution_cost = -1;
// 	cplex.extract(model);
// 	cplex.setOut(env.getNullStream());
// 	int rst = 0;
// 	if (cplex.solve())
// 		rst = (int)cplex.getObjValue();
// 	else
// 	{
// 		runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
// 		if (time_limit > runtime + 1)
// 		{
// 			std::cout << "ERROR! remaining time = " << time_limit - runtime << " seconds" << endl;
// 			cplex.exportModel("error.lp");
// 			system("pause");
// 		}
// 	}
// 	env.end();
// 	return rst;
//
  return 0;
}
