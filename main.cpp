#include <iostream>
#include <ilcplex/ilocplex.h>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <math.h>
using namespace std;

int main()
{
    // Create data
        // Suppose that there are 12 demand nodes, and root node is located at node 0
    int numDemandNodes = 12, rootNode = 0, numNodes = numDemandNodes+1;
        // Suppose that there are 5 trucks
    int numTrucks = 5;
        // Capacity each truck
    vector<double> C(numTrucks);
    for(int i=0; i<numTrucks; i++) {
        C[i] = 15 + rand()%10;
    }

    srand(time(NULL));
    int xPos[numNodes], yPos[numNodes];
    for(int i=0; i<numNodes; i++) {
        xPos[i] = rand()%100;
        yPos[i] = rand()%100;
    }

    vector<vector<double>> cost(numNodes, vector<double>(numNodes));
    for(int i=0; i<numNodes; i++) {
        for(int j=0; j<numNodes; j++) {
            cost[i][j] = sqrt(pow(xPos[j] - xPos[i], 2) + pow(yPos[j] - yPos[i], 2));
        }
    }

    // demand of nodes
    vector<double> d(numNodes);
    for(int i=0; i<numNodes; i++) {
        if(i != rootNode) {
            d[i] = 1 + rand()%10;
        }
    }
    IloEnv env;

    try{
        IloModel model(env);


        /*------DECISION VARIABLE--------*/
        IloArray<IloArray<IloNumVarArray>> x(env, numNodes);
        for(int i=0; i<numNodes; i++) {
            x[i] = IloArray<IloNumVarArray>(env, numNodes);
            for(int j=0; j<numNodes; j++) {
                x[i][j] = IloNumVarArray(env, numTrucks);
                for(int k=0; k<numTrucks; k++) {
                    x[i][j][k] = IloNumVar(env, 0, 1, ILOBOOL);
                }
            }
        }



        /*---------OBJECTIVE FUNTION--------*/
        IloExpr obj(env);
        for(int i=0; i<numNodes; i++) {
            for(int j=0; j<numNodes; j++) {
                for(int k=0; k<numTrucks; k++) {
                    obj += x[i][j][k]*cost[i][j];
                }
            }
        }
        model.add(IloMinimize(env, obj));
        obj.end();


        /*----------CONSTRAINTS-------------*/
        IloExpr expr(env), expr2(env);
        for(int i=0; i<numNodes; i++) {
            for(int k=0; k<numTrucks; k++) {
                model.add(x[i][i][k] == 0);
            }
        }

        for(int i=0; i<numNodes; i++) {
            if(i != rootNode) {
                for(int j=0; j<numNodes; j++) {
                    for(int k=0; k<numTrucks; k++) {
                        expr += x[i][j][k];
                        expr2 += x[j][i][k];
                    }
                }
                model.add(expr == 1);
                model.add(expr2 == 1);
                expr.clear();
                expr2.clear();
            }
        }

        for(int i=0; i<numNodes; i++) {
            for(int k=0; k<numTrucks; k++) {
                expr += x[rootNode][i][k];
                expr2 += x[i][rootNode][k];
            }
        }
        model.add(expr == numTrucks);
        model.add(expr2 == numTrucks);
        expr.clear();
        expr2.clear();

        for(int i=0; i<numNodes; i++) {
            for(int k=0; k<numTrucks; k++) {
                for(int j=0; j<numNodes; j++) {
                    expr += x[i][j][k];
                    expr2 += x[j][i][k];
                }
                model.add(expr == expr2);
                expr.clear();
                expr2.clear();
            }
        }

        // f[i][j][k] denotes the load on the truck k from i to j
        IloArray<IloArray<IloNumVarArray>> f(env, numNodes);
        for(int i=0; i<numNodes; i++) {
            f[i] = IloArray<IloNumVarArray>(env, numNodes);
            for(int j=0; j<numNodes; j++) {
                f[i][j] = IloNumVarArray(env, numTrucks);
                for(int k=0; k<numTrucks; k++) {
                    f[i][j][k] = IloNumVar(env, 0, C[k], ILOINT);
                }
            }
        }

        for(int i=0; i<numNodes; i++) {
            if(i != rootNode)  {
                for(int j=0; j<numNodes; j++) {
                    for(int k=0; k<numTrucks; k++) {
                        expr += f[i][j][k];
                        expr2 += f[j][i][k];
                    }
                }
                model.add(expr2-expr == d[i]);
                expr.clear();
                expr2.clear();
            }
        }

        for(int i=0; i<numNodes; i++) {
            for(int j=0; j<numNodes; j++) {
                for(int k=0; k<numTrucks; k++) {
                    model.add(f[i][j][k] <= C[k]*x[i][j][k]);
                }
            }
        }


        // solve and print solution
        IloCplex cplex(model);
        cplex.solve();

        double minSumCost = cplex.getObjValue();
        cout<< "The minimum cost is "<< minSumCost<< "\n";
        cout<< "The path is:"<< "\n";
        for(int i=0; i<numNodes; i++) {
            for(int k=0; k<numTrucks; k++) {
                if(cplex.getValue(x[rootNode][i][k]) == 1) {
                    cout<< rootNode<< "->"<< i;
                    int j=i;
                    while(j != rootNode) {
                        for(int f=0; f<numNodes; f++) {
                            if(cplex.getValue(x[j][f][k]) == 1) {
                                cout<< "->"<< f;
                                j=f;
                                break;
                            }
                        }
                    }
                    cout<< "\n";
                    break;
                }
            }
        }

        expr.end();
        expr2.end();


    }


    catch (IloException& e){
        cerr << "Conver exception caught: " << e << endl; // No solution exists
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }

    env.end();


    return 0;
}











