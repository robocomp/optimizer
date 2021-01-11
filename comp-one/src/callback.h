//
// Created by pbustos on 11/1/21.
//

#ifndef COMPONE_CALLBACK_H
#define COMPONE_CALLBACK_H

#include "gurobi_c++.h"

class Callback: public GRBCallback
{
    public:
        double lastiter;
        double lastnode;
        int numvars;
        GRBVar* vars;
        Callback(int xnumvars, GRBVar* xvars)
        {
            lastiter = lastnode = -GRB_INFINITY;
            numvars = xnumvars;
            vars = xvars;
        }
    protected:
        Callback ()
        {
            try {
                if (where == GRB_CB_POLLING) {
                    // Ignore polling callback
                } else if (where == GRB_CB_PRESOLVE) {
                    // Presolve callback
                    int cdels = getIntInfo(GRB_CB_PRE_COLDEL);
                    int rdels = getIntInfo(GRB_CB_PRE_ROWDEL);
                    if (cdels || rdels) {
                        cout << cdels << " columns and " << rdels
                             << " rows are removed" << endl;
                    }
                } else if (where == GRB_CB_SIMPLEX) {
                    // Simplex callback
                    double itcnt = getDoubleInfo(GRB_CB_SPX_ITRCNT);
                    if (itcnt - lastiter >= 100) {
                        lastiter = itcnt;
                        double obj = getDoubleInfo(GRB_CB_SPX_OBJVAL);
                        int ispert = getIntInfo(GRB_CB_SPX_ISPERT);
                        double pinf = getDoubleInfo(GRB_CB_SPX_PRIMINF);
                        double dinf = getDoubleInfo(GRB_CB_SPX_DUALINF);
                        char ch;
                        if (ispert == 0)      ch = ' ';
                        else if (ispert == 1) ch = 'S';
                        else                  ch = 'P';
                        cout << itcnt << " " << obj << ch << " "
                             << pinf << " " << dinf << endl;
                    }
                } else if (where == GRB_CB_MIP) {
                    // General MIP callback
                    double nodecnt = getDoubleInfo(GRB_CB_MIP_NODCNT);
                    double objbst = getDoubleInfo(GRB_CB_MIP_OBJBST);
                    double objbnd = getDoubleInfo(GRB_CB_MIP_OBJBND);
                    int solcnt = getIntInfo(GRB_CB_MIP_SOLCNT);
                    if (nodecnt - lastnode >= 100) {
                        lastnode = nodecnt;
                        int actnodes = (int) getDoubleInfo(GRB_CB_MIP_NODLFT);
                        int itcnt = (int) getDoubleInfo(GRB_CB_MIP_ITRCNT);
                        int cutcnt = getIntInfo(GRB_CB_MIP_CUTCNT);
                        cout << nodecnt << " " << actnodes << " " << itcnt
                             << " " << objbst << " " << objbnd << " "
                             << solcnt << " " << cutcnt << endl;
                    }
                    if (fabs(objbst - objbnd) < 0.1 * (1.0 + fabs(objbst))) {
                        cout << "Stop early - 10% gap achieved" << endl;
                        abort();
                    }
                    if (nodecnt >= 10000 && solcnt) {
                        cout << "Stop early - 10000 nodes explored" << endl;
                        abort();
                    }
                } else if (where == GRB_CB_MIPSOL) {
                    // MIP solution callback
                    int nodecnt = (int) getDoubleInfo(GRB_CB_MIPSOL_NODCNT);
                    double obj = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
                    int solcnt = getIntInfo(GRB_CB_MIPSOL_SOLCNT);
                    double* x = getSolution(vars, numvars);
                    cout << "**** New solution at node " << nodecnt
                         << ", obj " << obj << ", sol " << solcnt
                         << ", x[0] = " << x[0] << " ****" << endl;
                    delete[] x;
                } else if (where == GRB_CB_MIPNODE) {
                    // MIP node callback
                    cout << "**** New node ****" << endl;
                    if (getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL) {
                        double* x = getNodeRel(vars, numvars);
                        setSolution(vars, x, numvars);
                        delete[] x;
                    }
                } else if (where == GRB_CB_BARRIER) {
                    // Barrier callback
                    int itcnt = getIntInfo(GRB_CB_BARRIER_ITRCNT);
                    double primobj = getDoubleInfo(GRB_CB_BARRIER_PRIMOBJ);
                    double dualobj = getDoubleInfo(GRB_CB_BARRIER_DUALOBJ);
                    double priminf = getDoubleInfo(GRB_CB_BARRIER_PRIMINF);
                    double dualinf = getDoubleInfo(GRB_CB_BARRIER_DUALINF);
                    double cmpl = getDoubleInfo(GRB_CB_BARRIER_COMPL);
                    cout << itcnt << " " << primobj << " " << dualobj << " "
                         << priminf << " " << dualinf << " " << cmpl << endl;
                } else if (where == GRB_CB_MESSAGE) {
                    // Message callback
                    string msg = getStringInfo(GRB_CB_MSG_STRING);
                }
            } catch (GRBException e) {
                cout << "Error number: " << e.getErrorCode() << endl;
                cout << e.getMessage() << endl;
            } catch (...) {
                cout << "Error during callback" << endl;
            }
        }
};

#endif //COMPONE_CALLBACK_H
