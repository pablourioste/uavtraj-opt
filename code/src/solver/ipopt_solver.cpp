#include "ofc/solver.hpp"

#include <chrono>
#include <cmath>

#ifdef OFC_HAVE_IPOPT
#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#endif

namespace ofc::nlp {

Options Options::fromJson(const nlohmann::json& j) {
    Options o;
    o.tol = j.value("tol", o.tol);
    o.acceptableTol = j.value("acceptable_tol", o.acceptableTol);
    o.constrViolTol = j.value("constr_viol_tol", o.constrViolTol);
    o.maxIter = j.value("max_iter", o.maxIter);
    o.maxCpuTime = j.value("max_cpu_time_s", o.maxCpuTime);
    o.printLevel = j.value("print_level", o.printLevel);
    o.muStrategy = j.value("mu_strategy", o.muStrategy);
    o.lbfgsHistory = j.value("lbfgs_history", o.lbfgsHistory);
    o.linearSolver = j.value("linear_solver", o.linearSolver);
    o.derivativeTest = j.value("derivative_test", o.derivativeTest);
    return o;
}

const char* toString(Status s) {
    switch (s) {
    case Status::Solved:
        return "solved";
    case Status::Acceptable:
        return "acceptable";
    case Status::Infeasible:
        return "infeasible";
    case Status::MaxIterations:
        return "max_iter";
    case Status::MaxTime:
        return "max_time";
    case Status::Failed:
        return "failed";
    case Status::NotAvailable:
        return "no_solver";
    }
    return "?";
}

#ifdef OFC_HAVE_IPOPT

namespace {

using Ipopt::Index;
using Ipopt::Number;

bool finite(const Vec& v) {
    for (double x : v)
        if (!std::isfinite(x)) return false;
    return true;
}

// TNLP adapter. IPOPT sees y = z / s; every callback maps back to z.
class Adapter : public Ipopt::TNLP {
public:
    Adapter(const Problem& p, Result& r) : p_(p), r_(r), s_(p.variableScale()) {
        const Vec z0 = p_.initialPoint();
        for (const Triplet& t : p_.constraintJacobian(z0)) rows_.push_back(t.row), cols_.push_back(t.col);
    }

    bool get_nlp_info(Index& n, Index& m, Index& nnzJac, Index& nnzHess, IndexStyleEnum& style) override {
        n = p_.numVariables();
        m = p_.numConstraints();
        nnzJac = static_cast<Index>(rows_.size());
        nnzHess = 0; // limited-memory Hessian
        style = C_STYLE;
        return true;
    }

    bool get_bounds_info(Index n, Number* yl, Number* yu, Index m, Number* gl, Number* gu) override {
        Vec zl, zu, cl, cu;
        p_.variableBounds(zl, zu);
        p_.constraintBounds(cl, cu);
        for (Index j = 0; j < n; ++j) {
            yl[j] = zl[j] <= -kInf ? -kInf : zl[j] / s_[j];
            yu[j] = zu[j] >= kInf ? kInf : zu[j] / s_[j];
        }
        for (Index i = 0; i < m; ++i) gl[i] = cl[i], gu[i] = cu[i];
        return true;
    }

    bool get_starting_point(Index n, bool initX, Number* y, bool, Number*, Number*, Index, bool, Number*) override {
        if (!initX) return false;
        const Vec z0 = p_.initialPoint();
        for (Index j = 0; j < n; ++j) y[j] = z0[j] / s_[j];
        return true;
    }

    bool eval_f(Index n, const Number* y, bool, Number& f) override {
        f = p_.cost(toZ(n, y));
        return std::isfinite(f);
    }

    bool eval_grad_f(Index n, const Number* y, bool, Number* grad) override {
        const Vec g = p_.costGradient(toZ(n, y));
        for (Index j = 0; j < n; ++j) grad[j] = g[j] * s_[j];
        return finite(g);
    }

    bool eval_g(Index n, const Number* y, bool, Index m, Number* g) override {
        const Vec c = p_.constraints(toZ(n, y));
        for (Index i = 0; i < m; ++i) g[i] = c[i];
        return finite(c);
    }

    bool eval_jac_g(Index n, const Number* y, bool, Index, Index nnz, Index* iRow, Index* jCol,
                    Number* values) override {
        if (!values) {
            for (Index e = 0; e < nnz; ++e) iRow[e] = rows_[e], jCol[e] = cols_[e];
            return true;
        }
        const std::vector<Triplet> J = p_.constraintJacobian(toZ(n, y));
        if (static_cast<Index>(J.size()) != nnz) return false; // structure must not change
        for (Index e = 0; e < nnz; ++e) {
            values[e] = J[e].value * s_[J[e].col];
            if (!std::isfinite(values[e])) return false;
        }
        return true;
    }

    bool intermediate_callback(Ipopt::AlgorithmMode, Index iter, Number, Number, Number, Number, Number, Number, Number,
                               Number, Index, const Ipopt::IpoptData*, Ipopt::IpoptCalculatedQuantities*) override {
        r_.iterations = iter;
        return true;
    }

    void finalize_solution(Ipopt::SolverReturn status, Index n, const Number* y, const Number*, const Number*, Index,
                           const Number*, const Number*, Number obj, const Ipopt::IpoptData*,
                           Ipopt::IpoptCalculatedQuantities*) override {
        r_.z = toZ(n, y);
        r_.objective = obj;
        switch (status) {
        case Ipopt::SUCCESS:
            r_.status = Status::Solved;
            break;
        case Ipopt::STOP_AT_ACCEPTABLE_POINT:
            r_.status = Status::Acceptable;
            break;
        case Ipopt::LOCAL_INFEASIBILITY:
            r_.status = Status::Infeasible;
            break;
        case Ipopt::MAXITER_EXCEEDED:
            r_.status = Status::MaxIterations;
            break;
        case Ipopt::CPUTIME_EXCEEDED:
            r_.status = Status::MaxTime;
            break;
        default:
            r_.status = Status::Failed;
            break;
        }
    }

private:
    Vec toZ(Index n, const Number* y) const {
        Vec z(n);
        for (Index j = 0; j < n; ++j) z[j] = y[j] * s_[j];
        return z;
    }

    const Problem& p_;
    Result& r_;
    Vec s_;
    std::vector<Index> rows_, cols_;
};

const char* returnName(Ipopt::ApplicationReturnStatus rc) {
    switch (rc) {
    case Ipopt::Solve_Succeeded:
        return "Solve_Succeeded";
    case Ipopt::Solved_To_Acceptable_Level:
        return "Solved_To_Acceptable_Level";
    case Ipopt::Infeasible_Problem_Detected:
        return "Infeasible_Problem_Detected";
    case Ipopt::Search_Direction_Becomes_Too_Small:
        return "Search_Direction_Becomes_Too_Small";
    case Ipopt::Diverging_Iterates:
        return "Diverging_Iterates";
    case Ipopt::User_Requested_Stop:
        return "User_Requested_Stop";
    case Ipopt::Feasible_Point_Found:
        return "Feasible_Point_Found";
    case Ipopt::Maximum_Iterations_Exceeded:
        return "Maximum_Iterations_Exceeded";
    case Ipopt::Restoration_Failed:
        return "Restoration_Failed";
    case Ipopt::Error_In_Step_Computation:
        return "Error_In_Step_Computation";
    case Ipopt::Maximum_CpuTime_Exceeded:
        return "Maximum_CpuTime_Exceeded";
    case Ipopt::Not_Enough_Degrees_Of_Freedom:
        return "Not_Enough_Degrees_Of_Freedom";
    case Ipopt::Invalid_Problem_Definition:
        return "Invalid_Problem_Definition";
    case Ipopt::Invalid_Option:
        return "Invalid_Option";
    case Ipopt::Invalid_Number_Detected:
        return "Invalid_Number_Detected";
    default:
        return "Ipopt_Error";
    }
}

} // namespace

bool available() { return true; }

Result solve(const Problem& p, const Options& opt) {
    const auto t0 = std::chrono::steady_clock::now();
    Result r;
    r.z = p.initialPoint();
    r.status = Status::Failed;

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    Ipopt::SmartPtr<Ipopt::OptionsList> o = app->Options();
    o->SetIntegerValue("print_level", opt.printLevel);
    o->SetStringValue("sb", "yes");
    o->SetStringValue("hessian_approximation", "limited-memory");
    o->SetIntegerValue("limited_memory_max_history", opt.lbfgsHistory);
    o->SetNumericValue("tol", opt.tol);
    o->SetNumericValue("acceptable_tol", opt.acceptableTol);
    o->SetNumericValue("constr_viol_tol", opt.constrViolTol);
    o->SetIntegerValue("max_iter", opt.maxIter);
    o->SetNumericValue("max_cpu_time", opt.maxCpuTime);
    o->SetStringValue("mu_strategy", opt.muStrategy);
    if (!opt.linearSolver.empty()) o->SetStringValue("linear_solver", opt.linearSolver);
    if (opt.derivativeTest) o->SetStringValue("derivative_test", "first-order");

    if (app->Initialize() != Ipopt::Solve_Succeeded) {
        r.message = "IPOPT initialisation failed";
        return r;
    }
    Ipopt::SmartPtr<Ipopt::TNLP> tnlp = new Adapter(p, r);
    const Ipopt::ApplicationReturnStatus rc = app->OptimizeTNLP(tnlp);
    r.message = returnName(rc);
    if (rc == Ipopt::Maximum_CpuTime_Exceeded) r.status = Status::MaxTime;
    r.millis = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return r;
}

#else // built without IPOPT

bool available() { return false; }

Result solve(const Problem& p, const Options&) {
    Result r;
    r.z = p.initialPoint();
    r.status = Status::NotAvailable;
    r.message = "built without IPOPT (configure with -DOFC_IPOPT=ON on a system that has it)";
    return r;
}

#endif

} // namespace ofc::nlp
