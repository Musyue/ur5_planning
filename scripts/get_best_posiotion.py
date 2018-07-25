import numpy as np
import sys
from new_kinematics import Kinematic
def best_sol(sols, q_guess, weights):
    valid_sols = []
    for sol in sols:
        test_sol = np.ones(6)*9999.
        for i in range(6):
            for add_ang in [-2.*np.pi, 0, 2.*np.pi]:
                print add_ang
                test_ang = sol[i] + add_ang
                if (abs(test_ang) <= 2.*np.pi and
                    abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
                    test_sol[i] = test_ang
        if np.all(test_sol != 9999.):
            valid_sols.append(test_sol)
    if len(valid_sols) == 0:
        return None
    best_sol_ind = np.argmin(np.sum((weights*(valid_sols - np.array(q_guess)))**2,1))
    return valid_sols[best_sol_ind]
def main():
    q = [0, 0, 1, 0, 1, 0]
    q2 = [-0.25277, -0.8561733333333333, 1.2195411111111112, -3.557096666666667, -1.3205444444444445,-1.1349355555555556]
    c=Kinematic(q2)
    T=c.Forward()
    num_goals,sols=c.Iverse(T,0)
    sols1=c.display(0,3)
    qsol = best_sol(sols1, q2, [1.] * 6)
    print "##################################qsol",qsol
if __name__ == '__main__':
    main()