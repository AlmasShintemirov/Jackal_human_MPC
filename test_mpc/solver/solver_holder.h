
class MPC_solver
{
private:
  int num_steps;
public:
  MPC_solver(int n);
  int reinitialize();
  double * solve_mpc(double input_arr[7], double online_arr[600], double target_arr[7]);
};


