## Convergence Properties of Fast quasi-LPV Model Predictive Control

This repository contains the simulation code corresponding to the aforementioned paper.
The code has two main entry points, the scripts `benchmark_adip.m` and `benchmark_unicycle.m`, both in the root directory of this repository.

### Prerequisites

To run the simulation files, some additional packages need to be installed first.
1. Install *acados* according to [this guide](https://docs.acados.org/installation/index.html#windows-for-use-with-matlab)
2. Install [*CasADi*](https://web.casadi.org/) into the `external` directory of *acados*, as described on the *acados* [installation page](https://docs.acados.org/interfaces/index.html#download-casadi)
3. Add the *CasADi* directory to your Matlab path
4. Create the environment variable `ACADOS_INSTALL_DIR` pointing to the main *acados* directory

The simulation code in this repository was tested in the following environment:
* *Windows 10* Version 2004
* *Matlab* 2020b
* *acados* 0.1.4 ([Commit cd29492b](https://github.com/acados/acados/tree/cd29492b))
* *CasADi* 3.5.5

**In the default configuration, you will also need the *Matlab Coder*.**
It is used to generate C code for the qLMPC solvers.
If you do not have access to *Matlab Coder* and want to run the simulation nonetheless, you can disable code generation in the main simulation scripts by setting `generate_mex = false`.
