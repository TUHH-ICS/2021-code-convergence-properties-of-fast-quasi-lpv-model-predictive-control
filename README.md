# Convergence Properties of Fast quasi-LPV Model Predictive Control

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4633758.svg)](https://doi.org/10.5281/zenodo.4633758)

## General

This repository contains the simulation code to reproduce the tables and figures presented in

> C. Hespe and H. Werner, "Convergence Properties of Fast quasi-LPV Model Predictive Control," *60th Conference on Decision and Control*. Austin, TX, USA, 2021, doi: [10.1109/CDC45484.2021.9683612](https://doi.org/10.1109/CDC45484.2021.9683612).

The code has two main entry points, the scripts `benchmark_adip.m` and `benchmark_unicycle.m`, both in the root directory of this repository.
They will produce among others the material presented in the paper.

## Prerequisites

To run the simulation files, some additional packages need to be installed first.

1. Install *acados*, e.g. according to [this guide](https://docs.acados.org/installation/index.html#windows-for-use-with-matlab)
2. Install [*CasADi*](https://web.casadi.org/) into the `external` directory of *acados*, as described on the *acados* [interface page](https://docs.acados.org/matlab_octave_interface/index.html#setup-casadi)
3. Create the environment variable `ACADOS_INSTALL_DIR` pointing to the main *acados* directory

The simulation code in this repository was tested in the following environment:

* *Windows 10* Version 21H1
* *Matlab* 2021a
* *acados* [0.1.6](https://github.com/acados/acados/releases/tag/0.1.6)
* *CasADi* 3.5.5

**In the default configuration, you will also need the *Matlab Coder*.**
It is used to generate C code for the qLMPC solvers.
If you do not have access to *Matlab Coder* and want to run the simulation nonetheless, you can disable code generation in the main simulation scripts by setting `generate_mex = false`.
