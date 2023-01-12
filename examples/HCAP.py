"""Script running the parallel Hybrid Concurrent Acquition Process for single-objective optimization.

The Hybrid Concurrent Acquition Process is described in:
`G. Briffoteaux. Parallel surrogate-based algorithms for solving expensive optimization problems. Thesis. 2022. <https://hal.science/tel-03853862>`_

To run sequentially: ``python3.9 ./HCAP.py``

To run in parallel (in 4 computational units): ``mpiexec -n 4 python3.9 HCAP.py``

To run in parallel (in 4 computational units) specifying the units in `./hosts.txt`: ``mpiexec --machinefile ./host.txt -n 4 python3.9 HCAP.py``
"""

import sys
sys.path.append('../src')
import os
import time
import numpy as np
from mpi4py import MPI

from Problems.Schwefel import Schwefel
from Problems.DoE import DoE

from Evolution.Population import Population
from Evolution.Tournament import Tournament
from Evolution.Tournament_Position import Tournament_Position
from Evolution.SBX import SBX
from Evolution.Polynomial import Polynomial
from Evolution.Elitist import Elitist
from Evolution.Custom_Elitism import Custom_Elitism

from Surrogates.BNN_MCD import BNN_MCD
from Surrogates.GP import GP

from Evolution_Controls.POV_EC import POV_EC
from Evolution_Controls.Distance_EC import Distance_EC
from Evolution_Controls.Pareto_EC import Pareto_EC
from Evolution_Controls.Expected_Improvement_EC import Expected_Improvement_EC

from Global_Var import *


def main():

    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    nprocs = comm.Get_size()


    #--------------------------------#
    #-------------MASTER-------------#
    #--------------------------------#
    if rank==0:
        
        # Problem
        p = Schwefel(16)

        # Files
        DIR_STORAGE = "./outputs/"
        os.system("mkdir "+DIR_STORAGE)
        os.system("rm -rf "+DIR_STORAGE+"/*")
        F_SIM_ARCHIVE = DIR_STORAGE+"sim_archive.csv"
        F_TRAIN_LOG_BNN = DIR_STORAGE+"training_log_BNN.csv"
        F_TRAIN_LOG_GP = DIR_STORAGE+"training_log_GP.csv"
        F_TRAINED_MODEL_BNN = DIR_STORAGE+"trained_model_BNN"
        F_TRAINED_MODEL_GP = DIR_STORAGE+"trained_model_GP"
        F_TMP_DB=DIR_STORAGE+"tmp_db.csv"
        F_BEST_PROFILE = DIR_STORAGE+"best_profile.csv"
        F_INIT_POP = DIR_STORAGE+"init_pop.csv"
    
        # Search arguments
        TIME_BUDGET = 0
        N_GEN = 1
        SIM_TIME = 15
        if TIME_BUDGET > 0:
            assert TIME_BUDGET > SIM_TIME
            N_GEN = 1000000000000
        # Search arguments for SAEA
        POP_SIZE = 72
        N_CHLD = 288 # number of children issued per generation
        N_SIM = 63
        N_DISC = 225
        if N_CHLD!=N_SIM+N_DISC:
            print("[HCAP.py] number of children in SAEA does not match number of simulations and discardings")
            assert False        
        # Search arguments for q-EGO
        q=9
        qPOP_SIZE=50
        qN_GEN=100
        qN_CHLD=POP_SIZE

        # Database initialization / Parallel DoE
        sampler = DoE(p)
        pop = Population(p)
        pop.dvec = sampler.latin_hypercube_sampling(POP_SIZE)
        nb_sim_per_proc = (POP_SIZE//nprocs)*np.ones((nprocs,), dtype=int) # number of simulations per proc
        for i in range(POP_SIZE%nprocs):
            nb_sim_per_proc[i+1]+=1
        for i in range(1,nprocs): # sending to workers
            comm.send(nb_sim_per_proc[i], dest=i, tag=10)
            comm.send(pop.dvec[np.sum(nb_sim_per_proc[:i]):np.sum(nb_sim_per_proc[:i+1])], dest=i, tag=11)
        pop.obj_vals = np.zeros((pop.dvec.shape[0],))
        pop.obj_vals[0:nb_sim_per_proc[0]] = p.perform_real_evaluation(pop.dvec[0:nb_sim_per_proc[0]])
        for i in range(1,nprocs): # receiving from workers
            pop.obj_vals[np.sum(nb_sim_per_proc[:i]):np.sum(nb_sim_per_proc[:i+1])] = comm.recv(source=i, tag=12)
        pop.fitness_modes = True*np.ones(pop.obj_vals.shape, dtype=bool)

        # Logging
        pop.save_sim_archive(F_SIM_ARCHIVE)
        pop.update_best_sim(F_BEST_PROFILE)
        pop.save_sim_archive(F_TMP_DB)

        # Number of simulations per proc (only procs dedicated to parallel simulations of children generated by informed operators)
        nb_sim_per_proc = ((N_SIM+q)//nprocs)*np.ones((nprocs,), dtype=int)
        for i in range((N_SIM+q)%nprocs):
            nb_sim_per_proc[i+1]+=1

        # Set the constant for Constant Liar
        L = [np.sum(pop.obj_vals), pop.obj_vals.shape[0]]

        # Start chronometer
        if TIME_BUDGET>0:
            t_start = time.time()

        # Creating surrogate
        surr_BNN = BNN_MCD(F_SIM_ARCHIVE, p, float('inf'), F_TRAIN_LOG_BNN, F_TRAINED_MODEL_BNN, 5)
        surr_GP = GP(F_TMP_DB, p, 72, F_TRAIN_LOG_GP, F_TRAINED_MODEL_GP, "rbf")
        surr_BNN.perform_training()
        surr_GP.perform_training()

        # Evolution Controls
        ec_base_y = POV_EC(surr_BNN)
        ec_base_d = Distance_EC(surr_BNN)
        ec_op_1 = Pareto_EC([1.0, 1.0], "cd", ec_base_y, ec_base_d)
        ec_op_2 = Expected_Improvement_EC(surr_GP)

        # Operators
        select_op = Tournament(2)
        crossover_op = SBX(0.9, 10)
        mutation_op = Polynomial(1.0/p.n_dvar, 50)
        replace_op = Elitist()

        # Operators q-EGO
        qselect_op = Tournament_Position(2)
        qreplace_op = Custom_Elitism(ec_op_2)
        
        #----------------------Start Generation loop----------------------#
        for curr_gen in range(N_GEN):
            print("generation "+str(curr_gen))

            # Reproduction operators
            parents = select_op.perform_selection(pop, N_CHLD)
            children = crossover_op.perform_crossover(parents)
            children = mutation_op.perform_mutation(children)
            assert p.is_feasible(children.dvec)

            # Evolution Control
            idx_split = ec_op_1.get_sorted_indexes(children)
            batch_to_simulate = Population(p)
            batch_to_simulate.dvec = children.dvec[idx_split[0:N_SIM]]

            #----------------------q-EGO----------------------#

            q_cands = Population(p)
            q_cands.dvec = np.zeros((q,p.n_dvar))

            for curr_sub_cycle in range(q):

                # Population initialization
                qpop = Population(p)
                qpop.dvec = sampler.latin_hypercube_sampling(qPOP_SIZE)
                qpop.dvec = qpop.dvec[ec_op_2.get_sorted_indexes(qpop)]

                #----------------------Evolution loop
                for qcurr_gen in range(qN_GEN):

                    # Acquisition Process
                    qparents = qselect_op.perform_selection(qpop, qN_CHLD)
                    qchildren = crossover_op.perform_crossover(qparents)
                    qchildren = mutation_op.perform_mutation(qchildren)
                    assert p.is_feasible(qchildren.dvec)

                    # Replacement
                    qreplace_op.perform_replacement(qpop, qchildren)
                    assert p.is_feasible(qpop.dvec)
                    del qchildren
                #----------------------End evolution loop

                # Retaining the best individual
                qpop.obj_vals = np.zeros((qpop.dvec.shape[0],))
                qpop.obj_vals[0:1] = L[0]/L[1]
                qpop.fitness_modes = False*np.ones(qpop.obj_vals.shape, dtype=bool)
                qpop.fitness_modes[0:1] = True
                qpop.save_sim_archive(F_TMP_DB)
                q_cands.dvec[curr_sub_cycle,:] = qpop.dvec[0,:]

                # Surrogate partial update
                if curr_sub_cycle!=q-1:
                    surr_GP.perform_partial_training()

            #----------------------q-EGO----------------------#

            batch_to_simulate.dvec = np.append(batch_to_simulate.dvec, q_cands.dvec, axis=0)

            # Parallel simulations
            for i in range(1,nprocs): # sending to workers
                comm.send(nb_sim_per_proc[i], dest=i, tag=10)
                comm.send(batch_to_simulate.dvec[np.sum(nb_sim_per_proc[:i]):np.sum(nb_sim_per_proc[:i+1])], dest=i, tag=11)
            batch_to_simulate.obj_vals = np.zeros((np.sum(nb_sim_per_proc),))
            batch_to_simulate.obj_vals[0:nb_sim_per_proc[0]] = p.perform_real_evaluation(batch_to_simulate.dvec[0:nb_sim_per_proc[0]])

            for i in range(1,nprocs): # receiving from workers
                batch_to_simulate.obj_vals[np.sum(nb_sim_per_proc[:i]):np.sum(nb_sim_per_proc[:i+1])] = comm.recv(source=i, tag=12)
            batch_to_simulate.dvec = batch_to_simulate.dvec[:np.sum(nb_sim_per_proc)]
            batch_to_simulate.fitness_modes = np.ones(batch_to_simulate.obj_vals.shape, dtype=bool)

            # Logging
            batch_to_simulate.save_sim_archive(F_SIM_ARCHIVE) 
            batch_to_simulate.update_best_sim(F_BEST_PROFILE)
            os.system("cat "+F_SIM_ARCHIVE+" > "+F_TMP_DB)

            # Surrogate update
            surr_BNN.perform_training()
            surr_GP.perform_training()

            # Replacement
            replace_op.perform_replacement(pop, batch_to_simulate)
            assert p.is_feasible(pop.dvec)
            del batch_to_simulate
            del children

            # Exit Generation loop if budget time exhausted
            if TIME_BUDGET>0:
                t_now = time.time()
                if TIME_BUDGET-(t_now-t_start)<SIM_TIME:
                    break
            if curr_gen==N_GEN-1:
                break

        #----------------------End Generation loop----------------------#

        # Stop workers
        for i in range(1,nprocs):
            comm.send(-1, dest=i, tag=10)

    
    #---------------------------------#
    #-------------WORKERS-------------#
    #---------------------------------#
    else:
        nsim = comm.recv(source=0, tag=10)
        while nsim!=-1:
            candidates = np.empty((nsim, p.n_dvar))
            candidates = comm.recv(source=0, tag=11)
            obj_vals = p.perform_real_evaluation(candidates)
            comm.send(obj_vals, dest=0, tag=12)
            nsim = comm.recv(source=0, tag=10)

if __name__ == "__main__":
    main()
