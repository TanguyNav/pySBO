import time
import numpy as np
import sys

import pathlib
import importlib

# SOFA Libraries
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../../../predictingcompliance")
import Sofa
import SofaRuntime
SofaRuntime.importPlugin("SofaPython3")
from Sofa import SofaConstraintSolver

from Problems.Box_Constrained import Box_Constrained


#-------------------------------------#
#-------------class SOFA-------------#
#-------------------------------------#
class SOFA_Compliance(Box_Constrained):

    #-----------------------------------------#
    #-------------special methods-------------#
    #-----------------------------------------#

    #-------------__init__-------------#    
    def __init__(self, model_name="Trunk"):
        assert type(model_name)==str

        self.__model_name = model_name
        self.__config = self.init_config() # The config object describing a Soft Robot optimization problem in the predicting_compliance project
        self.init_Box_Constrained()

        # self.__param_set = read_list_of_param_sets_from_csv(self.__country)[0]
        # self.__root_model = run_root_model(self.__country, self.__param_set)

    #-------------__del__-------------#
    def __del__(self):
        Box_Constrained.__del__(self)
        del self.__model_name
        del self.__config
        # del self.__param_set
        # del self.__root_model

    #-------------__str__-------------#
    def __str__(self):
        return "SOFT Robot compliance prediction problem:\n  model_name: " + self.__model_name

    #---------------------------------------------#
    #-------------getters and setters-------------#
    #---------------------------------------------#
    
    #-------------_get_model_name-------------#
    def _get_model_name(self):
        return self.__model_name

    #-------------_set_model_name-------------#
    def _set_model_name(self,new_model_name):
        pass

    #-------------_del_model_name-------------#
    def _del_model_name(self):
        pass

    #-------------_get_config-------------#
    def _get_config(self):
        return self.__config

    #-------------_set_config-------------#
    def _set_config(self,new_config):
        pass

    #-------------_del_config-------------#
    def _del_config(self):
        pass
        
    #-----------------------------------------#
    #-------------linking methods-------------#
    #-----------------------------------------#
    def init_config(self):
        """ This method load a soft robot config from the "predicting_compliance" project.
        TODO: add requirements on folder localizations. """
        config_lib = importlib.import_module("Models."+ self.__model_name +".Config")
        Config = config_lib.Config()
        # TODO: add special treatment of config file
        return Config

    def init_Box_Constrained(self):
        n_dvar = len(self.__config.get_actuators_variables()) # Get number of decision variable
        #n_dvar = len(self.__config.get_actuators_variables()) + len(self.__config.get_design_variables()) 
        Box_Constrained.__init__(self, n_dvar, 1) 

    #----------------------------------------#
    #-------------object methods-------------#
    #----------------------------------------#
    
    #-------------perform_real_evaluation-------------#
    def perform_real_evaluation(self, candidates):
        assert self.is_feasible(candidates)
        if candidates.ndim==1:
            candidates = np.array([candidates])

        costs = np.zeros((candidates.shape[0],))
        
        for i,cand in enumerate(candidates):
            print(cand)
            t_start = time.time()
            loss = self.objective_function(cand)
            t_end = time.time()
            costs[i] = loss
            print(t_end-t_start)
        
        return costs

    def objective_function(self, candidate):
        # Init SOFA scene
        scene_lib = importlib.import_module("Models." + self.__config.model_name + "." + self.__config.model_name)
        root = Sofa.Core.Node("root")
        scene_lib.createScene(root, self.__config)
        Sofa.Simulation.init(root)

        # Simulate
        try:
            # Animate without actuation to reach equilibrium
            null_action = len(self.__config.get_actuators_variables()) * [0]
            root.Controller.apply_actions(null_action)
            for step in range(self.__config.get_n_eq_dt()):
                Sofa.Simulation.animate(root, root.dt.value)
                time.sleep(root.dt.value)
            W_0 = root.Controller.get_compliance_matrice_in_constraint_space()
            dfree_0 = root.Controller.get_dfree()

            # Apply action
            interpolated_action = self.__config.interpolate_actuation(candidate)
            for step in range(self.__config.get_n_dt()):
                interpolated_action_step = [min((step + 1) * v/(self.__config.get_n_dt() - 10), v) for v in interpolated_action] # Apply gradually action
                root.Controller.apply_actions(interpolated_action_step)
                Sofa.Simulation.animate(root, root.dt.value)
                time.sleep(root.dt.value)

            actuation_state = root.Controller.get_actuators_state()
            effectors_state = root.Controller.get_effectors_state() # By default it is null
            W = root.Controller.get_compliance_matrice_in_constraint_space()
            dfree = root.Controller.get_dfree()
            print(">> Simulation done")

            # Reset simulation
            Sofa.Simulation.reset(root) 

        except:
            print("[ERROR] >> The scene did crash. We may penalize this set of parameters.")
            return 100000

        return 3 

    #-------------get_bounds-------------#
    def get_bounds(self):
        pass
        # bounds=np.ones((2,self.n_dvar))
        # bounds[0,:]*=0.0
        # bounds[1,:]*=1.0
        # return bounds

    #-------------is_feasible-------------#
    def is_feasible(self, candidate):
        return True
        # feasible=False
        # if Box_Constrained.is_feasible(self, candidate)==True:
        #     lower_bounds=self.get_bounds()[0,:]
        #     upper_bounds=self.get_bounds()[1,:]
        #     feasible=(lower_bounds<=candidate).all() and (candidate<=upper_bounds).all()
        # return feasible

