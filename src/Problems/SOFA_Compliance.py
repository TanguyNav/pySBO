import time
import numpy as np
import importlib
import copy

# SOFA Libraries
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
        self.__scene_lib = importlib.import_module("SofaModels." + self.__config.model_name + "." + self.__config.model_name)
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
        config_lib = importlib.import_module("SofaModels."+ self.__model_name +".Config")
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
        """
        Perform a set of simulation evaluation.
        -------
        Inputs:
        -------
            candidates: list o lists of floats
                List of list of float values describing sampling variables. 
        --------
        Outputs:
        --------
            time_costs: list of floats
                Time consumption for evaluating each candidate
            mechanical_matrices = list of dicts of numpy arrays
                List of mechanical matrices obtained from simulated candidates
        """
        assert self.is_feasible(candidates)
        
        if candidates.ndim==1:
            candidates = np.array([candidates])
        
        time_costs = np.zeros((candidates.shape[0],))
        mechanical_matrices = []

        for i,cand in enumerate(candidates):
            t_start = time.time()
            try: 
                w_0, dfree_0, disp_state, w, dfree = self.get_simulated_mechanical_matrices(cand)
                t_end = time.time()
                time_costs[i] = t_end-t_start
                mechanical_matrices.append(
                    {"W_0": w_0, "dfree_0": dfree_0, 
                     "disp_state": disp_state, "W": w, 
                     "dfree": dfree                  
                    }
                )
            except:
                print("Simulation failed. Do something (??)")      

        return time_costs, mechanical_matrices


    def get_simulated_mechanical_matrices(self, sample_vars):
        """
        Perform a simulation evaluation.
        -------
        Inputs:
        -------
            sample_vars: lists of floats
               List of float values describing the sample. 
        --------
        Outputs:
        --------
            w_0, dfree_0, disp_state, w, dfree: numpy arrays
                The mechanical matrices obtained from simulation
                
        """
        if self.__config.is_direct_control_sampling:
            w_0, dfree_0, disp_state, w, dfree = self.direct_simulation(sample_vars)
        else:
            w_0, dfree_0, disp_state, w, dfree = self.inverse_simulation(sample_vars)
        return w_0, dfree_0, disp_state, w, dfree


    def direct_simulation(self, sample_vars):
        # Separate constraint values from design variables
        n_act = len(self.__config.get_actuators_variables())
        n_sample_constraint_size = len(sample_vars) - len(self.__config.get_design_variables())
        sample_actuation = sample_vars[:n_act]
        sample_contact = sample_vars[n_act:n_sample_constraint_size]
        sample_design = sample_vars[n_sample_constraint_size:]

        # Compute real values for constraints
        interpolated_actuation = self.__config.interpolate_variables(sample_actuation, var_type = "actuation")
        interpolated_contact = self.__config.interpolate_variables(sample_contact, var_type = "contact")
        interpolated_constraints = interpolated_actuation + interpolated_contact

        # Update config with real values for contacts
        copy_config = self.__config
        interpolated_design = []
        if len(sample_design) != 0:
            interpolated_design = self.__config.interpolate_variables(sample_design, var_type = "design")
            copy_config.set_design_variables(interpolated_design)

        print("Start Simulation ... >>")

        # Init SOFA scene
        root = Sofa.Core.Node("root")
        self.__scene_lib.createScene(root, copy_config)
        Sofa.Simulation.init(root)

        # Animate without actuation to reach equilibrium
        null_action = (len(self.__config.get_actuators_variables()) + len(self.__config.get_contacts_variables())) * [0]
        root.Controller.apply_actions(null_action)
        for step in range(self.__config.get_n_eq_dt()):
            Sofa.Simulation.animate(root, root.dt.value)
            time.sleep(root.dt.value)
        w_0 = copy.deepcopy(root.Controller.get_compliance_matrice_in_constraint_space())
        dfree_0 = copy.deepcopy(root.Controller.get_dfree())

        # Apply actuation / contact displacement
        for step in range(self.__config.get_n_dt() + self.__config.get_post_sim_n_eq_dt()):
            interpolated_constraints_step = [min((step + 1) * v/(self.__config.get_n_dt()), v) for v in interpolated_constraints] # Apply gradually action
            root.Controller.apply_actions(interpolated_constraints_step)
            Sofa.Simulation.animate(root, root.dt.value)
            time.sleep(root.dt.value)

        actuation_state = root.Controller.get_actuators_state() # Contact points are handled as actuators in the SOFA scene
        effectors_state = root.Controller.get_effectors_state() # By default it is null
        w = copy.deepcopy(root.Controller.get_compliance_matrice_in_constraint_space())
        dfree = copy.deepcopy(root.Controller.get_dfree())
        print(">> ... Simulation done")

        # Reset simulation
        Sofa.Simulation.reset(root)

        return w_0, dfree_0, actuation_state + effectors_state, w, dfree   


    def inverse_simulation(self, sample_vars):
        # TODO: A function for managing special sampling space
        pass
        
    
    def compute_loss(self, pred_y, sim_y, mode = "MSE"):
        if mode == "MSE":
            diff = pred_y - sim_y
            squared_diff = diff ** 2
            mean_diff = squared_diff.mean()            
            return mean_diff


    def get_sampling_bounds(self):
        """
        Return the absolute bounds for the sampeld variables.
        --------
        Outputs:
        --------
            bounds: list of 2D lists
                A list of min and max bounds for each sampling variable        
        """    
        bounds = []

        if self.__config.is_direct_control_sampling:
            # Actuator displacements
            actuator_ranges = list(self.__config.get_actuators_variables().values())
            for i in range(len(actuator_ranges)):
                bounds.append([actuator_ranges[i][1], actuator_ranges[i][2]])

            # Contact displacements
            contact_ranges = list(self.__config.get_contacts_variables().values())
            for i in range(len(contact_ranges)):
                bounds.append([contact_ranges[i][1], contact_ranges[i][2]])

            # Design variables
            design_ranges = list(self.__config.get_design_variables().values())
            for i in range(len(design_ranges)):
                bounds.append([design_ranges[i][1], design_ranges[i][2]])
        else:
            pass
        
        return bounds

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

