# -*- coding: utf-8 -*-
"""Base config class to save config data.
"""

__authors__ = "emenager, tnavez"
__contact__ = "etienne.menager@inria.fr, tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Jun 29 2022"

class BaseConfig(object):

    def __init__(self, model_name):
        self.model_name = model_name
        self.scene_name = model_name
        self.generation_script_name = "Generation"
        self.n_robot = 1

        self.is_inverse = False
        self.init_model_parameters()
        self.scene_config = {"inverseMode": self.is_inverse,
                             "goalPos": None,
                             "is_force": False}

        # Specific attribute for managing evolving design data acquisition
        self.in_data_acquisition_loop = False
        self.is_direct_control_sampling = True

    def get_scene_name(self):
        """
        Return the name of the model SOFA simulation scene

        Outputs
        ----------
        name : str
            Name of the SOFA scene
        """
        return self.scene_name


    #########################################################
    ### Methods for managing traveling in actuation space ###
    #########################################################
    def get_actuators_variables(self):
        """
        Return a dictionnary {key = name_act, value = [init_value, min_value, max_value]} for each actuator variable.

        Outputs
        ----------
        name_act : str
            Name of the actuator. It must be the same in the associated SOFA scene.
        init_value : float
            Initial value for the actuator.
        min_value : float
            Minimum value for the actuator.
        max_value : float
            Maximum value for the actuator.
        """
        return {}

    def get_contacts_variables(self):
        """
        Return a dictionnary {key = name_cont, value = [init_value, min_value, max_value]} for each contact variable.
        There is 1 to list of values to provide depending on the dimension considered for the contact point (1D, 2D or 3D).

        Outputs
        ----------
        name_cont : str
            Name of the contact var. It must be the same in the associated SOFA scene.
            We use the PointEffector actuator component for representing contacts in the SOFA scene.
        init_value : float
            Initial values for the contact displacement.
        min_value : float
            Minimum values for the contact displacement.
        max_value : float
            Maximum values for the contact displacement.
        """
        return {}

    def get_inverse_variables(self):
        """
        Return a dictionnary {key = name_inv, value = [init_value, min_value, max_value]} for each inverse variable.
        The inverse variable is the variable defining the position / orientation of the effector in an inverse problem.

        Outputs
        ----------
        name_inv : str
            Name of the inverse var. We have to handle it in the controler of the SOFA scene.
        init_value : float
            Initial values for the variable (position, orientation).
        min_value : float
            Minimum values for the variable (position, orientation).
        max_value : float
            Maximum values for the variable (position, orientation).
        """
        return {}

    def interpolate_variables(self, normalized_values, var_type = "actuation"):
        """
        Compute bounds interpolation of [0,1] to [min_value, max_value] for a list of actuation values.
        This method should be reimplemented if using a different sampling strategy.
        ----------
        Parameters
        ----------
        normalized_values: list of float
            Normalized values for the actuator in [0,1].
        var_type: string in {"actuation", "contact"}
            Specify if the variable is an actuation or contact variable.
        ----------
        Outputs
        ----------
        values: list of float
            Value for the actuator in [min_value, max_value].
        """
        if var_type == "contact":
            variables = list(self.get_contacts_variables().values())
        elif var_type == "design": 
            variables = list(self.get_design_variables().values())
        elif var_type == "inverse":
            variables = list(self.get_inverse_variables().values())
        else: #Actuation by default
            variables = list(self.get_actuators_variables().values())
        values = [normalized_values[i] * (variables[i][2] - variables[i][1]) + variables[i][1] for i in range(len(normalized_values))]
        return values


    ######################################################
    ### Methods for managing traveling in design space ###
    ######################################################
    def init_model_parameters(self):
        """
        This function implement initialization of model parameters
        """
        pass

    def get_design_variables(self):
        """
        Return a dictionnary {key = name, value = [value, min_value, max_value]} for each design variable.
        ----------
        Outputs
        ----------
        name : str
            Name of the design variable.
        value : float
            Value for the design variable.
        min_value : float
            Minimum value for the design variable.
        max_value : float
            Maximum value for the design variable.
        """
        return {}


    def set_design_variables(self, new_values):
        """
        Set new values for design variables.
        ----------
        Inputs
        ----------
        new_values: list of float
            New value for each design variables. The values are in the same order as in get_design_variables().
        """
        for i, name in enumerate(self.get_design_variables().keys()):
            if new_values[i] >= self.get_design_variables()[name][1] and new_values[i] <= self.get_design_variables()[name][2]:
                setattr(self, name, new_values[i])
            else:
                print("Error: assigned new value for design variable are out of bounds.")

    #############################################
    ### Methods for managing data acquisition ###
    #############################################
    def get_n_sampling_variables(self):
        """
        Return the number of variables used for sampling.
        By default this number is the number of constraint.
        This method should be reimplemented if using a different sampling strategy.

        Outputs
        ----------
        n_sampling_vars : int
            Number of samplign variables.
        """
        return len(self.get_actuators_variables()) + len(self.get_contacts_variables()) + len(self.get_design_variables())


    #######################################
    ### Methods for managing simulation ###
    #######################################
    def get_scene_name(self):
        """
        Return the name of the model SOFA simulation scene

        Outputs
        ----------
        name : str
            Name of the SOFA scene
        """
        return self.scene_name

    def get_scene_config(self):
        """
        Return a configuration dictionnary for the scene.

        Outputs
        ----------
        scene_config : dic
            Configuration dictionnary of the scene (keys = name of the parameters, value = the
            value of the parameters).
        """
        return self.scene_config

    def set_scene_config(self, config):
        """
        Update the scene_config with the config.

        Parameters
        ----------
        config : dic
            Configuration dictionnary to include in scene_config.
        """
        self.scene_config.update(config)

    def set_is_inverse(self):
        """
        Set the scene in inverse mode.
        """
        self.is_inverse = True
        self.set_scene_config({"inverseMode": self.is_inverse})
        
    def set_action_type(self, is_force = False):
        """
        Set the value type for the actuators in the scene
        
        Parameters:
        -----------
            is_force: bool
                Wether actuators are controlled in force or displacement.
        """
        self.set_scene_config({"is_force": is_force})
    
    
    def set_goalPos(self, pos):
        """
        Set the goalPos of the scene.

        Parameters:
        -----------
            pos: list of float
                The new position of the goal.
        """
        self.set_scene_config({"goalPos": pos})

    @staticmethod
    def get_n_eq_dt(self):
        """
        Return the number of dt step to reach equilibrium.
        Is usefull for robots subjets to other than actuation/collision forces such as gravity.

        Outputs:
        -----------
            n_eq_dt: int
                The number of dt steps.
        """
        return None

    @staticmethod
    def get_n_dt(self):
        """
        Return the number of dt step for a simulation evaluation.

        Outputs:
        -----------
            n_dt: int
                The number of dt steps.
        """
        return None

    def get_post_sim_n_eq_dt(self):
        """
        Return the number of dt step to wait after simulation.
        This number is 0 by default.

        Outputs:
        -----------
            post_sim_n_eq_dt: int
                The number of dt steps to wait after simulation.
        """
        return 10

    @staticmethod
    def get_trajectory(self):
        """
        Return a trajectory to be performed by the robot.

        Outputs:
        -----------
            goals: list of list of arrays
                List of successive goals describing a trajectory of the robot.
        """
        return None
    
    
