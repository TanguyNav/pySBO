import sys
sys.path.append('../src/')
from Problems.SOFA_Compliance import SOFA_Compliance
import numpy as np

""" About the Diamond case:
        The diamond is a soft robot actuated by 4 cables.
        We are interested in moving the end effector of the Diamond
    # The sampling space is the cable actuators length displacement space. 
"""
    

def main():
    # Load model
    model_class = SOFA_Compliance(model_name="Diamond")

    # Get the bounds on the sampling variables
    sampling_bounds = model_class.get_sampling_bounds()
    print("sampling_bounds:", sampling_bounds)

    # Generate a bunch of simulation values
    candidates = np.array([[0, 0, 1, 1], [1, 1, 0, 0], [1, 0, 1, 0], [1, 0, 0, 1]]) 
    time_costs, mechanical_matrices = model_class.perform_real_evaluation(candidates)
    print("Simulation time_costs:", time_costs)
    print("Cand 0", mechanical_matrices[0])

    # Testing the loss computation 
    test_id = 0 
    sim_y = np.concatenate((mechanical_matrices[test_id]["W"].flatten(), mechanical_matrices[test_id]["dfree"]))
    # For this test, we use initial mechanical matrices as the predicted values. 
    # We should use results given by the surrogate in the future
    pred_y = np.concatenate((mechanical_matrices[test_id]["W_0"].flatten(), mechanical_matrices[test_id]["dfree_0"])) 
    loss = model_class.compute_loss(pred_y, sim_y)
    print("Loss for first candidate:", loss)

    
if __name__ == "__main__":
    main()
