import sys
sys.path.append('../src/')
from Problems.SOFA_Compliance import SOFA_Compliance
import numpy as np

def main():
    p = SOFA_Compliance(model_name="Trunk")
    print(p)

    candidates = np.array([[0, 0, 1, 1, 0, 0, 1, 0], [0, 0, 1, 1, 0, 1, 1, 0]]) # Trunk
    objectives = p.perform_real_evaluation(candidates)
    print("objectives:", objectives)

if __name__ == "__main__":
    main()
