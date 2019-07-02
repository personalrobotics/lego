# LEGO 
The repository follows along with the paper LEGO: Leveraging Experience in Roadmap
Generation for Sampling-Based Planning by Rahul Kumar, Aditya Mandalika, Sanjiban Choudhury and Siddhartha S. Srinivasa of Personal Robotics Lab, University of Washington, submitted to IROS 2019.

An example procedure has been provided in `Model-LEGO.ipynb` for a 2D environment represented as an occupancy grid. The model learns samples along bottleneck and diverse regions to maximize the likelihood of a feasible path while maintaining bounded sub-optimality. We train a CVAE model with the target samples conditioned on the problem's initial state, goal state, and an occupancy grid of the obstacles. The dense graph and the dataset required have also been provided.
