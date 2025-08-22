# GRN-SMC: Predictive Entrapment in Swarm Robotics

This repository contains the implementation of GRN-SMC (Gene Regulatory Network with Sliding Mode Control), a predictive entrapment control algorithm for swarm robotic systems.
The approach addresses challenges of slow convergence and formation maintenance in time-varying target entrapment tasks, by combining hierarchical GRN-based prediction with a sliding mode controller for stable and robust swarm coordination.The project includes three MATLAB simulation scenarios and two Python simulation scenarios, which demonstrate predictive entrapment under static and dynamic obstacle environments.

The supplementary video is available at: https://youtu.be/JialCK84jfA?si=xZuN0fif10IyU1zF

🌈If you use this code, please cite our paper. Please hit the star at the top-right corner. Thanks!


📂 Project Structure

├── Matlab_three_scenes/        # MATLAB-based simulation scenarios
│   ├── main1_tunnel_obs.m       # Scenario 1: Predictive entrapment in a tunnel with obstacles
│   ├── main2_free_obs.m         # Scenario 2: Predictive entrapment in an open space with obstacle free
│   ├── main3_dynamic_obs.m      # Scenario 3: Predictive entrapment with dynamic obstacles
│   ├── Sliding_model_controller.m
│   ├── point_ANN.m
│   └── (other utility functions for pattern generation, obstacle generation, etc.)
│
├── Python_two_scenes/          # Python-based simulation scenarios
│   ├── main_ori.py             # Scenario A: Baseline entrapment with static obstacles
│   ├── main_cir.py             # Scenario B: Entrapment of a circular-moving (time-varying) target
│   ├── hunter.txt              # Initial hunter robot positions
│   ├── target.txt              # Target trajectory
│   ├── obstacles.txt
│   ├── animation.gif           # Simulation result animation
│   ├── GRN-SMC_Dv.txt          # Output: entrapment performance metric (Dv)
│   ├── GRN_SMC_time_array.txt  # Output: runtime performance per timestep
│   └── UAV.png                 # UAV icon for visualization

⚙️ Requirements

MATLAB Scenarios

MATLAB R2020b or later (recommended)

No external toolboxes required, but Neural Network Toolbox may be used in some scripts.

Python Scenarios

Python 3.8+

Required packages:

pip install numpy pandas matplotlib

▶️ Usage

1. Run MATLAB Scenarios

Open MATLAB, navigate to Matlab_three_scenes/, and run one of the main scripts:

main1_tunnel_obs    % Scenario 1: Tunnel entrapment with obstacles
main2_free_obs      % Scenario 2: Open-space entrapment with obstacles
main3_dynamic_obs   % Scenario 3: Entrapment under dynamic obstacles


These simulations demonstrate predictive swarm entrapment under different obstacle environments.

2. Run Python Scenarios

Navigate to Python_two_scenes/ and execute:

python main_ori.py   # Scenario A: Baseline entrapment with static obstacles
python main_cir.py   # Scenario B: Time-varying target entrapment

📊 Results

MATLAB Scenarios:
Demonstrate predictive swarm entrapment under tunnel, open field, and dynamic obstacle conditions.

Python Scenarios:
Provide lightweight demonstrations of entrapment dynamics, including time-varying targets and performance evaluation.

📌 Citation

If you use this code in your research, please cite:

@article{wen2025time,
  title={Time-varying Target Predictive Entrapment Based on Gene Regulatory Network and Sliding Mode Control},
  author={Wen, Ziling and Wang, Dongliang and Wang, Zhaojun and Huang, Dawei and Yang, Binghao and Li, Wenji and Fan, Zhun and Zou, An-Min},
  journal={IEEE Internet of Things Journal},
  year={2025},
  publisher={IEEE}

}

