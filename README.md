Network-Scale Traffic Signal Prioritization System for Emergency Vehicles
DOI MATLAB PTV VISSIM License: GPL v3

📌 Project Description
This repository contains the source code, algorithms, and simulation datasets developed as part of the doctoral thesis "Emergent traffic signal systems for the circulation of priority vehicles".

The algorithmic framework proposes a dynamic, integrated network-scale prioritization system for emergency vehicles, overcoming the structural limitations of conventional hard-preemption approaches. The system is composed of three main modular components:

Topological Extraction: Deterministic generation of directional adjacency lists at the individual traffic light level.
Conflict Detection: Automatic topological identification of signal incompatibilities (convergence and crossing conflicts) without manual configuration.
Extended Green Wave: A dynamic hierarchical control algorithm that establishes expansive priority corridors in real-time and manages a "red line" for the preventive flushing of civil traffic.
The system has been empirically validated across three highly complex urban topologies: the orthogonal grid of Boise (USA), the dense topological mesh of Luxembourg, and the irregular historical layout of London (UK).

📂 Repository Structure
📦 Emerging-Priority-System
 ┣ 📂 data/                 # Base topologies (.inpx files) and spatial matrices (Boise, Luxembourg, London)
 ┣ 📂 src/                  # Source code (MATLAB)
 ┃ ┣ 📁 01_adjacency/       # Scripts for the extraction of directional topological relationships
 ┃ ┣ 📁 02_conflicts/       # Parametric topological algorithms for conflict graphs
 ┃ ┗ 📁 03_control_loop/    # COM interface and real-time control loop for the dynamic green wave
 ┣ 📂 results/              # Output datasets, kinematic logs, and macroscopic metrics
 ┣ 📄 README.md             # Project documentation
 ┗ 📄 LICENSE               # GNU GPLv3 Open-source license

⚙️ Requirements and Dependencies
To reproduce the simulations and execute the algorithms, the following software is required:

MATLAB (R202X or higher recommended) with COM integration support.

PTV VISSIM (Research/Commercial License) to run the dynamic microscopic traffic simulation engine.

🚀 Usage and Implementation
Clone this repository to your local machine:

Bash
git clone https://github.com/Sergio-Rojas-Blanco/Emerging-Priority-System.git
Open MATLAB and set the root directory of the repository as your Current Folder.

Execute the topological extraction scripts in src/01_adjacency/ to precalculate the spatial structures.

Run the main simulation loop in src/03_control_loop/ to initialize the VISSIM COM server and deploy the extended green wave.

📊 Key Findings
The integration of the extended green wave has empirically demonstrated:

Emergency Vehicle Travel Time Reduction: Up to a 57.7% net reduction in meshed environments, stabilizing cruising speeds regardless of civil traffic saturation.

Systemic Decompression: An increase in the global average speed of the civil network by up to 28.1% under moderate congestion conditions, reversing the classic paradigm of transverse disruption.

✒️ Author & Affiliation
Sergio Rojas Blanco University of Cádiz (UCA)

Department of Mechanical Engineering and Industrial Design

📖 Citation
If you use this code, the topological algorithms, or the network datasets in your research, please cite the repository and the corresponding doctoral thesis using the following IEEE format:

[1] S. Rojas Blanco, "Network-Scale Traffic Signal Prioritization System for Emergency Vehicles", Emerging-Priority-System GitHub Repository, 2026. [Online]. Available: https://doi.org/10.5281/zenodo.XXXXXXX

📜 License
This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

This is a strong copyleft license. You may copy, distribute, and modify the software as long as you track changes/dates in source files. Any modifications to or software including (via compiler) GPL-licensed code must also be made available under the GPL along with build & install instructions. See the LICENSE file for details.
