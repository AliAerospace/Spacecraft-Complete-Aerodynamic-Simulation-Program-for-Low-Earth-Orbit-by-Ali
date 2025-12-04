# Spacecraft Rarefied Aerodynamics Simulator (LEO, CLL–GSIM Model)

This project is based on my publishing research on spacecraft aerodynamics which will be published on the Journal of Spacecraft and Rockets very soon. This uses a ray-tracing panel method.

**YOU MAY NOT ALTER OR COPY THIS PROGRAM AS IT IS A NOVEL MODEL.**

**IMPORTANCE ❗️ - This program beats the existing methods (industry standard Direct Simulation Monte-Carlo (DSMC)) for low earth orbit aerodynamic simulations by orders of magnitude in terms of speed and efficiency with the results being within a few percent. My program uses the state-of-the-art Cercignani-Lampis-Lord gas surface interaction model (GSIM) in a closed form approximate solution which was not previously possible (more details will be in paper). Other attempts used simpler GSIM's which meant that they could not obtain realistic results with quasi-specular energy accommodation coefficients.**

This is a complete MATLAB-based graphical user interface (GUI) for simulating **rarefied aerodynamic forces and moments (with coefficients)**, **ray-traced molecular shadowing**, and **pressure/shear stress distributions** on spacecraft in Low Earth Orbit (LEO).

This tool visualises any spacecraft geometry, computes drag/lift/side forces, evaluates gas-surface interaction physics using the **Cercignani–Lampis–Lord GSIM model**, and performs a full **3D ray-tracing shadow detection** for free-molecular flow.

⬇️ **SHOWCASE VIDEO AT BOTTOM**

Designed for research, education, and demonstration purposes.
If you have are at a company and wish to use the model then do message me for advice before continuing.

## 🛰️ Features

### **Geometry Handling**
- Load STL models (counter-clockwise indexed vertices required, which is normally done by most programs such as Fusion 360).
- Real-time 3D preview of spacecraft.
- Yaw–Pitch–Roll rotation with interactive controls.
- Ray-tracing visualisation of the portion of the spacecraft that can see molecules (blue) and the other black portion that is shadowed. A shadowed surface feels no force as it is not impinged by any gas molecule directly from the free-stream.

### **Atmospheric Modelling**
- **Automatic mode** using the NRLMSISE-00 upper atmosphere model.
- Space weather inputs from CelesTrak (F10.7, geomagnetic indices) this is contained in the SWLast5Years file.
- Computation of species densities, total freestream density, gas temperature, average molecular mass...

### **Flow & Physical Parameters**
- Automatic orbital velocity using momentum balance.
- Optional manual override for bulk flow velocity.
- Adjustable wall temperature, normal energy accommodation coefficient alpha_n, and tangential energy accommodation coefficient alpha_t.

Force and Moment Predictions
- Drag coefficient CD
- Lift coefficient CL
- Side-force coefficient CS
- Rolling, pitching, and yawing moment coefficients Cl, Cm, Cn
- Body-axis force coefficients CAb, CSb, CNb
- and forces too, see program video below.

### **Ray-Tracing Shadow Solver**
- Panel-wise forward-/backward-facing classification.
- Parallelised ray-casting to detect panel occlusion.
- Shadow caching (skips recomputation for repeated orientations) which **other programs cannot do**.
- Uses the Moller-Trumbore fast ray-triangle intersection check with **custom ray-tracing code**.

### **Pressure & Shear Stress Visualisation**
- Incident pressure distribution
- Reflected pressure distribution
- Tangential shear stress distribution
- Colour-mapped 3D pressure visuals

### **Overview Tab**
Cleanly formatted "Inputs & Outputs Overview":
- Left side: **all user inputs**
- Right side: **all computed aerodynamic outputs**

## 🛠 Installation

### **Requirements**
- MATLAB R2021a or newer  
- Aerospace Toolbox (for NRLMSISE-00 atmospheric model)  
- Parallel Computing Toolbox 

### 🗂️ **INSTALLATION STEPS**
Note: The .stl files are some high-fidelity satellites that I have made myself in Fusion 360 (NASA Aura satellite, GRACE satellite, Mars Microprobe Aeroshell) and some canonical shapes. You must convert the CAD model to triangular discretised panels and save as .stl.
- Download all of the files provided in this repository.
- In your MATLAB folder paste all of the files (excluding Readme file).
- Make a new folder called MATLAB RAYTRACING within the MATLAB folder.
- REMEMBER: WHENEVER YOU LAUNCH THE PROGRAM YOU MUST ENSURE THAT YOU RIGHT CLICK ON EACH FOLDER AND SELECT "Selected Folder(s) and Subfolders"
- 
  <img width="640" height="109" alt="image" src="https://github.com/user-attachments/assets/091f90b9-9b9f-4a6b-8a78-efcf2984b482" />\

- The reason for this MATLAB RAYTRACING folder is for the program to save the computed shadows in it to avoid excessive wait times later on.
- Open the LEOAEROSIMbyAliAerospace.m file within MATLAB and scroll down to (look at image below):
- 
  <img width="937" height="162" alt="image" src="https://github.com/user-attachments/assets/7dc1a9b2-6dd0-4955-855b-360aeab8a53a" />
  
and in here replace "/Users/ali/Documents/MATLAB" with your file location which can be found by simply double clicking here:
<img width="443" height="275" alt="image" src="https://github.com/user-attachments/assets/921ed5b9-3342-4ffd-985f-da5d0bf4fadc" />
- Hit RUN and enjoy using the program.

### 🎥 DEMONSTRATION - 1 min video:

Note: in this video I had ran previously ran the 30deg yaw, which is why it computed so quickly. With a decent computer, around 13000 panels computes in a few seconds. I could make the ray-tracing even faster but for now it provides results that are very good so that may come in a future update.

Extra Note: If you wish to alter the date/time/orbital parameters these can be altered in the code here. In an update it will be added to the UI:

<img width="916" height="165" alt="image" src="https://github.com/user-attachments/assets/21f58a86-c9bd-410d-8b42-a6d04e1aa4bf" />




https://github.com/user-attachments/assets/e2c71994-0bb8-4828-8ad6-d59a18a0750f










  


