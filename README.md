<div align="left">
<h1 align="left">
<br>GNSS-DR_LOCO</h1>
<h3> Developed with the software and tools below.</h3>
<p align="left">
</p>
<img src="https://img.shields.io/github/license/alstondu/GNSS-DR_Loco?style=flat-square&color=5D6D7E" alt="GitHub license" />
<img src="https://img.shields.io/github/last-commit/alstondu/GNSS-DR_Loco?style=flat-square&color=5D6D7E" alt="git-last-commit" />
<img src="https://img.shields.io/github/commit-activity/m/alstondu/GNSS-DR_Loco?style=flat-square&color=5D6D7E" alt="GitHub commit activity" />
<img src="https://img.shields.io/github/languages/top/alstondu/GNSS-DR_Loco?style=flat-square&color=5D6D7E" alt="GitHub top language" />
</div>

---

## 📖 Table of Contents
- [📖 Table of Contents](#-table-of-contents)
- [📍 Overview](#-overview)
- [📦 Workflow](#-workflow)
- [📂 repository Structure](#-repository-structure)
- [⚙️ Modules](#-modules)
- [🔧 Installation](#-installation)
- [🤖 Running GNSS-DR_Loco](#-running-gnss-dr_loco)
- [📄 License](#-license)

---
## 📍 Overview

This project implements integrated navigation, GNSS and Dead-reckoning, for a robotic lawnmower.

---

## 📦 Workflow

1. Determine the initial position and velocity of the lawnmower based on iterative least squares using
data from GNSS.

2. Determine the position and velocity of the lawnmower at each time based on Kalman filtering using
data from GNSS.

3. Generate the corrected heading of the lawnmower based on Kalman filtering using data from a gyro-
scope and magnetic compass.

4. Model the speed model. Determine the position and the velocity of the lawnmower at each time using
data from wheel speed sensors with the Dead-Reckoning approach.

5. Fuse the data from step 2 and 4 based on Kalman filtering and integrate DR/GNSS to get the final
result

  <div align="center">
    <img width="60%" src="https://github.com/alstondu/GNSS-DR_Loco/blob/main/fig/flow%20chart.png"></a>
  </div>

---


## 📂 Repository Structure

```sh
└── GNSS-DR_Loco/
    ├── CTM_to_Euler.m
    ├── DR.mat
    ├── DR_Solution.m
    ├── Data/
        ├── Dead_reckoning.csv
        ├──Pseudo_ranges.csv
        ├──Pseudo_range_rates.csv
        ├──Integrated_result.csv
    ├── Define_Constants.m
    ├── ECEF_to_NED.m
    ├── Euler_to_CTM.m
    ├── GNDD_DR_Fusion.m
    ├── GNSS.mat
    ├── GNSS_KF.m
    ├── Gravity_ECEF.m
    ├── Integrated_Heading.m
    ├── Integrated_Heading.mat
    ├── LS_Init.m
    ├── LS_init.mat
    ├── NED_to_ECEF.m
    ├── Radii_of_curvature.m
    ├── Satellite_position_and_velocity.m
    ├── Skew_symmetric.m
    ├── pv_ECEF_to_NED.m
    └── pv_NED_to_ECEF.m

```

---
## ⚙️ Modules
<details closed><summary>Root</summary>

| File                                                                                                                      | Summary                   |
| ---                                                                                                                       | ---                       |
| [Define_Constants.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Define_Constants.m)                               | Defined constants for use |
| [pv_NED_to_ECEF.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/pv_NED_to_ECEF.m)                                   | Converts curvilinear to Cartesian position and velocity resolving axes from NED to ECEF |
| [GNDD_DR_Fusion.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/GNDD_DR_Fusion.m)                                   | Integrate DR/GNSS solution based on Kalman Filter |
| [Integrated_Heading.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Integrated_Heading.m)                           | Integrates gyroscope and magnetometer based on Kalman Filter |
| [NED_to_ECEF.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/NED_to_ECEF.m)                                         | Converts curvilinear to Cartesian position, velocity resolving axes from NED to ECEF and attitude from NED- to ECEF-referenced |
| [Satellite_position_and_velocity.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Satellite_position_and_velocity.m) | Returns ECEF Cartesian positions and ECEF velocities for one satellite |
| [LS_Init.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/LS_Init.m)                                                 | Initialises position and velocity at time=0 using least-queare algorithm |
| [Euler_to_CTM.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Euler_to_CTM.m)                                       | Converts a set of Euler angles to the corresponding coordinate transformation matrix |
| [ECEF_to_NED.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/ECEF_to_NED.m)                                         | Converts Cartesian  to curvilinear position, velocity resolving axes from ECEF to NED and attitude from ECEF- to NED-referenced |
| [Gravity_ECEF.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Gravity_ECEF.m)                                       | Calculates  acceleration due to gravity resolved about ECEF-frame |
| [DR_Solution.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/DR_Solution.m)                                         | Calculates pos and vel with Dead-reckoning method|
| [pv_ECEF_to_NED.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/pv_ECEF_to_NED.m)                                   |Converts Cartesian  to curvilinear position and velocity resolving axes from ECEF to NED |
| [CTM_to_Euler.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/CTM_to_Euler.m)                                       | Converts a coordinate transformation matrix to the corresponding set of Euler angles|
| [GNSS_KF.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/GNSS_KF.m)                                                 | Calculates pos amd vel with Kalman filter based GNSS and outlier detection |
| [Radii_of_curvature.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Radii_of_curvature.m)                           | Calculates the meridian and transverse radii of curvature |
| [Skew_symmetric.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Skew_symmetric.m)                                   | Calculates skew-symmetric matrix |

</details>

---
### 🔧 Installation

Clone the GNSS-DR_Loco repository:
```sh
git clone https://github.com/alstondu/GNSS-DR_Loco
```

---
### 🤖 Running GNSS-DR_Loco

1. Run [LS_Init.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/LS_Init.m) to obtain initial data

2. Run [Integrated_Heading.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Integrated_Heading.m) to get fused heading
   
3. Run [GNSS_KF.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/GNSS_KF.m) to get GNSS solution
   
4. Run [DR_Solution.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/DR_Solution.m) to get Dead-reckoning solution
   
5. Run [GNDD_DR_Fusion.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/GNDD_DR_Fusion.m) to get final integrated solution

---

## 📄 License


This project is protected under the [MIT](https://github.com/alstondu/GNSS-DR_Loco/blob/main/LICENSE) License.


