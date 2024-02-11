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
- [📦 Features](#-features)
- [📂 repository Structure](#-repository-structure)
- [⚙️ Modules](#modules)
- [🚀 Getting Started](#-getting-started)
    - [🔧 Installation](#-installation)
    - [🤖 Running GNSS-DR_Loco](#-running-GNSS-DR_Loco)
    - [🧪 Tests](#-tests)
- [🛣 Roadmap](#-roadmap)
- [🤝 Contributing](#-contributing)
- [📄 License](#-license)
- [👏 Acknowledgments](#-acknowledgments)

---


## 📍 Overview

This project implements integrated navigation, GNSS and Dead-reckoning, for a robotic lawnmower.


---

## 📦 Features

HTTPStatus Exception: 404

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
| [LS_Init.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/LS_Init.m)                                                 | HTTPStatus Exception: 404 |
| [Euler_to_CTM.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Euler_to_CTM.m)                                       | HTTPStatus Exception: 404 |
| [ECEF_to_NED.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/ECEF_to_NED.m)                                         | HTTPStatus Exception: 404 |
| [Gravity_ECEF.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Gravity_ECEF.m)                                       | HTTPStatus Exception: 404 |
| [DR_Solution.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/DR_Solution.m)                                         | HTTPStatus Exception: 404 |
| [pv_ECEF_to_NED.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/pv_ECEF_to_NED.m)                                   | HTTPStatus Exception: 404 |
| [CTM_to_Euler.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/CTM_to_Euler.m)                                       | HTTPStatus Exception: 404 |
| [GNSS_KF.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/GNSS_KF.m)                                                 | HTTPStatus Exception: 404 |
| [Radii_of_curvature.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Radii_of_curvature.m)                           | HTTPStatus Exception: 404 |
| [Skew_symmetric.m](https://github.com/alstondu/GNSS-DR_Loco/blob/main/Skew_symmetric.m)                                   | HTTPStatus Exception: 404 |

</details>

---
### 🔧 Installation

Clone the GNSS-DR_Loco repository:
```sh
git clone https://github.com/alstondu/GNSS-DR_Loco
```

---
### 🤖 Running GNSS-DR_Loco

```sh
./myapp
```

---

## 📄 License


This project is protected under the [MIT](https://github.com/alstondu/GNSS-DR_Loco/blob/main/LICENSE) License.


