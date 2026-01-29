# UAV Strategic Deconfliction in Shared Airspace

## Overview
This project implements a **Strategic Deconfliction System for Unmanned Aerial Vehicles (UAVs)**.
The system verifies whether a drone’s planned waypoint mission is safe to execute in shared
airspace by checking **spatial and temporal conflicts** with other drones **before takeoff**.

Both **2D (x, y, time)** and **3D / 4D (x, y, z, time)** simulations are implemented, along with
visualizations to clearly show conflict and no-conflict scenarios.

---

## Objectives
- Perform **pre-flight conflict checking** for UAV missions
- Detect conflicts in **space and time**
- Provide clear conflict explanations
- Visualize drone trajectories and conflict points
- Demonstrate scalability concepts for real-world UAV traffic management

---

## Features
- Waypoint-based drone missions
- Temporal interpolation for continuous collision detection
- Safety distance (minimum separation buffer)
- Multiple-drone conflict detection
- Robust edge-case handling
- 2D and 3D visualizations

---

## Project Structure
